/*
 * S.E.N.S.E. - ESP32 Earthquake & Smoke Emergency Door-Unlock System
 *              with Built-in Wi-Fi Phone Monitor Web App
 *
 * Hardware:
 *   - Input 1 : ADXL345 (I2C 3-axis Accelerometer) -> Earthquake / Seismic detection
 *   - Input 2 : JGN-301-A Smoke Detector -> Interfaced via PC817 optocoupler
 *               (or dry-contact relay if using a relay-modified unit)
 *   - Output  : Electric Doorknob / Solenoid Lock via Relay Module (+ flyback diode)
 *   - Controls: Manual Reset/Lock Button (GPIO 14) & Built-in Status LED (GPIO 2)
 *
 * Phone Monitoring:
 *   - Connects to your Wi-Fi router (configure WIFI_SSID / WIFI_PASSWORD below), OR
 *     automatically creates its own Wi-Fi hotspot ("SENSE-Monitor" / "12345678")
 *     accessible on your phone browser at http://192.168.4.1
 *   - Supports "Add to Home Screen" on Android/iOS as a web app.
 *
 * Libraries:
 *   - Uses built-in Wire.h, WiFi.h, and WebServer.h only (no external libraries).
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// -------------------- Wi-Fi Configuration --------------------
// Leave as "YOUR_WIFI_SSID" to immediately use Hotspot (AP) mode at 192.168.4.1,
// or enter your router credentials to join your local Wi-Fi network.
constexpr char WIFI_SSID[]     = "YOUR_WIFI_SSID";
constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

// Fallback Hotspot (AP) credentials when router Wi-Fi is not configured/available
constexpr char AP_SSID[]       = "SENSE-Monitor";
constexpr char AP_PASSWORD[]   = "12345678";

// -------------------- Pin Definitions --------------------
constexpr uint8_t PIN_I2C_SDA    = 21;
constexpr uint8_t PIN_I2C_SCL    = 22;
constexpr uint8_t PIN_SMOKE_IN   = 32;  // Digital input from JGN-301-A (via optocoupler)
constexpr uint8_t PIN_DOOR_RELAY = 25;  // Drives relay/MOSFET for Electric Doorknob
constexpr uint8_t PIN_RESET_BTN  = 14;  // Pushbutton to GND: re-lock & clear alarm
constexpr uint8_t PIN_LED_STATUS = 2;   // Built-in LED on ESP32 DevKit

// -------------------- Logic Polarity Config --------------------
// Most JGN-301-A + PC817 optocoupler circuits pull the ESP32 pin LOW on alarm.
// Change to HIGH if your interface outputs 3.3V when smoke is detected.
constexpr uint8_t SMOKE_ACTIVE_LEVEL = LOW;

// Most 1-channel relay modules for ESP32/Arduino are Active-LOW (LOW = energized).
// - Fail-secure lock (unlocks when powered): RELAY_UNLOCK_LEVEL = LOW
// - Fail-safe / normally-free electric doorknob (unlocked when unpowered): RELAY_UNLOCK_LEVEL = HIGH
constexpr uint8_t RELAY_UNLOCK_LEVEL = LOW;
constexpr uint8_t RELAY_LOCK_LEVEL   = (RELAY_UNLOCK_LEVEL == LOW) ? HIGH : LOW;

// -------------------- ADXL345 Registers --------------------
constexpr uint8_t ADXL345_ADDR        = 0x53;  // Use 0x1D if SDO/ALT pin is tied to 3.3V
constexpr uint8_t REG_DEVID           = 0x00;  // Expected value: 0xE5
constexpr uint8_t REG_BW_RATE         = 0x2C;
constexpr uint8_t REG_POWER_CTL       = 0x2D;
constexpr uint8_t REG_DATA_FORMAT     = 0x31;
constexpr uint8_t REG_DATAX0          = 0x32;
constexpr float   ADXL_G_PER_LSB      = 0.00390625f;  // Full-resolution mode (~3.9 mg/LSB)

// -------------------- Detection & Safety Tuning --------------------
constexpr float    QUAKE_THRESHOLD_G   = 0.035f; // Detects light tremors (~M4.3+) through major quakes
constexpr uint8_t  QUAKE_HITS_REQUIRED = 4;      // Tremor samples required within window
constexpr uint32_t QUAKE_WINDOW_MS     = 1500;   // Filters out single accidental knocks
constexpr uint32_t QUAKE_SAMPLE_MS     = 25;     // 40 Hz seismic sampling

constexpr uint32_t SMOKE_DEBOUNCE_MS   = 40;     // Confirm real alarm pulse
constexpr uint32_t SMOKE_HOLD_MS       = 8000;   // Bridge gaps between JGN-301-A beeps

// Normal state: door starts UNLOCKED so people can open and close it normally
constexpr uint32_t TELEMETRY_MS         = 1000;
constexpr uint32_t DATA_LOG_INTERVAL_MS = 10000;  // Periodic log every 10 seconds
constexpr uint16_t DATA_LOG_CAPACITY    = 120;    // Ring buffer capacity in RAM

// -------------------- State Variables --------------------
WebServer server(80);
bool usingApMode = false;
bool adxlReady   = false;

// Low-pass gravity baseline per axis
float baseAx = 0.0f, baseAy = 0.0f, baseAz = 1.0f;
float currentTremorG = 0.0f;
float peakTremorG    = 0.0f;

// Hazard states
uint8_t  quakeHits        = 0;
uint32_t quakeWindowStart = 0;
uint32_t lastQuakeMs      = 0;
bool     quakeActive      = false;

uint32_t smokeActiveStart = 0;
uint32_t lastSmokeSeenMs  = 0;
bool     smokeActive      = false;

bool     doorUnlocked     = true;  // Default: free for normal opening/closing
uint32_t lastHazardMs     = 0;

// Loop timers
uint32_t lastQuakeSampleMs = 0;
uint32_t lastTelemetryMs   = 0;
uint32_t lastDataLogMs     = 0;

// Periodic & event data log ring buffer
struct DataLogEntry {
  uint32_t uptimeSec;
  float    currentG;
  float    peakG;
  float    estMag;
  char     mmi[4];
  char     tag[10];
  bool     quakeActive;
  bool     smokeActive;
  bool     doorUnlocked;
};

DataLogEntry dataLogs[DATA_LOG_CAPACITY];
uint16_t dataLogHead  = 0;  // Next write index
uint16_t dataLogCount = 0;  // Number of stored entries (up to DATA_LOG_CAPACITY)
uint32_t totalLogged  = 0;

// -------------------- Embedded Mobile Web App --------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0, viewport-fit=cover" />
  <meta name="theme-color" content="#000000" />
  <meta name="apple-mobile-web-app-capable" content="yes" />
  <link rel="manifest" href="/manifest.webmanifest" />
  <title>S.E.N.S.E. Monitor</title>
  <style>
    :root {
      --bg: #000000;
      --panel: #dce2ea;
      --inner: #0f172a;
      --text: #0f172a;
      --dark-text: #0b0f17;
      --muted: #64748b;
      --ok: #16a34a;
      --warn: #facc15;
      --danger: #dc2626;
      --accent: #38bdf8;
    }
    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    html, body { margin: 0; min-height: 100dvh; background: var(--bg); color: var(--text); overflow-x: hidden; }
    body { padding: calc(14px + env(safe-area-inset-top)) 16px calc(16px + env(safe-area-inset-bottom)); display: flex; justify-content: center; }
    .app { width: 100%; max-width: 920px; display: flex; flex-direction: column; gap: 14px; }
    .topbar { display: flex; align-items: center; justify-content: space-between; gap: 10px; }
    .top-left { display: flex; align-items: center; gap: 8px; }
    .menu-btn { width: 48px; height: 44px; border: 1px solid #cbd5e1; border-radius: 999px; background: var(--panel); color: var(--dark-text); font-size: 1.35rem; font-weight: 800; display: inline-flex; align-items: center; justify-content: center; cursor: pointer; }
    .brand-pill { height: 44px; padding: 0 18px; border: 1px solid #cbd5e1; border-radius: 999px; background: var(--panel); color: var(--dark-text); font-size: 0.92rem; font-weight: 800; letter-spacing: 0.03em; display: inline-flex; align-items: center; }
    .status-pill { height: 44px; padding: 0 22px; border-radius: 999px; font-size: 0.9rem; font-weight: 800; letter-spacing: 0.04em; display: inline-flex; align-items: center; justify-content: center; background: var(--danger); color: #ffffff; }
    .status-pill.live { background: var(--ok); color: #ffffff; }
    .status-pill.err { background: var(--danger); color: #ffffff; }
    .monitor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; flex: 1; }
    .outer-card { background: var(--panel); border: 1px solid #cbd5e1; border-radius: 24px; padding: 12px; display: flex; flex-direction: column; gap: 10px; }
    .title-pill { background: #000000; color: #ffffff; border-radius: 999px; padding: 9px 14px; font-size: 0.88rem; font-weight: 700; text-align: center; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .status-box { background: var(--ok); color: #ffffff; border-radius: 18px; padding: 22px 12px; min-height: 185px; flex: 1; display: flex; flex-direction: column; align-items: center; justify-content: center; text-align: center; gap: 8px; transition: background 0.25s, color 0.25s, box-shadow 0.25s; }
    .status-box.ok { background: var(--ok); color: #ffffff; }
    .status-box.warn { background: var(--warn); color: #111827; }
    .status-box.danger { background: var(--danger); color: #ffffff; animation: boxPulse 1.2s infinite; }
    @keyframes boxPulse { 0%,100% { box-shadow: 0 0 0 0 rgba(220,38,38,0.45); } 50% { box-shadow: 0 0 0 8px rgba(220,38,38,0); } }
    .big-state { font-size: clamp(1.15rem, 3.6vw, 2rem); font-weight: 800; letter-spacing: 0.03em; line-height: 1.15; }
    .sub-metrics { font-size: clamp(0.75rem, 2.2vw, 0.92rem); line-height: 1.4; opacity: 0.95; font-weight: 600; }
    .sub-dim { font-size: 0.74rem; opacity: 0.85; }
    .door-card { background: var(--panel); border: 1px solid #cbd5e1; border-radius: 24px; padding: 12px; display: flex; flex-direction: column; gap: 10px; }
    .door-status-pill { background: #000000; color: #ffffff; border-radius: 999px; padding: 10px 18px; font-size: 0.9rem; font-weight: 700; display: inline-flex; align-items: center; gap: 10px; width: fit-content; max-width: 100%; }
    .door-state-word { font-weight: 800; color: #4ade80; }
    .door-state-word.warn { color: #facc15; }
    .door-state-word.danger { color: #f87171; }
    .door-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .pill-btn { background: #000000; color: #ffffff; border: 0; border-radius: 999px; padding: 12px 14px; min-height: 46px; font-size: 0.9rem; font-weight: 700; cursor: pointer; }
    .pill-btn:disabled { opacity: 0.45; cursor: not-allowed; }
    .backdrop { position: fixed; inset: 0; background: rgba(15,23,42,0.45); opacity: 0; pointer-events: none; transition: opacity 0.25s ease; z-index: 40; }
    .backdrop.open { opacity: 1; pointer-events: auto; }
    .sidebar { position: fixed; top: 0; left: 0; bottom: 0; width: min(330px, 86vw); background: #f8fafc; color: #0f172a; border-right: 1px solid #cbd5e1; padding: calc(16px + env(safe-area-inset-top)) 14px calc(18px + env(safe-area-inset-bottom)); transform: translateX(-100%); transition: transform 0.25s ease; z-index: 50; display: flex; flex-direction: column; gap: 12px; overflow-y: auto; touch-action: pan-y; }
    .sidebar.dragging { transition: none; }
    .sidebar.open { transform: translateX(0); }
    .sb-hdr { display: flex; justify-content: space-between; align-items: center; padding-bottom: 8px; border-bottom: 1px solid #cbd5e1; }
    .sb-hdr h2 { margin: 0; font-size: 1rem; letter-spacing: 0.04em; }
    .sb-close { background: #e2e8f0; color: #0f172a; border: 1px solid #cbd5e1; border-radius: 10px; padding: 6px 12px; font-weight: 700; cursor: pointer; }
    .sb-box { background: #ffffff; border: 1px solid #cbd5e1; border-radius: 14px; padding: 12px; display: flex; flex-direction: column; gap: 8px; }
    .sb-label { font-size: 0.72rem; text-transform: uppercase; letter-spacing: 0.05em; color: var(--muted); font-weight: 700; }
    .sb-btn { border: 1px solid #cbd5e1; border-radius: 10px; padding: 9px 12px; min-height: 40px; font-size: 0.82rem; font-weight: 700; cursor: pointer; background: #f1f5f9; color: #0f172a; text-align: center; }
    .sb-btn.ok { background: #16a34a; color: #ffffff; border: 0; }
    .sb-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .toggle-row { display: flex; justify-content: space-between; align-items: center; font-size: 0.84rem; }
    ul { margin: 0; padding: 0; list-style: none; max-height: 170px; overflow-y: auto; font-size: 0.75rem; color: #334155; }
    li { padding: 5px 0; border-bottom: 1px solid #e2e8f0; }
    @media (max-width: 520px) { .monitor-grid { grid-template-columns: 1fr; } .status-box { min-height: 130px; padding: 16px 12px; } }
  </style>
</head>
<body>
  <div id="backdrop" class="backdrop" onclick="closeSidebar()"></div>
  <aside id="sidebar" class="sidebar">
    <div class="sb-hdr"><h2>S.E.N.S.E. MENU</h2><button class="sb-close" onclick="closeSidebar()">Close</button></div>
    <div class="sb-box">
      <div class="sb-label">Alerts &amp; Calibration</div>
      <div class="toggle-row"><span>Sound &amp; Vibration</span><input id="alertToggle" type="checkbox" checked style="width:20px;height:20px;" /></div>
      <button class="sb-btn" onclick="sendDoorAction('reset-peak')">Reset Peak Intensity &amp; Mag</button>
    </div>
    <div class="sb-box">
      <div class="sb-label">Data Logging (Every 10s + Events)</div>
      <div class="sb-grid">
        <button class="sb-btn ok" onclick="exportExcelXlsx()">Excel (.xlsx)</button>
        <button class="sb-btn" onclick="clearServerLogs()">Clear Logs</button>
      </div>
      <div id="logMeta" class="sub-dim" style="color:var(--muted);">Recording periodic snapshots...</div>
      <ul id="logList"></ul>
    </div>
  </aside>

  <main class="app">
    <header class="topbar">
      <div class="top-left">
        <button class="menu-btn" onclick="toggleSidebar()" aria-label="Open menu">=</button>
        <div class="brand-pill">S.E.N.S.E MONITOR</div>
      </div>
      <div id="connPill" class="status-pill err">OFFLINE</div>
    </header>

    <section class="monitor-grid">
      <div class="outer-card">
        <div class="title-pill">Earthquake Intensity &amp; Magnitude</div>
        <div id="quakeBox" class="status-box ok">
          <div id="quakeStatusText" class="big-state">NORMAL</div>
          <div id="quakeMetrics" class="sub-metrics">MMI I · Not Felt<br />Est. M 0.0 · Calm</div>
          <div id="pgaMeta" class="sub-dim">Peak PGA: 0.000 g (0.0 cm/s&sup2;)</div>
        </div>
      </div>

      <div class="outer-card">
        <div class="title-pill">Smoke Detector Status</div>
        <div id="smokeBox" class="status-box ok">
          <div id="smokeVal" class="big-state">CLEAR</div>
          <div id="smokeMeta" class="sub-metrics">No Smoke Detected</div>
          <div id="smokeSignal" class="sub-dim">JGN-301-A: Standby</div>
        </div>
      </div>
    </section>

    <section class="door-card">
      <div class="door-status-pill">
        <span>ELECTRIC DOORKNOB:</span>
        <span id="doorVal" class="door-state-word">UNLOCKED (NORMAL)</span>
      </div>
      <div class="door-actions">
        <button id="btnUnlock" class="pill-btn" onclick="sendDoorAction('unlock')">Emergency Unlock</button>
        <button id="btnLock" class="pill-btn" onclick="sendDoorAction('lock')">Re-Lock Door</button>
      </div>
    </section>
  </main>

  <script>
    const sidebar = document.getElementById('sidebar'), backdrop = document.getElementById('backdrop');
    function openSidebar() { sidebar.classList.add('open'); backdrop.classList.add('open'); }
    function closeSidebar() { sidebar.classList.remove('open'); backdrop.classList.remove('open'); }
    function toggleSidebar() { sidebar.classList.contains('open') ? closeSidebar() : openSidebar(); }
    let dragStartX = null, dragActive = false, dragOpen = false;
    document.addEventListener('touchstart', e => {
      const x = e.touches[0].clientX; dragOpen = sidebar.classList.contains('open');
      if ((!dragOpen && x <= 34) || dragOpen) { dragStartX = x; dragActive = true; }
    }, { passive: true });
    document.addEventListener('touchmove', e => {
      if (!dragActive || dragStartX === null) return;
      const dx = e.touches[0].clientX - dragStartX, w = sidebar.offsetWidth || 300;
      if (!dragOpen && dx > 12) { sidebar.classList.add('dragging'); sidebar.style.transform = `translateX(${Math.min(0, -w + dx)}px)`; }
      else if (dragOpen && dx < -12) { sidebar.classList.add('dragging'); sidebar.style.transform = `translateX(${Math.max(-w, dx)}px)`; }
    }, { passive: true });
    document.addEventListener('touchend', e => {
      if (!dragActive || dragStartX === null) return;
      const dx = e.changedTouches[0].clientX - dragStartX;
      sidebar.classList.remove('dragging'); sidebar.style.transform = '';
      if (!dragOpen && dx > 70) openSidebar(); else if (dragOpen && dx < -70) closeSidebar();
      dragStartX = null; dragActive = false;
    });

    let lastBeepAt = 0;
    function triggerPhoneAlert() {
      if (!document.getElementById('alertToggle').checked) return;
      const now = Date.now(); if (now - lastBeepAt < 3000) return; lastBeepAt = now;
      if (navigator.vibrate) navigator.vibrate([300, 150, 300]);
      try {
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        const osc = ctx.createOscillator(), gain = ctx.createGain();
        osc.type = 'square'; osc.frequency.value = 880; gain.gain.value = 0.08;
        osc.connect(gain); gain.connect(ctx.destination); osc.start();
        setTimeout(() => { osc.stop(); ctx.close(); }, 250);
      } catch (_) {}
    }

    async function sendDoorAction(action) {
      try { await fetch('/api/' + action, { method: 'POST' }); await pollStatus(); } catch (_) {}
    }

    async function pollStatus() {
      const pill = document.getElementById('connPill');
      try {
        const res = await fetch('/api/status', { cache: 'no-store' });
        const d = await res.json();
        pill.textContent = 'LIVE · ' + d.wifiMode; pill.className = 'status-pill live';
        const hazard = d.quakeActive || d.smokeActive;

        const isLightQuake = d.quakeActive && d.estMagnitude < 5.0;
        const qTone = !d.quakeActive ? (d.adxlReady ? 'ok' : 'warn') : (isLightQuake ? 'warn' : 'danger');
        document.getElementById('quakeBox').className = 'status-box ' + qTone;
        document.getElementById('quakeStatusText').textContent = d.quakeActive ? (isLightQuake ? 'LIGHT EARTHQUAKE' : 'EARTHQUAKE!') : (d.adxlReady ? 'NORMAL' : 'NO SENSOR');
        document.getElementById('quakeMetrics').innerHTML = `MMI ${d.intensityMmi} · ${d.intensityLabel}<br />Est. M ${d.estMagnitude.toFixed(1)} · ${d.magnitudeClass}`;
        document.getElementById('pgaMeta').textContent = `Peak PGA: ${d.peakTremorG.toFixed(3)} g (${d.pgaCms2.toFixed(1)} cm/s²)`;

        document.getElementById('smokeBox').className = 'status-box ' + (d.smokeActive ? 'danger' : 'ok');
        document.getElementById('smokeVal').textContent = d.smokeActive ? 'SMOKE DETECTED!' : 'CLEAR';
        document.getElementById('smokeMeta').textContent = d.smokeActive ? 'Evacuate — Smoke Alarm Active' : 'No Smoke Detected';
        document.getElementById('smokeSignal').textContent = 'JGN-301-A: ' + (d.smokeRawSignal ? 'ALARMING' : 'Standby');

        const doorEl = document.getElementById('doorVal');
        doorEl.textContent = hazard ? 'EMERGENCY UNLOCKED' : (d.doorUnlocked ? 'UNLOCKED (NORMAL)' : 'LOCKED');
        doorEl.className = 'door-state-word ' + (hazard ? 'danger' : (d.doorUnlocked ? '' : 'warn'));
        document.getElementById('btnUnlock').disabled = d.doorUnlocked;
        document.getElementById('btnLock').disabled = hazard || !d.doorUnlocked;
        if (hazard) triggerPhoneAlert();
      } catch (_) {
        pill.textContent = 'OFFLINE'; pill.className = 'status-pill err';
      }
    }

    async function fetchServerLogs() {
      try {
        const res = await fetch('/api/logs', { cache: 'no-store' });
        const d = await res.json();
        document.getElementById('logMeta').textContent = `Stored: ${d.totalLogged} entries (latest ${d.logs.length})`;
        const list = document.getElementById('logList'); list.innerHTML = '';
        for (const row of d.logs.slice(0, 35)) {
          const li = document.createElement('li');
          li.textContent = `t+${row.uptimeSec}s [${row.tag}] Q:${row.quakeActive ? 'ALARM' : 'OK'} MMI:${row.mmi} ~M${row.estMag.toFixed(1)} | S:${row.smokeActive ? 'ALARM' : 'OK'} | Door:${row.doorUnlocked ? 'OPEN' : 'LOCK'}`;
          list.appendChild(li);
        }
      } catch (_) {}
    }

    function escXml(s) { return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;').replace(/'/g,'&apos;'); }
    function crc32(bytes) { let c = 0xFFFFFFFF; for (let i = 0; i < bytes.length; i++) { c ^= bytes[i]; for (let k = 0; k < 8; k++) c = (c >>> 1) ^ (c & 1 ? 0xEDB88320 : 0); } return (c ^ 0xFFFFFFFF) >>> 0; }
    function buildStoredZip(files) {
      const enc = new TextEncoder(), parts = [], central = []; let offset = 0;
      for (const [name, text] of files) {
        const nb = enc.encode(name), db = enc.encode(text), crc = crc32(db);
        const lh = new Uint8Array(30 + nb.length), lv = new DataView(lh.buffer);
        lv.setUint32(0,0x04034b50,true); lv.setUint16(4,20,true); lv.setUint32(14,crc,true); lv.setUint32(18,db.length,true); lv.setUint32(22,db.length,true); lv.setUint16(26,nb.length,true); lh.set(nb,30);
        parts.push(lh, db);
        const ch = new Uint8Array(46 + nb.length), cv = new DataView(ch.buffer);
        cv.setUint32(0,0x02014b50,true); cv.setUint16(4,20,true); cv.setUint16(6,20,true); cv.setUint32(16,crc,true); cv.setUint32(20,db.length,true); cv.setUint32(24,db.length,true); cv.setUint16(28,nb.length,true); cv.setUint32(42,offset,true); ch.set(nb,46);
        central.push(ch); offset += lh.length + db.length;
      }
      const cdSize = central.reduce((s, a) => s + a.length, 0), eocd = new Uint8Array(22), ev = new DataView(eocd.buffer);
      ev.setUint32(0,0x06054b50,true); ev.setUint16(8,files.length,true); ev.setUint16(10,files.length,true); ev.setUint32(12,cdSize,true); ev.setUint32(16,offset,true);
      return new Blob([...parts, ...central, eocd], { type: 'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet' });
    }
    async function exportExcelXlsx() {
      let rows = [];
      try {
        const res = await fetch('/api/logs', { cache: 'no-store' });
        const d = await res.json();
        if (d.logs && d.logs.length) {
          rows = d.logs.slice().reverse().map(r => ({ timestamp:'t+'+r.uptimeSec+'s', tag:r.tag||'PERIODIC', quake:r.quakeActive?'ALARM':'OK', tremorG:Number(r.currentG||0), peakG:Number(r.peakG||0), mmi:r.mmi||'I', estMag:Number(r.estMag||0), smoke:r.smokeActive?'ALARM':'OK', door:r.doorUnlocked?'UNLOCKED':'LOCKED' }));
        }
      } catch (_) {}
      if (!rows.length) rows = [{ timestamp:'No Data', tag:'EMPTY', quake:'OK', tremorG:0, peakG:0, mmi:'I', estMag:0, smoke:'OK', door:'UNLOCKED' }];
      const lastRow = rows.length + 1; let qA = 0, sA = 0, okN = 0, maxG = 0, maxM = 0;
      for (const r of rows) { if (r.quake === 'ALARM') qA++; if (r.smoke === 'ALARM') sA++; if (r.quake !== 'ALARM' && r.smoke !== 'ALARM') okN++; if (r.peakG > maxG) maxG = r.peakG; if (r.estMag > maxM) maxM = r.estMag; }
      const sc = (ref, t, s = 3) => `<c r="${ref}" s="${s}" t="inlineStr"><is><t>${escXml(t)}</t></is></c>`;
      const nc = (ref, v, s = 3) => `<c r="${ref}" s="${s}"><v>${Number(v).toFixed(4)}</v></c>`;
      const fc = (ref, f, v, s = 3) => `<c r="${ref}" s="${s}"><f>${f}</f><v>${v}</v></c>`;
      const cols = ['Timestamp','Event','Quake','Tremor (g)','Peak PGA (g)','MMI','Est. Magnitude','Smoke','Door'];
      let sheet2Rows = `<row r="1">` + cols.map((h, i) => sc(String.fromCharCode(65 + i) + '1', h, 1)).join('') + `</row>`;
      rows.forEach((r, i) => { const n = i + 2; sheet2Rows += `<row r="${n}">${sc('A'+n,r.timestamp)}${sc('B'+n,r.tag)}${sc('C'+n,r.quake)}${nc('D'+n,r.tremorG)}${nc('E'+n,r.peakG)}${sc('F'+n,r.mmi)}${nc('G'+n,r.estMag)}${sc('H'+n,r.smoke)}${sc('I'+n,r.door)}</row>`; });
      const sheet1 = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><cols><col min="1" max="1" width="24" customWidth="1"/><col min="2" max="2" width="16" customWidth="1"/></cols><sheetData><row r="1">${sc('A1','S.E.N.S.E. Report & Graphs',2)}</row><row r="3">${sc('A3','Metric',1)}${sc('B3','Count / Peak',1)}</row><row r="4">${sc('A4','Total Samples',4)}${fc('B4',`COUNTA('Data Log'!A2:A${lastRow})`,rows.length)}</row><row r="5">${sc('A5','Earthquake Alarms',4)}${fc('B5',`COUNTIF('Data Log'!C2:C${lastRow},"ALARM")`,qA)}</row><row r="6">${sc('A6','Smoke Alarms',4)}${fc('B6',`COUNTIF('Data Log'!H2:H${lastRow},"ALARM")`,sA)}</row><row r="7">${sc('A7','Normal Logs',4)}${fc('B7',`COUNTIFS('Data Log'!C2:C${lastRow},"OK",'Data Log'!H2:H${lastRow},"OK")`,okN)}</row><row r="8">${sc('A8','Max Peak PGA (g)',4)}${fc('B8',`MAX('Data Log'!E2:E${lastRow})`,maxG.toFixed(4))}</row><row r="9">${sc('A9','Max Est. Magnitude',4)}${fc('B9',`MAX('Data Log'!G2:G${lastRow})`,maxM.toFixed(2))}</row></sheetData><drawing r:id="rId1"/></worksheet>`;
      const sheet2 = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><cols><col min="1" max="1" width="22" customWidth="1"/><col min="2" max="9" width="15" customWidth="1"/></cols><sheetData>${sheet2Rows}</sheetData></worksheet>`;
      const chart1 = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><c:chartSpace xmlns:c="http://schemas.openxmlformats.org/drawingml/2006/chart" xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"><c:date1904 val="0"/><c:roundedCorners val="0"/><c:chart><c:title><c:tx><c:rich><a:bodyPr/><a:lstStyle/><a:p><a:r><a:t>Earthquake Magnitude &amp; Peak PGA Over Time</a:t></a:r></a:p></c:rich></c:tx><c:overlay val="0"/></c:title><c:plotArea><c:layout/><c:lineChart><c:grouping val="standard"/><c:varyColors val="0"/><c:ser><c:idx val="0"/><c:order val="0"/><c:tx><c:strRef><c:f>'Data Log'!$G$1</c:f></c:strRef></c:tx><c:spPr><a:ln w="28575"><a:solidFill><a:srgbClr val="EF4444"/></a:solidFill></a:ln></c:spPr><c:marker><c:symbol val="circle"/><c:size val="6"/><c:spPr><a:solidFill><a:srgbClr val="EF4444"/></a:solidFill><a:ln><a:solidFill><a:srgbClr val="EF4444"/></a:solidFill></a:ln></c:spPr></c:marker><c:cat><c:strRef><c:f>'Data Log'!$A$2:$A$${lastRow}</c:f></c:strRef></c:cat><c:val><c:numRef><c:f>'Data Log'!$G$2:$G$${lastRow}</c:f></c:numRef></c:val></c:ser><c:ser><c:idx val="1"/><c:order val="1"/><c:tx><c:strRef><c:f>'Data Log'!$E$1</c:f></c:strRef></c:tx><c:spPr><a:ln w="28575"><a:solidFill><a:srgbClr val="2563EB"/></a:solidFill></a:ln></c:spPr><c:marker><c:symbol val="diamond"/><c:size val="6"/><c:spPr><a:solidFill><a:srgbClr val="2563EB"/></a:solidFill><a:ln><a:solidFill><a:srgbClr val="2563EB"/></a:solidFill></a:ln></c:spPr></c:marker><c:cat><c:strRef><c:f>'Data Log'!$A$2:$A$${lastRow}</c:f></c:strRef></c:cat><c:val><c:numRef><c:f>'Data Log'!$E$2:$E$${lastRow}</c:f></c:numRef></c:val></c:ser><c:axId val="1001"/><c:axId val="1002"/></c:lineChart><c:catAx><c:axId val="1001"/><c:scaling><c:orientation val="minMax"/></c:scaling><c:delete val="0"/><c:axPos val="b"/><c:crossAx val="1002"/><c:crosses val="autoZero"/></c:catAx><c:valAx><c:axId val="1002"/><c:scaling><c:orientation val="minMax"/></c:scaling><c:delete val="0"/><c:axPos val="l"/><c:majorGridlines/><c:crossAx val="1001"/><c:crosses val="autoZero"/></c:valAx></c:plotArea><c:legend><c:legendPos val="b"/><c:overlay val="0"/></c:legend><c:plotVisOnly val="1"/></c:chart></c:chartSpace>`;
      const chart2 = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><c:chartSpace xmlns:c="http://schemas.openxmlformats.org/drawingml/2006/chart" xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"><c:date1904 val="0"/><c:roundedCorners val="0"/><c:chart><c:title><c:tx><c:rich><a:bodyPr/><a:lstStyle/><a:p><a:r><a:t>Event Summary (Earthquake vs Smoke vs Normal)</a:t></a:r></a:p></c:rich></c:tx><c:overlay val="0"/></c:title><c:plotArea><c:layout/><c:barChart><c:barDir val="col"/><c:grouping val="clustered"/><c:varyColors val="1"/><c:ser><c:idx val="0"/><c:order val="0"/><c:tx><c:strRef><c:f>'Charts &amp; Summary'!$B$3</c:f></c:strRef></c:tx><c:dPt><c:idx val="0"/><c:spPr><a:solidFill><a:srgbClr val="EF4444"/></a:solidFill></c:spPr></c:dPt><c:dPt><c:idx val="1"/><c:spPr><a:solidFill><a:srgbClr val="F59E0B"/></a:solidFill></c:spPr></c:dPt><c:dPt><c:idx val="2"/><c:spPr><a:solidFill><a:srgbClr val="16A34A"/></a:solidFill></c:spPr></c:dPt><c:cat><c:strRef><c:f>'Charts &amp; Summary'!$A$5:$A$7</c:f></c:strRef></c:cat><c:val><c:numRef><c:f>'Charts &amp; Summary'!$B$5:$B$7</c:f></c:numRef></c:val></c:ser><c:axId val="2001"/><c:axId val="2002"/></c:barChart><c:catAx><c:axId val="2001"/><c:scaling><c:orientation val="minMax"/></c:scaling><c:delete val="0"/><c:axPos val="b"/><c:crossAx val="2002"/><c:crosses val="autoZero"/></c:catAx><c:valAx><c:axId val="2002"/><c:scaling><c:orientation val="minMax"/></c:scaling><c:delete val="0"/><c:axPos val="l"/><c:majorGridlines/><c:crossAx val="2001"/><c:crosses val="autoZero"/></c:valAx></c:plotArea><c:legend><c:legendPos val="b"/><c:overlay val="0"/></c:legend><c:plotVisOnly val="1"/></c:chart></c:chartSpace>`;
      const ct = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/><Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/><Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/><Override PartName="/xl/worksheets/sheet2.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/><Override PartName="/xl/drawings/drawing1.xml" ContentType="application/vnd.openxmlformats-officedocument.drawing+xml"/><Override PartName="/xl/charts/chart1.xml" ContentType="application/vnd.openxmlformats-officedocument.drawingml.chart+xml"/><Override PartName="/xl/charts/chart2.xml" ContentType="application/vnd.openxmlformats-officedocument.drawingml.chart+xml"/></Types>`;
      const rr = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/></Relationships>`;
      const wb = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><sheets><sheet name="Charts &amp; Summary" sheetId="1" r:id="rId1"/><sheet name="Data Log" sheetId="2" r:id="rId2"/></sheets><calcPr calcId="191029" fullCalcOnLoad="1"/></workbook>`;
      const wbr = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/><Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet2.xml"/><Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/></Relationships>`;
      const st = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"><fonts count="3"><font><sz val="11"/><name val="Calibri"/></font><font><b/><sz val="11"/><color rgb="FFFFFFFF"/><name val="Calibri"/></font><font><b/><sz val="14"/><color rgb="FF0F172A"/><name val="Calibri"/></font></fonts><fills count="4"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill><fill><patternFill patternType="solid"><fgColor rgb="FF0F172A"/></patternFill></fill><fill><patternFill patternType="solid"><fgColor rgb="FFE2E8F0"/></patternFill></fill></fills><borders count="2"><border><left/><right/><top/><bottom/><diagonal/></border><border><left style="thin"><color rgb="FFCBD5E1"/></left><right style="thin"><color rgb="FFCBD5E1"/></right><top style="thin"><color rgb="FFCBD5E1"/></top><bottom style="thin"><color rgb="FFCBD5E1"/></bottom><diagonal/></border></borders><cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs><cellXfs count="5"><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/><xf numFmtId="0" fontId="1" fillId="2" borderId="1" xfId="0" applyFont="1" applyFill="1" applyBorder="1"/><xf numFmtId="0" fontId="2" fillId="0" borderId="0" xfId="0" applyFont="1"/><xf numFmtId="0" fontId="0" fillId="0" borderId="1" xfId="0" applyBorder="1"/><xf numFmtId="0" fontId="0" fillId="3" borderId="1" xfId="0" applyFill="1" applyBorder="1"/></cellXfs></styleSheet>`;
      const s1r = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/drawing" Target="../drawings/drawing1.xml"/></Relationships>`;
      const drw = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><xdr:wsDr xmlns:xdr="http://schemas.openxmlformats.org/drawingml/2006/spreadsheetDrawing" xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"><xdr:twoCellAnchor><xdr:from><xdr:col>3</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>1</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:from><xdr:to><xdr:col>13</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>17</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:to><xdr:graphicFrame macro=""><xdr:nvGraphicFramePr><xdr:cNvPr id="2" name="SeismicChart"/><xdr:cNvGraphicFramePr/></xdr:nvGraphicFramePr><xdr:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/></xdr:xfrm><xdr:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/chart"><c:chart xmlns:c="http://schemas.openxmlformats.org/drawingml/2006/chart" r:id="rId1"/></a:graphicData></a:graphic></xdr:graphicFrame><xdr:clientData/></xdr:twoCellAnchor><xdr:twoCellAnchor><xdr:from><xdr:col>3</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>18</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:from><xdr:to><xdr:col>13</xdr:col><xdr:colOff>0</xdr:colOff><xdr:row>34</xdr:row><xdr:rowOff>0</xdr:rowOff></xdr:to><xdr:graphicFrame macro=""><xdr:nvGraphicFramePr><xdr:cNvPr id="3" name="HazardSummaryChart"/><xdr:cNvGraphicFramePr/></xdr:nvGraphicFramePr><xdr:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/></xdr:xfrm><xdr:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/chart"><c:chart xmlns:c="http://schemas.openxmlformats.org/drawingml/2006/chart" r:id="rId2"/></a:graphicData></a:graphic></xdr:graphicFrame><xdr:clientData/></xdr:twoCellAnchor></xdr:wsDr>`;
      const drwr = `<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/chart" Target="../charts/chart1.xml"/><Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/chart" Target="../charts/chart2.xml"/></Relationships>`;
      const blob = buildStoredZip([['[Content_Types].xml',ct],['_rels/.rels',rr],['xl/workbook.xml',wb],['xl/_rels/workbook.xml.rels',wbr],['xl/styles.xml',st],['xl/worksheets/sheet1.xml',sheet1],['xl/worksheets/_rels/sheet1.xml.rels',s1r],['xl/worksheets/sheet2.xml',sheet2],['xl/drawings/drawing1.xml',drw],['xl/drawings/_rels/drawing1.xml.rels',drwr],['xl/charts/chart1.xml',chart1],['xl/charts/chart2.xml',chart2]]);
      const url = URL.createObjectURL(blob); const a = document.createElement('a'); a.href = url; a.download = 'sense-data-log.xlsx';
      document.body.appendChild(a); a.click(); document.body.removeChild(a); URL.revokeObjectURL(url);
    }
    async function clearServerLogs() { try { await fetch('/api/logs/clear', { method: 'POST' }); await fetchServerLogs(); } catch (_) {} }

    pollStatus(); fetchServerLogs();
    setInterval(pollStatus, 800); setInterval(fetchServerLogs, 4000);
  </script>
</body>
</html>
)rawliteral";

const char MANIFEST_JSON[] PROGMEM = R"rawliteral({
  "name": "S.E.N.S.E. Monitor",
  "short_name": "SENSE",
  "start_url": "/",
  "display": "standalone",
  "background_color": "#000000",
  "theme_color": "#000000"
})rawliteral";

// -------------------- ADXL345 Helpers --------------------
bool writeAdxlRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readAdxlRegister(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(ADXL345_ADDR, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  val = Wire.read();
  return true;
}

bool readAdxlAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(REG_DATAX0);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(ADXL345_ADDR, static_cast<uint8_t>(6)) != 6) {
    return false;
  }

  uint8_t x0 = Wire.read(), x1 = Wire.read();
  uint8_t y0 = Wire.read(), y1 = Wire.read();
  uint8_t z0 = Wire.read(), z1 = Wire.read();

  int16_t rawX = static_cast<int16_t>((static_cast<uint16_t>(x1) << 8) | x0);
  int16_t rawY = static_cast<int16_t>((static_cast<uint16_t>(y1) << 8) | y0);
  int16_t rawZ = static_cast<int16_t>((static_cast<uint16_t>(z1) << 8) | z0);

  ax = rawX * ADXL_G_PER_LSB;
  ay = rawY * ADXL_G_PER_LSB;
  az = rawZ * ADXL_G_PER_LSB;
  return true;
}

bool initAdxl345() {
  uint8_t devId = 0;
  if (!readAdxlRegister(REG_DEVID, devId) || devId != 0xE5) {
    return false;
  }

  writeAdxlRegister(REG_BW_RATE, 0x0A);     // 100 Hz output data rate
  writeAdxlRegister(REG_DATA_FORMAT, 0x08); // FULL_RES, +/-2g
  writeAdxlRegister(REG_POWER_CTL, 0x08);   // Enable measurement mode
  delay(30);

  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
  int valid = 0;
  for (int i = 0; i < 40; i++) {
    float ax, ay, az;
    if (readAdxlAccel(ax, ay, az)) {
      sumX += ax;
      sumY += ay;
      sumZ += az;
      valid++;
    }
    delay(10);
  }

  if (valid == 0) {
    return false;
  }

  baseAx = sumX / valid;
  baseAy = sumY / valid;
  baseAz = sumZ / valid;
  return true;
}

// -------------------- Door Control --------------------
void recordDataLog(const char *tag);

void setDoorUnlocked(bool unlock) {
  if (doorUnlocked == unlock) {
    return;
  }
  doorUnlocked = unlock;
  digitalWrite(PIN_DOOR_RELAY, unlock ? RELAY_UNLOCK_LEVEL : RELAY_LOCK_LEVEL);
  Serial.println(unlock ? "[ACTION] Electric Doorknob UNLOCKED!"
                        : "[ACTION] Electric Doorknob LOCKED.");
  recordDataLog("DOOR");
}

// -------------------- Web Server Handlers --------------------
void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void handleCorsPreflight() {
  addCorsHeaders();
  server.send(204);
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleManifest() {
  server.send_P(200, "application/manifest+json", MANIFEST_JSON);
}

// -------------------- Seismic Intensity & Magnitude --------------------
// Maps Peak Ground Acceleration (g) to Modified Mercalli Intensity (USGS ShakeMap scale)
// and an estimated near-field local magnitude (~M) for showcase/telemetry.
void classifyEarthquake(float pgaG,
                        const char *&mmiRoman,
                        const char *&mmiLabel,
                        float &estMag,
                        const char *&magClass) {
  if (pgaG < 0.0017f) {
    mmiRoman = "I";   mmiLabel = "Not Felt";
  } else if (pgaG < 0.014f) {
    mmiRoman = "III"; mmiLabel = "Weak";
  } else if (pgaG < 0.039f) {
    mmiRoman = "IV";  mmiLabel = "Light";
  } else if (pgaG < 0.092f) {
    mmiRoman = "V";   mmiLabel = "Moderate";
  } else if (pgaG < 0.18f) {
    mmiRoman = "VI";  mmiLabel = "Strong";
  } else if (pgaG < 0.34f) {
    mmiRoman = "VII"; mmiLabel = "Very Strong";
  } else if (pgaG < 0.65f) {
    mmiRoman = "VIII"; mmiLabel = "Severe";
  } else if (pgaG < 1.24f) {
    mmiRoman = "IX";  mmiLabel = "Violent";
  } else {
    mmiRoman = "X+";  mmiLabel = "Extreme";
  }

  if (pgaG < 0.005f) {
    estMag = 0.0f;
    magClass = "Calm";
    return;
  }

  float pgaCms2 = pgaG * 980.665f;
  estMag = 1.65f * log10f(pgaCms2) + 1.8f;
  if (estMag < 1.0f) estMag = 1.0f;
  if (estMag > 9.0f) estMag = 9.0f;

  if (estMag < 2.0f) {
    magClass = "Micro";
  } else if (estMag < 4.0f) {
    magClass = "Minor";
  } else if (estMag < 5.0f) {
    magClass = "Light";
  } else if (estMag < 6.0f) {
    magClass = "Moderate";
  } else if (estMag < 7.0f) {
    magClass = "Strong";
  } else {
    magClass = "Major";
  }
}

void handleStatusApi() {
  addCorsHeaders();
  bool smokeRaw = (digitalRead(PIN_SMOKE_IN) == SMOKE_ACTIVE_LEVEL);
  String ipStr = usingApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();

  float evalG = (peakTremorG > currentTremorG) ? peakTremorG : currentTremorG;
  const char *mmiRoman = "I";
  const char *mmiLabel = "Not Felt";
  const char *magClass = "Calm";
  float estMag = 0.0f;
  classifyEarthquake(evalG, mmiRoman, mmiLabel, estMag, magClass);
  float pgaCms2 = evalG * 980.665f;

  char json[512];
  snprintf(json, sizeof(json),
           "{\"quakeActive\":%s,\"adxlReady\":%s,\"currentTremorG\":%.4f,\"peakTremorG\":%.4f,"
           "\"pgaCms2\":%.2f,\"intensityMmi\":\"%s\",\"intensityLabel\":\"%s\","
           "\"estMagnitude\":%.2f,\"magnitudeClass\":\"%s\","
           "\"smokeActive\":%s,\"smokeRawSignal\":%s,\"doorUnlocked\":%s,"
           "\"uptimeSec\":%lu,\"wifiMode\":\"%s\",\"ip\":\"%s\"}",
           quakeActive ? "true" : "false",
           adxlReady ? "true" : "false",
           currentTremorG,
           peakTremorG,
           pgaCms2,
           mmiRoman,
           mmiLabel,
           estMag,
           magClass,
           smokeActive ? "true" : "false",
           smokeRaw ? "true" : "false",
           doorUnlocked ? "true" : "false",
           static_cast<unsigned long>(millis() / 1000UL),
           usingApMode ? "AP" : "WiFi",
           ipStr.c_str());

  server.send(200, "application/json", json);
}

void handleDoorUnlockApi() {
  addCorsHeaders();
  setDoorUnlocked(true);
  lastHazardMs = millis();
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"Door unlocked from phone.\"}");
}

void handleResetPeakApi() {
  addCorsHeaders();
  if (!quakeActive) {
    peakTremorG = 0.0f;
  }
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"Peak intensity/magnitude reset.\"}");
}

void recordDataLog(const char *tag) {
  float evalG = (peakTremorG > currentTremorG) ? peakTremorG : currentTremorG;
  const char *mmiRoman = "I";
  const char *mmiLabel = "Not Felt";
  const char *magClass = "Calm";
  float estMag = 0.0f;
  classifyEarthquake(evalG, mmiRoman, mmiLabel, estMag, magClass);

  DataLogEntry &e = dataLogs[dataLogHead];
  e.uptimeSec    = millis() / 1000UL;
  e.currentG     = currentTremorG;
  e.peakG        = peakTremorG;
  e.estMag       = estMag;
  e.quakeActive  = quakeActive;
  e.smokeActive  = smokeActive;
  e.doorUnlocked = doorUnlocked;
  snprintf(e.mmi, sizeof(e.mmi), "%s", mmiRoman);
  snprintf(e.tag, sizeof(e.tag), "%s", tag);

  dataLogHead = (dataLogHead + 1) % DATA_LOG_CAPACITY;
  if (dataLogCount < DATA_LOG_CAPACITY) {
    dataLogCount++;
  }
  totalLogged++;
}

void handleLogsApi() {
  addCorsHeaders();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  char hdr[64];
  snprintf(hdr, sizeof(hdr), "{\"totalLogged\":%lu,\"logs\":[",
           static_cast<unsigned long>(totalLogged));
  server.sendContent(hdr);

  uint16_t maxSend = dataLogCount;
  for (uint16_t i = 0; i < maxSend; i++) {
    int32_t idx = static_cast<int32_t>(dataLogHead) - 1 - static_cast<int32_t>(i);
    if (idx < 0) idx += DATA_LOG_CAPACITY;
    const DataLogEntry &e = dataLogs[idx];

    char item[192];
    snprintf(item, sizeof(item),
             "%s{\"uptimeSec\":%lu,\"tag\":\"%s\",\"quakeActive\":%s,\"currentG\":%.3f,"
             "\"peakG\":%.3f,\"mmi\":\"%s\",\"estMag\":%.2f,\"smokeActive\":%s,\"doorUnlocked\":%s}",
             (i > 0) ? "," : "",
             static_cast<unsigned long>(e.uptimeSec),
             e.tag,
             e.quakeActive ? "true" : "false",
             e.currentG,
             e.peakG,
             e.mmi,
             e.estMag,
             e.smokeActive ? "true" : "false",
             e.doorUnlocked ? "true" : "false");
    server.sendContent(item);
  }

  server.sendContent("]}");
  server.sendContent("");
}

void handleLogsCsvApi() {
  addCorsHeaders();
  server.sendHeader("Content-Disposition", "attachment; filename=\"sense-data-log.csv\"");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("Uptime(s),Event,Quake,Tremor(g),PeakPGA(g),MMI,EstMagnitude,Smoke,Door\n");

  for (uint16_t i = 0; i < dataLogCount; i++) {
    int32_t idx = static_cast<int32_t>(dataLogHead) - static_cast<int32_t>(dataLogCount) + static_cast<int32_t>(i);
    if (idx < 0) idx += DATA_LOG_CAPACITY;
    const DataLogEntry &e = dataLogs[idx];

    char row[128];
    snprintf(row, sizeof(row),
             "%lu,%s,%s,%.3f,%.3f,%s,%.2f,%s,%s\n",
             static_cast<unsigned long>(e.uptimeSec),
             e.tag,
             e.quakeActive ? "ALARM" : "OK",
             e.currentG,
             e.peakG,
             e.mmi,
             e.estMag,
             e.smokeActive ? "ALARM" : "OK",
             e.doorUnlocked ? "UNLOCKED" : "LOCKED");
    server.sendContent(row);
  }
  server.sendContent("");
}

void handleLogsClearApi() {
  addCorsHeaders();
  dataLogHead = 0;
  dataLogCount = 0;
  totalLogged = 0;
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"ESP32 data log cleared.\"}");
}

void handleDoorLockApi() {
  addCorsHeaders();
  if (quakeActive || smokeActive) {
    server.send(409, "application/json",
                "{\"ok\":false,\"message\":\"Blocked: hazard is still active.\"}");
    return;
  }
  quakeHits = 0;
  peakTremorG = 0.0f;
  setDoorUnlocked(false);
  server.send(200, "application/json", "{\"ok\":true,\"message\":\"Door re-locked from phone.\"}");
}

void setupWiFiAndServer() {
  bool trySta = (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0) && (strlen(WIFI_SSID) > 0);

  if (trySta) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start < 8000)) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      usingApMode = false;
      Serial.printf("[WIFI] Connected! Open on phone: http://%s\n",
                    WiFi.localIP().toString().c_str());
    } else {
      Serial.println("[WIFI] Router connection failed. Falling back to Hotspot mode.");
      trySta = false;
    }
  }

  if (!trySta) {
    usingApMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("[WIFI] Hotspot active! SSID: %s | Pass: %s | Open: http://%s\n",
                  AP_SSID, AP_PASSWORD, WiFi.softAPIP().toString().c_str());
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/manifest.webmanifest", HTTP_GET, handleManifest);
  server.on("/api/status", HTTP_GET, handleStatusApi);
  server.on("/api/status", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/api/unlock", HTTP_POST, handleDoorUnlockApi);
  server.on("/api/unlock", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/api/lock", HTTP_POST, handleDoorLockApi);
  server.on("/api/lock", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/api/reset-peak", HTTP_POST, handleResetPeakApi);
  server.on("/api/reset-peak", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/api/logs", HTTP_GET, handleLogsApi);
  server.on("/api/logs", HTTP_OPTIONS, handleCorsPreflight);
  server.on("/api/logs.csv", HTTP_GET, handleLogsCsvApi);
  server.on("/api/logs/clear", HTTP_POST, handleLogsClearApi);
  server.on("/api/logs/clear", HTTP_OPTIONS, handleCorsPreflight);
  server.begin();
  Serial.println("[HTTP] Phone monitor web server started.");
}

// -------------------- Sensor Updates --------------------
void updateEarthquakeSensor(uint32_t now) {
  if (!adxlReady || (now - lastQuakeSampleMs < QUAKE_SAMPLE_MS)) {
    return;
  }
  lastQuakeSampleMs = now;

  float ax, ay, az;
  if (!readAdxlAccel(ax, ay, az)) {
    return;
  }

  constexpr float alpha = 0.05f;
  baseAx = alpha * ax + (1.0f - alpha) * baseAx;
  baseAy = alpha * ay + (1.0f - alpha) * baseAy;
  baseAz = alpha * az + (1.0f - alpha) * baseAz;

  float dx = ax - baseAx;
  float dy = ay - baseAy;
  float dz = az - baseAz;
  currentTremorG = sqrtf(dx * dx + dy * dy + dz * dz);
  if (currentTremorG > peakTremorG) {
    peakTremorG = currentTremorG;
  }

  if (quakeHits > 0 && (now - quakeWindowStart > QUAKE_WINDOW_MS)) {
    quakeHits = 0;
  }

  if (currentTremorG >= QUAKE_THRESHOLD_G) {
    if (quakeHits == 0) {
      quakeWindowStart = now;
    }
    quakeHits++;

    if (quakeHits >= QUAKE_HITS_REQUIRED) {
      bool wasActive = quakeActive;
      quakeActive = true;
      lastQuakeMs = now;
      lastHazardMs = now;
      if (!wasActive) {
        Serial.printf("[ALERT] EARTHQUAKE DETECTED! Tremor: %.3f g\n", currentTremorG);
        recordDataLog("QUAKE");
      }
    }
  }

  if (quakeActive && (now - lastQuakeMs > 5000)) {
    quakeActive = false;
    quakeHits = 0;
    Serial.println("[INFO] Seismic tremor stopped.");
    recordDataLog("Q_CLEAR");
  }
}

void updateSmokeSensor(uint32_t now) {
  bool rawSignal = (digitalRead(PIN_SMOKE_IN) == SMOKE_ACTIVE_LEVEL);

  if (rawSignal) {
    if (smokeActiveStart == 0) {
      smokeActiveStart = now;
    }
    if (now - smokeActiveStart >= SMOKE_DEBOUNCE_MS) {
      bool wasActive = smokeActive;
      smokeActive = true;
      lastSmokeSeenMs = now;
      lastHazardMs = now;
      if (!wasActive) {
        Serial.println("[ALERT] SMOKE DETECTED from JGN-301-A!");
        recordDataLog("SMOKE");
      }
    }
  } else {
    smokeActiveStart = 0;
    if (smokeActive && (now - lastSmokeSeenMs > SMOKE_HOLD_MS)) {
      smokeActive = false;
      Serial.println("[INFO] JGN-301-A smoke signal cleared.");
      recordDataLog("S_CLEAR");
    }
  }
}

void updateDoorAndStatus(uint32_t now) {
  // Always force door unlocked during earthquake or smoke;
  // otherwise keep the current normal (unlocked) or manually-locked state.
  if (quakeActive || smokeActive) {
    setDoorUnlocked(true);
  }

  if (quakeActive || smokeActive) {
    digitalWrite(PIN_LED_STATUS, (now / 150) % 2);  // Fast blink on emergency
  } else if (!doorUnlocked) {
    digitalWrite(PIN_LED_STATUS, HIGH);             // Solid ON when manually locked
  } else {
    digitalWrite(PIN_LED_STATUS, (now % 2000) < 150 ? HIGH : LOW);  // Heartbeat in normal use
  }
}

void handleResetButton(uint32_t now) {
  static bool prevPressed = false;
  static uint32_t lastDebounceMs = 0;

  bool pressed = (digitalRead(PIN_RESET_BTN) == LOW);
  if (pressed && !prevPressed && (now - lastDebounceMs > 250)) {
    lastDebounceMs = now;
    if (quakeActive || smokeActive) {
      Serial.println("[WARN] Cannot lock while earthquake or smoke hazard is active.");
      prevPressed = pressed;
      return;
    }
    quakeHits = 0;
    peakTremorG = 0.0f;
    setDoorUnlocked(!doorUnlocked);
    Serial.println(doorUnlocked ? "[INPUT] Button: Normal unlocked mode."
                                : "[INPUT] Button: Door manually locked.");
  }
  prevPressed = pressed;
}

void updatePeriodicDataLog(uint32_t now) {
  if (now - lastDataLogMs < DATA_LOG_INTERVAL_MS) {
    return;
  }
  lastDataLogMs = now;
  recordDataLog("PERIODIC");
  Serial.printf("[LOG] #%lu recorded (stored: %u/%u)\n",
                static_cast<unsigned long>(totalLogged),
                dataLogCount,
                DATA_LOG_CAPACITY);
}

void printTelemetry(uint32_t now) {
  if (now - lastTelemetryMs < TELEMETRY_MS) {
    return;
  }
  lastTelemetryMs = now;

  float evalG = (peakTremorG > currentTremorG) ? peakTremorG : currentTremorG;
  const char *mmiRoman = "I";
  const char *mmiLabel = "Not Felt";
  const char *magClass = "Calm";
  float estMag = 0.0f;
  classifyEarthquake(evalG, mmiRoman, mmiLabel, estMag, magClass);

  Serial.printf("Status | Quake: %s (%.3fg, MMI %s %s, ~M%.1f %s) | Smoke: %s | Door: %s\n",
                quakeActive ? "ALARM" : "OK",
                currentTremorG,
                mmiRoman,
                mmiLabel,
                estMag,
                magClass,
                smokeActive ? "ALARM" : "OK",
                doorUnlocked ? "UNLOCKED" : "LOCKED");
}

// -------------------- Arduino Entry Points --------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== S.E.N.S.E. ADXL345 + JGN-301-A + Electric Doorknob ===");

  pinMode(PIN_SMOKE_IN, (SMOKE_ACTIVE_LEVEL == LOW) ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_DOOR_RELAY, RELAY_UNLOCK_LEVEL);  // Default: normal open/close use
  pinMode(PIN_DOOR_RELAY, OUTPUT);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  adxlReady = initAdxl345();
  if (adxlReady) {
    Serial.println("[READY] ADXL345 initialized (I2C 0x53).");
  } else {
    Serial.println("[WARN] ADXL345 not detected! Check SDA(21), SCL(22), and CS->3.3V.");
  }

  setupWiFiAndServer();
  recordDataLog("BOOT");
}

void loop() {
  uint32_t now = millis();

  server.handleClient();
  handleResetButton(now);
  updateEarthquakeSensor(now);
  updateSmokeSensor(now);
  updateDoorAndStatus(now);
  updatePeriodicDataLog(now);
  printTelemetry(now);
}
