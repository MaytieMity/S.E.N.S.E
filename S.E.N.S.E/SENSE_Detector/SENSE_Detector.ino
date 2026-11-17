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
  <link rel="icon" type="image/png" href="data:image/png;base64,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" />
  <link rel="apple-touch-icon" href="data:image/png;base64,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" />
  <title>S.E.N.S.E. Monitor</title>
  <style>
    :root {
      --bg: #000000;
      --panel-top: #e8edf4;
      --panel-bot: #cbd5e1;
      --text: #f8fafc;
      --dark-text: #090d16;
      --muted: #94a3b8;
      --ok: #16a34a;
      --warn: #facc15;
      --danger: #dc2626;
      --accent: #38bdf8;
    }
    * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    html, body { margin: 0; min-height: 100dvh; background: var(--bg); color: var(--text); overflow-x: hidden; }
    body { padding: calc(14px + env(safe-area-inset-top)) 16px calc(18px + env(safe-area-inset-bottom)); display: flex; justify-content: center; }
    .app { width: 100%; max-width: 940px; display: flex; flex-direction: column; gap: 14px; }

    .topbar { display: flex; align-items: center; justify-content: space-between; gap: 10px; }
    .top-left { display: flex; align-items: center; gap: 8px; }
    .menu-btn { width: 48px; height: 44px; border: 1px solid rgba(255,255,255,0.45); border-radius: 999px; background: linear-gradient(180deg, var(--panel-top), var(--panel-bot)); color: var(--dark-text); display: inline-flex; align-items: center; justify-content: center; cursor: pointer; box-shadow: inset 0 1px 0 rgba(255,255,255,0.85), 0 4px 12px rgba(0,0,0,0.45); transition: transform 0.14s ease, filter 0.14s ease; }
    .menu-btn:active { transform: scale(0.93); filter: brightness(0.95); }
    .brand-pill { height: 44px; padding: 0 18px; border: 1px solid rgba(255,255,255,0.45); border-radius: 999px; background: linear-gradient(180deg, var(--panel-top), var(--panel-bot)); color: var(--dark-text); font-size: 0.9rem; font-weight: 800; letter-spacing: 0.05em; display: inline-flex; align-items: center; gap: 8px; box-shadow: inset 0 1px 0 rgba(255,255,255,0.85), 0 4px 12px rgba(0,0,0,0.45); }
    .status-pill { height: 44px; padding: 0 18px; border-radius: 999px; font-size: 0.85rem; font-weight: 800; letter-spacing: 0.05em; display: inline-flex; align-items: center; justify-content: center; gap: 8px; color: #ffffff; border: 1px solid rgba(255,255,255,0.22); transition: background 0.25s, box-shadow 0.25s; }
    .status-pill .dot { width: 9px; height: 9px; border-radius: 50%; background: currentColor; box-shadow: 0 0 8px currentColor; }
    .status-pill.live { background: linear-gradient(135deg, #22c55e, #15803d); box-shadow: 0 4px 16px rgba(34,197,94,0.38); }
    .status-pill.live .dot { animation: dotBlink 1.5s infinite; }
    .status-pill.err { background: linear-gradient(135deg, #ef4444, #b91c1c); box-shadow: 0 4px 16px rgba(239,68,68,0.38); }
    @keyframes dotBlink { 0%,100% { opacity: 1; transform: scale(1); } 50% { opacity: 0.45; transform: scale(0.8); } }

    .monitor-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; flex: 1; }
    .outer-card { background: linear-gradient(180deg, var(--panel-top), var(--panel-bot)); border: 1px solid rgba(255,255,255,0.5); border-radius: 26px; padding: 12px; display: flex; flex-direction: column; gap: 10px; box-shadow: inset 0 1px 0 rgba(255,255,255,0.85), 0 12px 28px rgba(0,0,0,0.6); }
    .title-pill { background: #070b12; color: #f8fafc; border: 1px solid rgba(255,255,255,0.08); border-radius: 999px; padding: 10px 14px; font-size: 0.86rem; font-weight: 700; letter-spacing: 0.02em; display: flex; align-items: center; justify-content: center; gap: 8px; box-shadow: 0 3px 10px rgba(0,0,0,0.28); }
    .title-pill span { white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
    .status-box { position: relative; overflow: hidden; background: linear-gradient(155deg, #22c55e 0%, #15803d 100%); color: #ffffff; border-radius: 20px; padding: 20px 14px; min-height: 205px; flex: 1; display: flex; flex-direction: column; align-items: center; justify-content: center; text-align: center; gap: 10px; border: 1px solid rgba(255,255,255,0.25); box-shadow: inset 0 1px 0 rgba(255,255,255,0.32), 0 8px 22px rgba(22,163,74,0.35); transition: background 0.3s ease, color 0.3s ease, box-shadow 0.3s ease; }
    .status-box::after { content: ""; position: absolute; inset: 0; background: radial-gradient(circle at 50% 0%, rgba(255,255,255,0.22), transparent 65%); pointer-events: none; }
    .status-box.ok { background: linear-gradient(155deg, #22c55e 0%, #15803d 100%); color: #ffffff; box-shadow: inset 0 1px 0 rgba(255,255,255,0.32), 0 8px 22px rgba(22,163,74,0.35); }
    .status-box.warn { background: linear-gradient(155deg, #fef08a 0%, #eab308 100%); color: #111827; box-shadow: inset 0 1px 0 rgba(255,255,255,0.55), 0 8px 22px rgba(234,179,8,0.38); }
    .status-box.danger { background: linear-gradient(155deg, #ef4444 0%, #b91c1c 100%); color: #ffffff; animation: boxPulse 1.15s infinite; }
    @keyframes boxPulse { 0%,100% { box-shadow: inset 0 1px 0 rgba(255,255,255,0.28), 0 0 0 0 rgba(239,68,68,0.65); } 50% { box-shadow: inset 0 1px 0 rgba(255,255,255,0.28), 0 0 0 12px rgba(239,68,68,0); } }
    .icon-ring { width: 52px; height: 52px; border-radius: 50%; background: rgba(255,255,255,0.18); border: 1px solid rgba(255,255,255,0.32); display: flex; align-items: center; justify-content: center; backdrop-filter: blur(4px); }
    .status-box.warn .icon-ring { background: rgba(17,24,39,0.10); border-color: rgba(17,24,39,0.22); }
    .big-state { font-size: clamp(1.2rem, 3.6vw, 2rem); font-weight: 900; letter-spacing: 0.04em; line-height: 1.1; text-shadow: 0 2px 6px rgba(0,0,0,0.18); }
    .status-box.warn .big-state { text-shadow: none; }
    .chip-row { display: flex; flex-wrap: wrap; justify-content: center; gap: 6px; width: 100%; }
    .metric-chip { background: rgba(0,0,0,0.22); border: 1px solid rgba(255,255,255,0.22); border-radius: 12px; padding: 6px 10px; font-size: 0.78rem; font-weight: 700; line-height: 1.25; }
    .status-box.warn .metric-chip { background: rgba(17,24,39,0.10); border-color: rgba(17,24,39,0.22); }
    .seg-bar { display: grid; grid-template-columns: repeat(6, 1fr); gap: 4px; width: min(210px, 90%); margin-top: 2px; }
    .seg { height: 6px; border-radius: 999px; background: rgba(255,255,255,0.24); transition: background 0.25s, transform 0.25s; }
    .status-box.warn .seg { background: rgba(17,24,39,0.18); }
    .seg.on { background: #ffffff; transform: scaleY(1.15); }
    .status-box.warn .seg.on { background: #111827; }
    .sub-dim { font-size: 0.74rem; font-weight: 600; opacity: 0.9; }

    .door-card { background: linear-gradient(180deg, var(--panel-top), var(--panel-bot)); border: 1px solid rgba(255,255,255,0.5); border-radius: 26px; padding: 12px; display: flex; flex-direction: column; gap: 10px; box-shadow: inset 0 1px 0 rgba(255,255,255,0.85), 0 12px 28px rgba(0,0,0,0.6); }
    .door-status-pill { background: #070b12; color: #ffffff; border-radius: 999px; padding: 10px 18px; font-size: 0.88rem; font-weight: 700; display: inline-flex; align-items: center; gap: 10px; width: fit-content; max-width: 100%; box-shadow: 0 3px 10px rgba(0,0,0,0.28); }
    .door-state-word { font-weight: 800; color: #4ade80; }
    .door-state-word.warn { color: #facc15; }
    .door-state-word.danger { color: #f87171; }
    .door-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .pill-btn { background: linear-gradient(180deg, #111827, #030712); color: #ffffff; border: 1px solid rgba(255,255,255,0.14); border-radius: 999px; padding: 12px 14px; min-height: 48px; font-size: 0.88rem; font-weight: 700; cursor: pointer; display: inline-flex; align-items: center; justify-content: center; gap: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.25); transition: transform 0.12s ease, opacity 0.2s ease; }
    .pill-btn.unlock-accent { border-color: rgba(250,204,21,0.55); }
    .pill-btn:disabled { opacity: 0.42; cursor: not-allowed; }
    .pill-btn:active:not(:disabled) { transform: scale(0.97); }

    .backdrop { position: fixed; inset: 0; background: rgba(0,0,0,0.72); backdrop-filter: blur(3px); opacity: 0; pointer-events: none; transition: opacity 0.25s ease; z-index: 40; }
    .backdrop.open { opacity: 1; pointer-events: auto; }
    .sidebar { position: fixed; top: 0; left: 0; bottom: 0; width: min(335px, 86vw); background: linear-gradient(180deg, #0f172a, #090e1a); color: #f8fafc; border-right: 1px solid #233252; padding: calc(16px + env(safe-area-inset-top)) 14px calc(18px + env(safe-area-inset-bottom)); transform: translateX(-100%); transition: transform 0.25s cubic-bezier(0.22, 1, 0.36, 1); z-index: 50; display: flex; flex-direction: column; gap: 12px; overflow-y: auto; touch-action: pan-y; box-shadow: 12px 0 36px rgba(0,0,0,0.75); }
    .sidebar.dragging { transition: none; }
    .sidebar.open { transform: translateX(0); }
    .sb-hdr { display: flex; justify-content: space-between; align-items: center; padding-bottom: 10px; border-bottom: 1px solid #1e293b; }
    .sb-hdr h2 { margin: 0; font-size: 1rem; letter-spacing: 0.06em; color: #f8fafc; }
    .sb-close { background: #1e293b; color: #f8fafc; border: 1px solid #334155; border-radius: 10px; padding: 6px 12px; font-weight: 700; cursor: pointer; }
    .sb-box { background: #131d33; border: 1px solid #233252; border-radius: 16px; padding: 12px; display: flex; flex-direction: column; gap: 9px; }
    .sb-label { font-size: 0.72rem; text-transform: uppercase; letter-spacing: 0.06em; color: var(--muted); font-weight: 700; }
    .sb-btn { border: 1px solid #334155; border-radius: 10px; padding: 9px 12px; min-height: 40px; font-size: 0.82rem; font-weight: 700; cursor: pointer; background: #1e293b; color: #f8fafc; text-align: center; transition: transform 0.1s ease; }
    .sb-btn:active { transform: scale(0.97); }
    .sb-btn.ok { background: linear-gradient(135deg, #22c55e, #15803d); color: #ffffff; border: 0; }
    .sb-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .toggle-row { display: flex; justify-content: space-between; align-items: center; font-size: 0.85rem; }
    .switch { position: relative; display: inline-block; width: 46px; height: 26px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; inset: 0; background: #334155; border-radius: 999px; transition: 0.22s; }
    .slider::before { content: ""; position: absolute; height: 20px; width: 20px; left: 3px; bottom: 3px; background: #ffffff; border-radius: 50%; transition: 0.22s; }
    .switch input:checked + .slider { background: #22c55e; }
    .switch input:checked + .slider::before { transform: translateX(20px); }
    ul { margin: 0; padding: 0; list-style: none; max-height: 170px; overflow-y: auto; font-size: 0.74rem; color: #cbd5e1; }
    li { padding: 6px 0; border-bottom: 1px solid rgba(148,163,184,0.14); line-height: 1.35; }
    @media (max-width: 520px) { .monitor-grid { grid-template-columns: 1fr; } .status-box { min-height: 155px; padding: 16px 12px; } }
  </style>
</head>
<body>
  <div id="backdrop" class="backdrop" onclick="closeSidebar()"></div>
  <aside id="sidebar" class="sidebar" aria-label="Control Menu">
    <div class="sb-hdr"><h2>S.E.N.S.E. MENU</h2><button class="sb-close" onclick="closeSidebar()">Close</button></div>
    <div class="sb-box">
      <div class="sb-label">Alerts &amp; Calibration</div>
      <div class="toggle-row">
        <span>Sound &amp; Vibration</span>
        <label class="switch"><input id="alertToggle" type="checkbox" checked /><span class="slider"></span></label>
      </div>
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
        <button class="menu-btn" onclick="toggleSidebar()" aria-label="Open menu">
          <svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.6" stroke-linecap="round">
            <line x1="4" y1="7" x2="20" y2="7"/><line x1="4" y1="12" x2="20" y2="12"/><line x1="4" y1="17" x2="20" y2="17"/>
          </svg>
        </button>
        <div class="brand-pill" style="padding:0 16px 0 10px;">
          <img src="data:image/png;base64,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" alt="S.E.N.S.E. Logo" width="28" height="28" style="border-radius:7px;display:block;" />
          <span>S.E.N.S.E MONITOR</span>
        </div>
      </div>
      <div id="connPill" class="status-pill err"><span class="dot"></span><span id="connText">OFFLINE</span></div>
    </header>

    <section class="monitor-grid">
      <div class="outer-card">
        <div class="title-pill">
          <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="#38bdf8" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round">
            <polyline points="2 12 6 12 9 4 13 20 17 9 19 12 22 12"/>
          </svg>
          <span>Earthquake Intensity &amp; Magnitude</span>
        </div>
        <div id="quakeBox" class="status-box ok">
          <div class="icon-ring">
            <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.3" stroke-linecap="round" stroke-linejoin="round">
              <polyline points="2 12 6 12 9 4 13 20 17 9 19 12 22 12"/>
            </svg>
          </div>
          <div id="quakeStatusText" class="big-state">NORMAL</div>
          <div id="quakeMetrics" class="chip-row">
            <span class="metric-chip">MMI I · Not Felt</span>
            <span class="metric-chip">Est. M 0.0 · Calm</span>
          </div>
          <div id="segBar" class="seg-bar">
            <div class="seg"></div><div class="seg"></div><div class="seg"></div>
            <div class="seg"></div><div class="seg"></div><div class="seg"></div>
          </div>
          <div id="pgaMeta" class="sub-dim">Peak PGA: 0.000 g (0.0 cm/s&sup2;)</div>
        </div>
      </div>

      <div class="outer-card">
        <div class="title-pill">
          <svg width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="#f97316" stroke-width="2.4" stroke-linecap="round" stroke-linejoin="round">
            <path d="M12 2c.5 4.5 4 6.5 4 11a4 4 0 1 1-8 0c0-2.5 1.5-4.5 2.5-6 0 2 1.5 3 1.5 3s-.5-3.5 0-8z"/>
          </svg>
          <span>Smoke Detector Status</span>
        </div>
        <div id="smokeBox" class="status-box ok">
          <div class="icon-ring">
            <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
              <path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/>
              <path d="M9 12l2 2 4-4"/>
            </svg>
          </div>
          <div id="smokeVal" class="big-state">CLEAR</div>
          <div class="chip-row">
            <span id="smokeMeta" class="metric-chip">No Smoke Detected</span>
          </div>
          <div id="smokeSignal" class="sub-dim">JGN-301-A · Standby</div>
        </div>
      </div>
    </section>

    <section class="door-card">
      <div class="door-status-pill">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="#94a3b8" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round">
          <rect x="3" y="11" width="18" height="11" rx="2"/><path d="M7 11V7a5 5 0 0 1 9.9-1"/>
        </svg>
        <span>ELECTRIC DOORKNOB:</span>
        <span id="doorVal" class="door-state-word">UNLOCKED (NORMAL)</span>
      </div>
      <div class="door-actions">
        <button id="btnUnlock" class="pill-btn unlock-accent" onclick="sendDoorAction('unlock')">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="#facc15" stroke-width="2.3" stroke-linecap="round" stroke-linejoin="round">
            <rect x="3" y="11" width="18" height="11" rx="2"/><path d="M7 11V7a5 5 0 0 1 9.9-1"/>
          </svg>
          <span>Emergency Unlock</span>
        </button>
        <button id="btnLock" class="pill-btn" onclick="sendDoorAction('lock')">
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="#cbd5e1" stroke-width="2.3" stroke-linecap="round" stroke-linejoin="round">
            <rect x="3" y="11" width="18" height="11" rx="2"/><path d="M7 11V7a5 5 0 0 1 10 0v4"/>
          </svg>
          <span>Re-Lock Door</span>
        </button>
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

    function updateSegBar(mag) {
      const lit = mag <= 0 ? 0 : Math.min(6, Math.max(1, Math.round((mag / 6.8) * 6)));
      document.querySelectorAll('#segBar .seg').forEach((el, idx) => el.classList.toggle('on', idx < lit));
    }

    function resetUiToStandby() {
      document.getElementById('quakeBox').className = 'status-box ok';
      document.getElementById('quakeStatusText').textContent = 'NORMAL';
      document.getElementById('quakeMetrics').innerHTML =
        '<span class="metric-chip">MMI I · Not Felt</span><span class="metric-chip">Est. M 0.0 · Calm</span>';
      updateSegBar(0);
      document.getElementById('pgaMeta').textContent = 'Peak PGA: 0.000 g (0.0 cm/s²)';
      document.getElementById('smokeBox').className = 'status-box ok';
      document.getElementById('smokeVal').textContent = 'CLEAR';
      document.getElementById('smokeMeta').textContent = 'No Smoke Detected';
      document.getElementById('smokeSignal').textContent = 'JGN-301-A · Standby';
      const doorEl = document.getElementById('doorVal');
      doorEl.textContent = 'UNLOCKED (NORMAL)';
      doorEl.className = 'door-state-word';
      document.getElementById('btnUnlock').disabled = true;
      document.getElementById('btnLock').disabled = false;
    }

    async function fetchWithTimeout(url, options = {}, timeoutMs = 1500) {
      const ctrl = new AbortController();
      const timer = setTimeout(() => ctrl.abort(), timeoutMs);
      try { return await fetch(url, { ...options, signal: ctrl.signal }); }
      finally { clearTimeout(timer); }
    }

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
      try { await fetchWithTimeout('/api/' + action, { method: 'POST' }, 2000); await pollStatus(); } catch (_) {}
    }

    async function pollStatus() {
      const pill = document.getElementById('connPill');
      const connText = document.getElementById('connText');
      try {
        const res = await fetchWithTimeout('/api/status', { cache: 'no-store' });
        const d = await res.json();
        connText.textContent = 'LIVE · ' + d.wifiMode; pill.className = 'status-pill live';
        const hazard = d.quakeActive || d.smokeActive;

        const isLightQuake = d.quakeActive && d.estMagnitude < 5.0;
        const qTone = !d.quakeActive ? (d.adxlReady ? 'ok' : 'warn') : (isLightQuake ? 'warn' : 'danger');
        document.getElementById('quakeBox').className = 'status-box ' + qTone;
        document.getElementById('quakeStatusText').textContent = d.quakeActive ? (isLightQuake ? 'LIGHT EARTHQUAKE' : 'EARTHQUAKE!') : (d.adxlReady ? 'NORMAL' : 'NO SENSOR');
        document.getElementById('quakeMetrics').innerHTML =
          `<span class="metric-chip">MMI ${d.intensityMmi} · ${d.intensityLabel}</span><span class="metric-chip">Est. M ${d.estMagnitude.toFixed(1)} · ${d.magnitudeClass}</span>`;
        updateSegBar(d.estMagnitude);
        document.getElementById('pgaMeta').textContent = `Peak PGA: ${d.peakTremorG.toFixed(3)} g (${d.pgaCms2.toFixed(1)} cm/s²)`;

        document.getElementById('smokeBox').className = 'status-box ' + (d.smokeActive ? 'danger' : 'ok');
        document.getElementById('smokeVal').textContent = d.smokeActive ? 'SMOKE DETECTED!' : 'CLEAR';
        document.getElementById('smokeMeta').textContent = d.smokeActive ? 'Evacuate — Smoke Alarm Active' : 'No Smoke Detected';
        document.getElementById('smokeSignal').textContent = 'JGN-301-A · ' + (d.smokeRawSignal ? 'ALARMING' : 'Standby');

        const doorEl = document.getElementById('doorVal');
        doorEl.textContent = hazard ? 'EMERGENCY UNLOCKED' : (d.doorUnlocked ? 'UNLOCKED (NORMAL)' : 'LOCKED');
        doorEl.className = 'door-state-word ' + (hazard ? 'danger' : (d.doorUnlocked ? '' : 'warn'));
        document.getElementById('btnUnlock').disabled = d.doorUnlocked;
        document.getElementById('btnLock').disabled = hazard || !d.doorUnlocked;
        if (hazard) triggerPhoneAlert();
      } catch (_) {
        connText.textContent = 'OFFLINE'; pill.className = 'status-pill err';
        resetUiToStandby();
      }
    }

    async function fetchServerLogs() {
      try {
        const res = await fetchWithTimeout('/api/logs', { cache: 'no-store' }, 2000);
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
        const res = await fetchWithTimeout('/api/logs', { cache: 'no-store' }, 2500);
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
    async function clearServerLogs() { try { await fetchWithTimeout('/api/logs/clear', { method: 'POST' }, 2000); await fetchServerLogs(); } catch (_) {} }

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
