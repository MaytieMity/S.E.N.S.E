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
// Change to HIGH if you use an Active-HIGH relay or logic-level MOSFET.
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
constexpr float    QUAKE_THRESHOLD_G   = 0.12f;  // Lower = more sensitive (0.08 - 0.25 g)
constexpr uint8_t  QUAKE_HITS_REQUIRED = 4;      // Tremor samples required within window
constexpr uint32_t QUAKE_WINDOW_MS     = 1500;   // Filters out single accidental knocks
constexpr uint32_t QUAKE_SAMPLE_MS     = 25;     // 40 Hz seismic sampling

constexpr uint32_t SMOKE_DEBOUNCE_MS   = 40;     // Confirm real alarm pulse
constexpr uint32_t SMOKE_HOLD_MS       = 8000;   // Bridge gaps between JGN-301-A beeps

// true  = Door stays unlocked until Reset button or Phone Re-Lock is pressed
// false = Door auto-locks after DOOR_AUTO_LOCK_MS once both hazards clear
constexpr bool     LATCH_UNLOCK_UNTIL_RESET = true;
constexpr uint32_t DOOR_AUTO_LOCK_MS        = 30000;
constexpr uint32_t TELEMETRY_MS             = 1000;

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

bool     doorUnlocked     = false;
uint32_t lastHazardMs     = 0;

// Loop timers
uint32_t lastQuakeSampleMs = 0;
uint32_t lastTelemetryMs   = 0;

// -------------------- Embedded Mobile Web App --------------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
  <meta name="theme-color" content="#0b1220" />
  <meta name="apple-mobile-web-app-capable" content="yes" />
  <link rel="manifest" href="/manifest.webmanifest" />
  <title>S.E.N.S.E. Phone Monitor</title>
  <style>
    :root {
      --bg: #0b1220;
      --card: #131d33;
      --border: #233252;
      --text: #f1f5f9;
      --muted: #94a3b8;
      --ok: #22c55e;
      --warn: #f59e0b;
      --danger: #ef4444;
      --accent: #38bdf8;
    }
    * { box-sizing: border-box; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
    body { margin: 0; background: var(--bg); color: var(--text); padding: 14px; }
    .wrap { max-width: 520px; margin: 0 auto; display: grid; gap: 12px; }
    header { display: flex; justify-content: space-between; align-items: center; padding: 4px 2px; }
    h1 { margin: 0; font-size: 1.15rem; letter-spacing: 0.04em; }
    .pill { font-size: 0.78rem; padding: 4px 10px; border-radius: 999px; background: #1e293b; color: var(--muted); border: 1px solid var(--border); }
    .pill.live { color: #86efac; border-color: rgba(34,197,94,0.45); background: rgba(34,197,94,0.12); }
    .pill.err { color: #fca5a5; border-color: rgba(239,68,68,0.45); background: rgba(239,68,68,0.12); }
    .banner { padding: 16px; border-radius: 16px; border: 1px solid var(--border); background: var(--card); transition: 0.25s; }
    .banner.ok { border-color: rgba(34,197,94,0.5); background: linear-gradient(135deg, rgba(34,197,94,0.16), var(--card)); }
    .banner.alarm { border-color: rgba(239,68,68,0.8); background: linear-gradient(135deg, rgba(239,68,68,0.28), var(--card)); animation: pulse 1.2s infinite; }
    @keyframes pulse { 0%,100% { box-shadow: 0 0 0 0 rgba(239,68,68,0.35); } 50% { box-shadow: 0 0 0 8px rgba(239,68,68,0); } }
    .banner-title { font-size: 1.25rem; font-weight: 700; margin-bottom: 4px; }
    .banner-sub { font-size: 0.88rem; color: var(--muted); }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .card { background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 14px; }
    .card.full { grid-column: 1 / -1; }
    .label { font-size: 0.76rem; text-transform: uppercase; letter-spacing: 0.05em; color: var(--muted); margin-bottom: 6px; }
    .value { font-size: 1.2rem; font-weight: 700; }
    .value.ok { color: var(--ok); }
    .value.danger { color: var(--danger); }
    .value.warn { color: var(--warn); }
    .meta { font-size: 0.82rem; color: var(--muted); margin-top: 6px; }
    .bar { height: 7px; background: #0f172a; border-radius: 999px; overflow: hidden; margin-top: 8px; }
    .bar > div { height: 100%; width: 0%; background: var(--accent); transition: width 0.2s, background 0.2s; }
    .actions { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 10px; }
    button { border: 0; border-radius: 12px; padding: 12px 10px; font-size: 0.92rem; font-weight: 600; cursor: pointer; }
    button:disabled { opacity: 0.45; cursor: not-allowed; }
    .btn-unlock { background: var(--warn); color: #111827; }
    .btn-lock { background: #1e293b; color: var(--text); border: 1px solid var(--border); }
    .toggle-row { display: flex; justify-content: space-between; align-items: center; font-size: 0.88rem; }
    ul { margin: 8px 0 0; padding: 0; list-style: none; max-height: 150px; overflow-y: auto; font-size: 0.82rem; color: var(--muted); }
    li { padding: 5px 0; border-bottom: 1px solid rgba(148,163,184,0.12); }
  </style>
</head>
<body>
  <div class="wrap">
    <header>
      <h1>S.E.N.S.E. MONITOR</h1>
      <span id="connPill" class="pill">Connecting...</span>
    </header>

    <div id="banner" class="banner ok">
      <div id="bannerTitle" class="banner-title">System Armed</div>
      <div id="bannerSub" class="banner-sub">Waiting for sensor telemetry...</div>
    </div>

    <div class="grid">
      <div class="card">
        <div class="label">Earthquake (ADXL345)</div>
        <div id="quakeVal" class="value ok">NORMAL</div>
        <div id="quakeMeta" class="meta">0.000 g | Peak: 0.000 g</div>
        <div class="bar"><div id="quakeBar"></div></div>
      </div>

      <div class="card">
        <div class="label">Smoke (JGN-301-A)</div>
        <div id="smokeVal" class="value ok">CLEAR</div>
        <div id="smokeMeta" class="meta">Detector: Standby</div>
      </div>

      <div class="card full">
        <div class="label">Electric Doorknob</div>
        <div id="doorVal" class="value ok">LOCKED</div>
        <div id="doorMeta" class="meta">Auto-unlocks immediately on earthquake or smoke</div>
        <div class="actions">
          <button class="btn-unlock" onclick="sendDoorAction('unlock')">Emergency Unlock</button>
          <button id="btnLock" class="btn-lock" onclick="sendDoorAction('lock')">Re-Lock Door</button>
        </div>
      </div>

      <div class="card full">
        <div class="toggle-row">
          <span>Phone Alert Sound &amp; Vibration</span>
          <input id="alertToggle" type="checkbox" checked style="width:18px;height:18px;" />
        </div>
      </div>

      <div class="card full">
        <div class="label">Event Log</div>
        <ul id="logList"></ul>
      </div>
    </div>
  </div>

  <script>
    let prevKey = '';
    let lastBeepAt = 0;

    function addLog(msg) {
      const li = document.createElement('li');
      li.textContent = new Date().toLocaleTimeString() + ' — ' + msg;
      const list = document.getElementById('logList');
      list.prepend(li);
      while (list.children.length > 20) list.removeChild(list.lastChild);
    }

    function triggerPhoneAlert() {
      if (!document.getElementById('alertToggle').checked) return;
      const now = Date.now();
      if (now - lastBeepAt < 3000) return;
      lastBeepAt = now;
      if (navigator.vibrate) navigator.vibrate([300, 150, 300]);
      try {
        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        const osc = ctx.createOscillator();
        const gain = ctx.createGain();
        osc.type = 'square';
        osc.frequency.value = 880;
        gain.gain.value = 0.08;
        osc.connect(gain);
        gain.connect(ctx.destination);
        osc.start();
        setTimeout(() => { osc.stop(); ctx.close(); }, 250);
      } catch (_) {}
    }

    async function sendDoorAction(action) {
      try {
        const res = await fetch('/api/' + action, { method: 'POST' });
        const data = await res.json();
        addLog(data.message || ('Door ' + action));
        await pollStatus();
      } catch (e) {
        addLog('Command failed: ' + e.message);
      }
    }

    async function pollStatus() {
      const pill = document.getElementById('connPill');
      try {
        const res = await fetch('/api/status', { cache: 'no-store' });
        const d = await res.json();

        pill.textContent = 'LIVE (' + d.wifiMode + ')';
        pill.className = 'pill live';

        const hazard = d.quakeActive || d.smokeActive;
        const banner = document.getElementById('banner');
        banner.className = 'banner ' + (hazard ? 'alarm' : 'ok');
        document.getElementById('bannerTitle').textContent = hazard
          ? 'EMERGENCY: ' + [d.quakeActive ? 'EARTHQUAKE' : '', d.smokeActive ? 'SMOKE' : ''].filter(Boolean).join(' & ')
          : (d.doorUnlocked ? 'Hazard Cleared — Door Unlocked' : 'All Clear — System Armed');
        document.getElementById('bannerSub').textContent =
          'Uptime: ' + d.uptimeSec + 's | IP: ' + d.ip + (d.adxlReady ? '' : ' | ADXL345 offline!');

        const qEl = document.getElementById('quakeVal');
        qEl.textContent = d.quakeActive ? 'EARTHQUAKE!' : (d.adxlReady ? 'NORMAL' : 'NO SENSOR');
        qEl.className = 'value ' + (d.quakeActive ? 'danger' : (d.adxlReady ? 'ok' : 'warn'));
        document.getElementById('quakeMeta').textContent =
          d.currentTremorG.toFixed(3) + ' g | Peak: ' + d.peakTremorG.toFixed(3) + ' g';

        const pct = Math.min(100, Math.round((d.currentTremorG / 0.30) * 100));
        const bar = document.getElementById('quakeBar');
        bar.style.width = pct + '%';
        bar.style.background = d.quakeActive ? 'var(--danger)' : 'var(--accent)';

        const sEl = document.getElementById('smokeVal');
        sEl.textContent = d.smokeActive ? 'SMOKE DETECTED!' : 'CLEAR';
        sEl.className = 'value ' + (d.smokeActive ? 'danger' : 'ok');
        document.getElementById('smokeMeta').textContent =
          'Signal: ' + (d.smokeRawSignal ? 'ALARMING' : 'Standby');

        const doorEl = document.getElementById('doorVal');
        doorEl.textContent = d.doorUnlocked ? 'UNLOCKED' : 'LOCKED';
        doorEl.className = 'value ' + (d.doorUnlocked ? 'warn' : 'ok');
        document.getElementById('btnLock').disabled = hazard || !d.doorUnlocked;

        const key = `${d.quakeActive}-${d.smokeActive}-${d.doorUnlocked}`;
        if (key !== prevKey) {
          prevKey = key;
          addLog(`Quake: ${d.quakeActive ? 'ALARM' : 'OK'}, Smoke: ${d.smokeActive ? 'ALARM' : 'OK'}, Door: ${d.doorUnlocked ? 'UNLOCKED' : 'LOCKED'}`);
        }
        if (hazard) triggerPhoneAlert();
      } catch (e) {
        pill.textContent = 'Offline';
        pill.className = 'pill err';
      }
    }

    pollStatus();
    setInterval(pollStatus, 800);
  </script>
</body>
</html>
)rawliteral";

const char MANIFEST_JSON[] PROGMEM = R"rawliteral({
  "name": "S.E.N.S.E. Monitor",
  "short_name": "SENSE",
  "start_url": "/",
  "display": "standalone",
  "background_color": "#0b1220",
  "theme_color": "#0b1220"
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
void setDoorUnlocked(bool unlock) {
  if (doorUnlocked == unlock) {
    return;
  }
  doorUnlocked = unlock;
  digitalWrite(PIN_DOOR_RELAY, unlock ? RELAY_UNLOCK_LEVEL : RELAY_LOCK_LEVEL);
  Serial.println(unlock ? "[ACTION] Electric Doorknob UNLOCKED!"
                        : "[ACTION] Electric Doorknob LOCKED.");
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

void handleStatusApi() {
  addCorsHeaders();
  bool smokeRaw = (digitalRead(PIN_SMOKE_IN) == SMOKE_ACTIVE_LEVEL);
  String ipStr = usingApMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();

  char json[384];
  snprintf(json, sizeof(json),
           "{\"quakeActive\":%s,\"adxlReady\":%s,\"currentTremorG\":%.4f,\"peakTremorG\":%.4f,"
           "\"smokeActive\":%s,\"smokeRawSignal\":%s,\"doorUnlocked\":%s,\"latched\":%s,"
           "\"uptimeSec\":%lu,\"wifiMode\":\"%s\",\"ip\":\"%s\"}",
           quakeActive ? "true" : "false",
           adxlReady ? "true" : "false",
           currentTremorG,
           peakTremorG,
           smokeActive ? "true" : "false",
           smokeRaw ? "true" : "false",
           doorUnlocked ? "true" : "false",
           LATCH_UNLOCK_UNTIL_RESET ? "true" : "false",
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
      if (!quakeActive) {
        Serial.printf("[ALERT] EARTHQUAKE DETECTED! Tremor: %.3f g\n", currentTremorG);
      }
      quakeActive = true;
      lastQuakeMs = now;
      lastHazardMs = now;
    }
  }

  if (quakeActive && (now - lastQuakeMs > 5000)) {
    quakeActive = false;
    quakeHits = 0;
    Serial.println("[INFO] Seismic tremor stopped.");
  }
}

void updateSmokeSensor(uint32_t now) {
  bool rawSignal = (digitalRead(PIN_SMOKE_IN) == SMOKE_ACTIVE_LEVEL);

  if (rawSignal) {
    if (smokeActiveStart == 0) {
      smokeActiveStart = now;
    }
    if (now - smokeActiveStart >= SMOKE_DEBOUNCE_MS) {
      if (!smokeActive) {
        Serial.println("[ALERT] SMOKE DETECTED from JGN-301-A!");
      }
      smokeActive = true;
      lastSmokeSeenMs = now;
      lastHazardMs = now;
    }
  } else {
    smokeActiveStart = 0;
    if (smokeActive && (now - lastSmokeSeenMs > SMOKE_HOLD_MS)) {
      smokeActive = false;
      Serial.println("[INFO] JGN-301-A smoke signal cleared.");
    }
  }
}

void updateDoorAndStatus(uint32_t now) {
  if (quakeActive || smokeActive) {
    setDoorUnlocked(true);
  } else if (doorUnlocked && !LATCH_UNLOCK_UNTIL_RESET &&
             (now - lastHazardMs >= DOOR_AUTO_LOCK_MS)) {
    setDoorUnlocked(false);
  }

  if (doorUnlocked) {
    digitalWrite(PIN_LED_STATUS, (now / 150) % 2);
  } else {
    digitalWrite(PIN_LED_STATUS, (now % 2000) < 150 ? HIGH : LOW);
  }
}

void handleResetButton(uint32_t now) {
  static bool prevPressed = false;
  static uint32_t lastDebounceMs = 0;

  bool pressed = (digitalRead(PIN_RESET_BTN) == LOW);
  if (pressed && !prevPressed && (now - lastDebounceMs > 250)) {
    lastDebounceMs = now;
    if (!quakeActive && !smokeActive) {
      quakeHits = 0;
      peakTremorG = 0.0f;
      setDoorUnlocked(false);
      Serial.println("[INPUT] Manual reset: door re-locked.");
    } else {
      Serial.println("[WARN] Cannot re-lock while earthquake or smoke hazard is active.");
    }
  }
  prevPressed = pressed;
}

void printTelemetry(uint32_t now) {
  if (now - lastTelemetryMs < TELEMETRY_MS) {
    return;
  }
  lastTelemetryMs = now;

  Serial.printf("Status | Quake: %s (%.3fg) | Smoke: %s | Door: %s\n",
                quakeActive ? "ALARM" : "OK",
                currentTremorG,
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
  digitalWrite(PIN_DOOR_RELAY, RELAY_LOCK_LEVEL);
  pinMode(PIN_DOOR_RELAY, OUTPUT);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  adxlReady = initAdxl345();
  if (adxlReady) {
    Serial.println("[READY] ADXL345 initialized (I2C 0x53).");
  } else {
    Serial.println("[WARN] ADXL345 not detected! Check SDA(21), SCL(22), and CS->3.3V.");
  }

  setupWiFiAndServer();
}

void loop() {
  uint32_t now = millis();

  server.handleClient();
  handleResetButton(now);
  updateEarthquakeSensor(now);
  updateSmokeSensor(now);
  updateDoorAndStatus(now);
  printTelemetry(now);
}
