#include <Arduino.h>
#include <ESP32Servo.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

// ==========================================================
// WIFI
// ==========================================================
const char *ssid = "PHAM_MINH_DUC";
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// ==========================================================
// PHẦN CỨNG
// ==========================================================
#define SERVO_PIN 13
#define ESC_PIN 14
#define RGB_LED_PIN 48

// ==========================================================
// GÓC LÁI THỰC TẾ
// ==========================================================
#define SERVO_MIN 60
#define SERVO_MAX 120
#define SERVO_CENTER 90

// ==========================================================
// XUNG ESC WP-1040 (ĐÃ FIX LẠI ĐỂ GA NHẠY HƠN)
// ==========================================================
// Bỏ qua vùng chết 1500 ± 50us của ESC
const int FWD_MIN_US = 1550;
const int FWD_MAX_US = 1680;

const int REV_MIN_US = 1450;
const int REV_MAX_US = 1200;

const int STOP_US = 1500;

Servo steeringServo;
Servo escMotor;

int targetFWD = 0;
int targetREV = 0;
int currentFWD = 0;
int currentREV = 0;

int targetSteer = 90;
float currentSteerFloat = 90.0;
int lastSteer = -1;
int lastPulseUs = -1;

unsigned long lastRecvTime = 0;
const unsigned long LINK_TIMEOUT = 1000;

// WEBSOCKET STATE
bool wsConnected = false;
unsigned long lastWsTelemetry = 0;
const unsigned long WS_TELEMETRY_INTERVAL = 100; // 10Hz

// TELEMETRY DATA (static buffers, no heap allocation)
float telemetrySpeed = 0.0;
int telemetrySteer = 90;
int telemetryPulse = 1500;
int telemetryRssi = 0;

unsigned long previousMillis = 0;
unsigned long rampMillis = 0;
bool ledState = false;

// ==========================================================
// ESC STATE MACHINE
// ==========================================================
enum EscState : uint8_t {
  ESC_IDLE,
  ESC_FWD,
  ESC_BRAKE,
  ESC_NEUTRAL_HOLD,
  ESC_REV
};
EscState escState = ESC_IDLE;
unsigned long escStateTimer = 0;

// Đã rút ngắn thời gian lừa ESC để xe lùi nhanh hơn
const unsigned long BRAKE_HOLD_MS = 250;
const unsigned long NEUTRAL_HOLD_MS = 100;

// ==========================================================
// WEB INTERFACE
// ==========================================================
const char PAGE_INDEX[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover">
  <meta name="apple-mobile-web-app-capable" content="yes">
  <meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
  <meta name="apple-mobile-web-app-title" content="RC PRO">
  <title>RC Pro Controller</title>
  <style>
    :root { --bg: #0b0c10; --accent-v: #00e5ff; --accent-h: #ffaa00; --surface: #1f2833; --glass: rgba(31, 40, 51, 0.4); }
    body { margin: 0; padding: 0; color: white; background: radial-gradient(circle at center, #1a222a 0%, #050508 100%); font-family: 'Segoe UI', Roboto, sans-serif; overscroll-behavior: none; touch-action: none; user-select: none; -webkit-user-select: none; height: 100vh; width: 100vw; overflow: hidden; padding-left: env(safe-area-inset-left); padding-right: env(safe-area-inset-right); }
    @media screen and (orientation: portrait) { #warning { display: flex !important; height: 100%; width: 100%; flex-direction: column; align-items: center; justify-content: center; background: #d32f2f; font-size: 20px; font-weight: bold; text-align: center; padding: 20px; letter-spacing: 1px; } #app { display: none !important; } }
    #warning { display: none; }
    #app { display: flex; flex-direction: row; justify-content: space-between; align-items: center; height: 100%; padding: 0 40px; box-sizing: border-box; }
    .track-v { width: 80px; height: 75%; background: rgba(0,0,0,0.6); border-radius: 40px; position: relative; border: 2px solid rgba(255,255,255,0.05); box-shadow: inset 0 10px 20px rgba(0,0,0,0.8), 0 0 15px rgba(0,229,255,0.1); }
    .track-v::before { content: "FWD"; position: absolute; top: 15px; left: 50%; transform: translateX(-50%); color: rgba(255,255,255,0.4); font-size: 13px; font-weight: bold; }
    .track-v::after { content: "REV"; position: absolute; bottom: 15px; left: 50%; transform: translateX(-50%); color: rgba(255,255,255,0.4); font-size: 13px; font-weight: bold; }
    .knob-v { width: 70px; height: 70px; border-radius: 50%; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); background: radial-gradient(circle at 30% 30%, #4a5568, #1a202c); border: 2px solid var(--accent-v); pointer-events: none; box-shadow: 0 10px 15px rgba(0,0,0,0.6), inset 0 2px 4px rgba(255,255,255,0.3), 0 0 20px rgba(0,229,255,0.4); display: flex; justify-content: center; align-items: center; }
    .knob-v::after { content: "▲\A▼"; white-space: pre; font-size: 10px; color: var(--accent-v); line-height: 12px; text-align: center; }
    .track-h { width: 240px; height: 80px; background: rgba(0,0,0,0.6); border-radius: 40px; position: relative; border: 2px solid rgba(255,255,255,0.05); box-shadow: inset 0 10px 20px rgba(0,0,0,0.8), 0 0 15px rgba(255,170,0,0.1); }
    .track-h::before { content: "L"; position: absolute; left: 20px; top: 50%; transform: translateY(-50%); color: rgba(255,255,255,0.4); font-size: 16px; font-weight: bold; }
    .track-h::after { content: "R"; position: absolute; right: 20px; top: 50%; transform: translateY(-50%); color: rgba(255,255,255,0.4); font-size: 16px; font-weight: bold; }
    .knob-h { width: 70px; height: 70px; border-radius: 50%; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); background: radial-gradient(circle at 30% 30%, #4a5568, #1a202c); border: 2px solid var(--accent-h); pointer-events: none; box-shadow: 0 10px 15px rgba(0,0,0,0.6), inset 0 2px 4px rgba(255,255,255,0.3), 0 0 20px rgba(255,170,0,0.4); display: flex; justify-content: center; align-items: center; }
    .knob-h::after { content: "◀ ▶"; font-size: 12px; color: var(--accent-h); letter-spacing: 2px; }
    .dashboard { display: flex; flex-direction: column; align-items: center; pointer-events: none; background: var(--glass); backdrop-filter: blur(10px); -webkit-backdrop-filter: blur(10px); padding: 15px 30px; border-radius: 20px; border: 1px solid rgba(255,255,255,0.1); box-shadow: 0 8px 32px rgba(0,0,0,0.5); }
    .dashboard h1 { margin: 0 0 10px 0; font-size: 16px; color: #45a29e; letter-spacing: 4px; text-transform: uppercase; border-bottom: 1px solid rgba(69,162,158,0.3); padding-bottom: 5px; }
    .hud { font-size: 12px; color: #8795a1; margin-top: 5px; font-family: 'Courier New', monospace; letter-spacing: 1px; display: flex; justify-content: space-between; width: 100%; align-items: center; }
    .hud span { color: #fff; font-weight: bold; font-size: 20px; text-shadow: 0 0 10px rgba(255,255,255,0.5); margin-left: 15px; }
    .hud.throt span { color: var(--accent-v); text-shadow: 0 0 10px var(--accent-v); }
    .hud.steer span { color: var(--accent-h); text-shadow: 0 0 10px var(--accent-h); }
  </style>
</head>
<body>
<div id="warning"><span>HỆ THỐNG YÊU CẦU MÀN HÌNH NGANG</span></div>
<div id="app">
    <div class="track-v" id="thrArea"><div class="knob-v" id="thrKnob"></div></div>
    <div class="dashboard">
        <h1>TELEMETRY</h1>
        <div class="hud throt">FWD <span><span id="txtFWD">0</span>%</span></div>
        <div class="hud throt">REV <span><span id="txtREV">0</span>%</span></div>
        <div class="hud steer">STEER <span><span id="txtStr">90</span>°</span></div>
        <div class="hud throt" id="speedHud">SPEED <span><span id="txtSpd">0.0</span> km/h</span></div>
        <div class="hud steer" id="rssiHud">RSSI <span><span id="txtRssi">---</span> dBm</span></div>
        <div class="hud throt" id="linkHud">LINK <span id="txtLink" style="color:#00ff00;">●</span></div>
    </div>
    <div class="track-h" id="strArea"><div class="knob-h" id="strKnob"></div></div>
</div>
<script>
document.addEventListener('touchmove', function(e) { e.preventDefault(); }, { passive: false });
let FWD = 0, REV = 0, steering = 90;
let lastFWD = -1, lastREV = -1, lastSteering = -1;
let lastSpeed = -1, lastRssi = -1, lastLink = -1;

const thrArea = document.getElementById('thrArea');
const thrKnob = document.getElementById('thrKnob');
let thrCenter = 0, thrMax = 0;

thrArea.addEventListener('touchstart', (e) => { let rect = thrArea.getBoundingClientRect(); thrCenter = rect.top + rect.height / 2; thrMax = (rect.height / 2) - 35; updateThr(e.targetTouches[0].clientY); });
thrArea.addEventListener('touchmove', (e) => { if (e.targetTouches.length > 0) updateThr(e.targetTouches[0].clientY); });
thrArea.addEventListener('touchend', () => { thrKnob.style.transform = 'translate(-50%, -50%)'; FWD = 0; REV = 0; document.getElementById('txtFWD').innerText = FWD; document.getElementById('txtREV').innerText = REV; });

function updateThr(y) {
    let dy = y - thrCenter;
    dy = Math.max(-thrMax, Math.min(thrMax, dy));
    thrKnob.style.transform = `translate(-50%, calc(-50% + ${dy}px))`;
    if (dy < 0) { FWD = Math.round((-dy / thrMax) * 100); REV = 0; }
    else if (dy > 0) { FWD = 0; REV = Math.round((dy / thrMax) * 100); }
    else { FWD = 0; REV = 0; }
    document.getElementById('txtFWD').innerText = FWD;
    document.getElementById('txtREV').innerText = REV;
}

const strArea = document.getElementById('strArea');
const strKnob = document.getElementById('strKnob');
let strCenter = 0, strMax = 0;

strArea.addEventListener('touchstart', (e) => { let rect = strArea.getBoundingClientRect(); strCenter = rect.left + rect.width / 2; strMax = (rect.width / 2) - 35; updateStr(e.targetTouches[0].clientX); });
strArea.addEventListener('touchmove', (e) => { if (e.targetTouches.length > 0) updateStr(e.targetTouches[0].clientX); });
strArea.addEventListener('touchend', () => { strKnob.style.transform = 'translate(-50%, -50%)'; steering = 90; document.getElementById('txtStr').innerText = steering; });

function updateStr(x) {
    let dx = x - strCenter;
    dx = Math.max(-strMax, Math.min(strMax, dx));
    strKnob.style.transform = `translate(calc(-50% + ${dx}px), -50%)`;
    steering = Math.round(90 + (dx / strMax) * 30);
    steering = Math.max(60, Math.min(120, steering));
    document.getElementById('txtStr').innerText = steering;
}

// WEBSOCKET REAL-TIME
let ws = null;
function connectWS() {
    ws = new WebSocket(`ws://${location.host}:81`);
    ws.onopen = function() { console.log('[WS] Connected'); };
    ws.onclose = function() { console.log('[WS] Disconnected, reconnecting...'); setTimeout(connectWS, 1000); };
    ws.onerror = function(e) { console.log('[WS] Error:', e); };
    ws.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            if (data.cmd === 'telemetry') {
                // Update speed
                const spd = data.speed.toFixed(1);
                if (spd !== lastSpeed) {
                    document.getElementById('txtSpd').innerText = spd;
                    lastSpeed = spd;
                }
                // Update RSSI
                if (data.rssi !== lastRssi) {
                    document.getElementById('txtRssi').innerText = data.rssi;
                    lastRssi = data.rssi;
                }
                // Update link status
                const link = data.status === 'ONLINE' ? '●' : '○';
                const linkColor = data.status === 'ONLINE' ? '#00ff00' : '#ff4444';
                if (link !== lastLink) {
                    const linkEl = document.getElementById('txtLink');
                    linkEl.innerText = link;
                    linkEl.style.color = linkColor;
                    lastLink = link;
                }
            }
        } catch (e) { /* ignore malformed */ }
    };
}
connectWS();

// Send control data via WebSocket (real-time, no polling delay)
function sendControl() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({cmd: "control", fwd: FWD, rev: REV, steer: steering}));
    } else {
        // Fallback to HTTP /api if WebSocket unavailable
        fetch(`/api?f=${FWD}&r=${REV}&t=${steering}`).catch(() => {});
    }
}

// Send immediately on control change (no 80ms debounce)
function checkControlChange() {
    if (FWD !== lastFWD || REV !== lastREV || steering !== lastSteering) {
        sendControl();
        lastFWD = FWD; lastREV = REV; lastSteering = steering;
    }
}
setInterval(checkControlChange, 20); // Fast check but WS sends instantly on change
</script>
</body>
</html>
)rawliteral";

// ==========================================================
// CẬP NHẬT ESC
// ==========================================================
void updateEscOutput() {
  unsigned long now = millis();
  int pulse = STOP_US;

  switch (escState) {
  case ESC_IDLE:
    pulse = STOP_US;
    if (targetFWD > 0) {
      escState = ESC_FWD;
    } else if (targetREV > 0) {
      escState = ESC_BRAKE;
      escStateTimer = now;
    }
    break;

  case ESC_FWD:
    if (targetREV > 0) {
      escState = ESC_BRAKE;
      escStateTimer = now;
      currentFWD = 0; // Cắt mô-men tiến lập tức
      break;
    }
    if (currentFWD > 0) {
      pulse = map(currentFWD, 1, 100, FWD_MIN_US, FWD_MAX_US);
    } else {
      escState = ESC_IDLE;
      pulse = STOP_US;
    }
    break;

  case ESC_BRAKE:
    // Đảo chiều nhanh: Nếu đang phanh mà user vuốt lên FWD thì cho chạy tiến
    // liền
    if (targetFWD > 0) {
      escState = ESC_FWD;
      currentREV = 0;
      break;
    }
    // Nhả tay hoàn toàn thì về IDLE
    if (targetREV == 0 && currentREV == 0) {
      escState = ESC_IDLE;
      break;
    }

    // Phanh mượt theo lực kéo hiện tại
    if (currentREV > 0) {
      pulse = map(currentREV, 1, 100, REV_MIN_US, REV_MAX_US);
    }

    // [LỖI CŨ Ở ĐÂY - ĐÃ FIX]: Tự động chuyển qua NEUTRAL sau 250ms
    // BẤT KỂ NGƯỜI DÙNG CÓ ĐANG GIỮ JOYSTICK LÙI HAY KHÔNG
    if (now - escStateTimer >= BRAKE_HOLD_MS) {
      escState = ESC_NEUTRAL_HOLD;
      escStateTimer = now;
    }
    break;

  case ESC_NEUTRAL_HOLD:
    pulse = STOP_US;
    if (targetFWD > 0) {
      escState = ESC_FWD;
      currentREV = 0;
    } else if (now - escStateTimer >= NEUTRAL_HOLD_MS) {
      escState = ESC_REV;
    }
    break;

  case ESC_REV:
    // Từ Lùi lên Tiến không cần quy trình phanh của ESC
    if (targetFWD > 0) {
      escState = ESC_FWD;
      currentREV = 0;
    } else if (targetREV == 0 && currentREV == 0) {
      escState = ESC_IDLE;
      pulse = STOP_US;
    } else if (currentREV > 0) {
      pulse = map(currentREV, 1, 100, REV_MIN_US, REV_MAX_US);
    }
    break;
  }

  if (pulse != lastPulseUs) {
    escMotor.writeMicroseconds(pulse);
    lastPulseUs = pulse;
  }
}

// ==========================================================
// WEBSOCKET
// ==========================================================
void sendTelemetry() {
  if (!wsConnected) return;

  char msg[160];
  snprintf(msg, sizeof(msg),
           "{\"cmd\":\"telemetry\",\"speed\":%.1f,\"steer\":%d,\"pulse\":%d,\"rssi\":%d,\"status\":\"%s\"}",
           telemetrySpeed, telemetrySteer, telemetryPulse, telemetryRssi,
           (millis() - lastRecvTime < LINK_TIMEOUT) ? "ONLINE" : "LOST");

  webSocket.broadcastTXT(msg);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_DISCONNECTED:
    wsConnected = false;
    break;

  case WStype_CONNECTED:
    wsConnected = true;
    Serial.println("[WS] Client connected");
    // Send initial telemetry immediately
    sendTelemetry();
    break;

  case WStype_TEXT:
    if (length > 0 && payload[0] == '{') {
      // Parse simple JSON control message:
      // {"cmd":"control","fwd":N,"rev":N,"steer":M}
      int fwd = -1, rev = -1, steer = -1;

      // Find values after each key
      char *p = (char *)payload;

      char *fpos = strstr(p, "\"fwd\"");
      if (fpos) {
        char *colon = strchr(fpos, ':');
        if (colon) fwd = atoi(colon + 1);
      }

      char *rpos = strstr(p, "\"rev\"");
      if (rpos) {
        char *colon = strchr(rpos, ':');
        if (colon) rev = atoi(colon + 1);
      }

      char *tpos = strstr(p, "\"steer\"");
      if (tpos) {
        char *colon = strchr(tpos, ':');
        if (colon) steer = atoi(colon + 1);
      }

      if (fwd >= 0 || rev >= 0 || steer >= 0) {
        if (fwd >= 0) targetFWD = constrain(fwd, 0, 100);
        if (rev >= 0) targetREV = constrain(rev, 0, 100);
        if (steer >= 0) targetSteer = constrain(steer, 60, 120);

        if (targetFWD > 0 && targetREV > 0) {
          targetFWD = 0;
          targetREV = 0;
        }

        lastRecvTime = millis();
      }
    }
    break;
  }
}

// ==========================================================
// SETUP
// ==========================================================
void setup() {
  Serial.begin(115200);

  neopixelWrite(RGB_LED_PIN, 0, 0, 0);

  // Only allocate timers we actually need (2 timers for servo and ESC)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  steeringServo.write(SERVO_CENTER);
  currentSteerFloat = SERVO_CENTER;
  targetSteer = SERVO_CENTER;
  lastSteer = SERVO_CENTER;

  escMotor.setPeriodHertz(50);
  escMotor.attach(ESC_PIN, 1000, 2000);

  Serial.println("[SETUP] ESC Neutral 1500us...");
  escMotor.writeMicroseconds(STOP_US);
  lastPulseUs = STOP_US;
  delay(3000);

  // WiFi optimization: use less congested channel and reduce power if needed
  WiFi.mode(WIFI_AP);
  // Using channel 6 (typically less crowded than 1) and reducing power to 20.5dBm (default is 20.5dBm anyway, but being explicit)
  WiFi.softAP(ssid, NULL, 6, 0, 4); // Channel 6, hidden SSID, max 4 connections
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Reduce TX power slightly to reduce interference

  Serial.print("[WIFI] IP: ");
  Serial.println(WiFi.softAPIP());

  // WEBSOCKET SERVER
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("[WS] WebSocket server started on port 81");

  server.on("/", HTTP_GET,
            []() { server.send_P(200, "text/html", PAGE_INDEX); });

  server.on("/api", HTTP_GET, []() {
    if (server.hasArg("f") && server.hasArg("r") && server.hasArg("t")) {
      targetFWD = constrain(server.arg("f").toInt(), 0, 100);
      targetREV = constrain(server.arg("r").toInt(), 0, 100);
      targetSteer = constrain(server.arg("t").toInt(), 60, 120);

      if (targetFWD > 0 && targetREV > 0) {
        targetFWD = 0;
        targetREV = 0;
      }
      lastRecvTime = millis();
    }
    server.send(200, "text/plain", "OK");
  });

  server.onNotFound([]() { server.send(404, "text/plain", "404 Not Found"); });
  server.begin();
  Serial.println("HE THONG DA SAN SANG!");
}

// ==========================================================
// LOOP
// ==========================================================
void loop() {
  server.handleClient();
  webSocket.loop();
  unsigned long now = millis();

  // FAILSAFE
  if (now - lastRecvTime > LINK_TIMEOUT) {
    targetFWD = 0;
    targetREV = 0;
    targetSteer = SERVO_CENTER;
  }

  if (now - rampMillis >= 15) {
    rampMillis = now;

    // SMOOTH ACCELERATION/DECELERATION WITH EXPONENTIAL APPROACH
    // More natural feeling - slower when close to target, faster when far
    const float ACCEL_FACTOR = 0.1f;   // Adjust for responsiveness (0.1 = ~10% of diff per step)
    const float DECEL_FACTOR = 0.15f;  // Slightly more aggressive deceleration

    // --- MƯỢT GA FWD ---
    if (currentFWD < targetFWD) {
      int diff = targetFWD - currentFWD;
      currentFWD += max(1, (int)round(diff * ACCEL_FACTOR));
      if (currentFWD > targetFWD) currentFWD = targetFWD;
    } else if (currentFWD > targetFWD) {
      int diff = currentFWD - targetFWD;
      currentFWD -= max(1, (int)round(diff * DECEL_FACTOR));
      if (currentFWD < targetFWD) currentFWD = targetFWD;
    }

    // --- MƯỢT GA REV & BRAKE ---
    if (currentREV < targetREV) {
      int diff = targetREV - currentREV;
      currentREV += max(1, (int)round(diff * ACCEL_FACTOR));
      if (currentREV > targetREV) currentREV = targetREV;
    } else if (currentREV > targetREV) {
      int diff = currentREV - targetREV;
      currentREV -= max(1, (int)round(diff * DECEL_FACTOR));
      if (currentREV < targetREV) currentREV = targetREV;
    }

    updateEscOutput();

    // --- SERVO SMOOTH ---
    // targetSteer is already 60-120 (= SERVO_MIN-SERVO_MAX), apply smoothing
    const float SMOOTH_FACTOR = 0.15f;
    currentSteerFloat += (targetSteer - currentSteerFloat) * SMOOTH_FACTOR;
    int finalSteer = constrain(round(currentSteerFloat), SERVO_MIN, SERVO_MAX);

    if (finalSteer != lastSteer) {
      steeringServo.write(finalSteer);
      lastSteer = finalSteer;
    }

    // Update telemetry data
    telemetrySteer = finalSteer;
    telemetryPulse = lastPulseUs;
  }

  // Update telemetry speed (calculate from throttle)
  if (currentFWD > 0) {
    telemetrySpeed = map(currentFWD, 1, 100, 0.5, 15.0); // Approximate kmh
  } else if (currentREV > 0) {
    telemetrySpeed = -map(currentREV, 1, 100, 0.5, 15.0); // Negative for reverse
  } else {
    telemetrySpeed = 0.0;
  }

  // Send telemetry via WebSocket
  if (wsConnected && (now - lastWsTelemetry >= WS_TELEMETRY_INTERVAL)) {
    lastWsTelemetry = now;
    telemetryRssi = WiFi.RSSI();
    sendTelemetry();
  }

  // RGB LED
  if (currentFWD == 0 && currentREV == 0 && lastSteer == SERVO_CENTER) {
    neopixelWrite(RGB_LED_PIN, 0, 0, 0);
    ledState = false;
  } else {
    int blinkInterval = 500;
    uint8_t r = 0, g = 0, b = 0;

    if (currentFWD > 0) {
      blinkInterval = map(currentFWD, 1, 100, 600, 50);
      g = 255;
    } else if (currentREV > 0) {
      blinkInterval = map(currentREV, 1, 100, 600, 50);
      r = 255;
    } else if (abs(lastSteer - SERVO_CENTER) > 1) {
      blinkInterval = 250;
      r = 255;
      g = 120;
    }

    if (now - previousMillis >= blinkInterval) {
      previousMillis = now;
      ledState = !ledState;
      if (ledState)
        neopixelWrite(RGB_LED_PIN, r, g, b);
      else
        neopixelWrite(RGB_LED_PIN, 0, 0, 0);
    }
  }

  // Yield to WiFi stack to prevent starvation and reduce interference
  yield();
}