#include <ESP8266WiFi.h>  // ESP8266 WiFi + Access Point mode
#include <ESP8266WebServer.h> // simple HTTP server (web pages + endpoints)
#include <Wire.h> // communication (ESP8266 <-> PCA9685)
#include <EEPROM.h>  // save settings (servo angles, delay) in flash
#include <Adafruit_PWMServoDriver.h> // PCA9685 16-channel PWM driver library

// WiFi AP 
const char* AP_SSID = "ColorSort-RobotArm";
const char* AP_PASS = "12345678";
ESP8266WebServer server(80);

// PCA9685 servo driver setup
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
const uint8_t SERVO_CH[5] = {0, 1, 2, 3, 4};

// arm resting position
int HOME_ANGLE[5] = {140, 16, 97, 120, 85};

struct ServoState { int currentAngle; };
ServoState s[5];

// Degrees → PWM ticks
int SERVO_MIN_TICK = 150;
int SERVO_MAX_TICK = 600;

// Motion tuning
const int stepDelayMs = 8;
const int stepSizeDeg = 1;

// PIN LIST
// I2C for PCA9685: SDA=D2, SCL=D1

// Relay (Conveyor)
const uint8_t PIN_RELAY = D6;   // GPIO12

// IR Sensor
const uint8_t PIN_IR = D5;      // GPIO14

// TCS3200
const uint8_t PIN_TCS_OUT = D7; // GPIO13
const uint8_t PIN_TCS_S0  = D3; // GPIO0
const uint8_t PIN_TCS_S1  = D4; // GPIO2
const uint8_t PIN_TCS_S2  = D8; // GPIO15
const uint8_t PIN_TCS_S3  = D0; // GPIO16

// Relay logic (most relay boards are active LOW)
const bool RELAY_ON  = LOW;
const bool RELAY_OFF = HIGH;

// IR logic (many IR sensors output LOW when object detected)
const bool IR_DETECTED_LEVEL = LOW;

// AUTO CONTROL + COUNTERS 
bool autoEnabled = false;
bool conveyorRunning = false;

unsigned long countTotal = 0;
unsigned long countRed = 0;
unsigned long countGreen = 0;
unsigned long countBlue = 0;
unsigned long countUnknown = 0;

String lastColor = "-";

// Relay OFF delay control 
// This delay is applied ONLY when switching relay from ON -> OFF.
uint16_t relayOffDelayMs = 20;   // default was 20 in your code

// TCS3200 enumeration
enum TcsColorFilter { TCSF_RED, TCSF_GREEN, TCSF_BLUE, TCSF_CLEAR };

// State Machine 
//ST_RUNNING: conveyor runs until object detected, then read color
//ST_SORTING_*: do arm sequence to correct bin
//ST_WAIT_CLEAR: wait until object is removed/clears sensor before running again
enum SortState { ST_RUNNING, ST_SORTING_RED, ST_SORTING_GREEN, ST_SORTING_BLUE, ST_WAIT_CLEAR };
SortState sortState = ST_RUNNING;

// EEPROM SAVE (Grab Point + Relay Delay) 
#define EEPROM_SIZE 256
#define CFG_MAGIC   0xA55A12EF

struct AppCfg {
  uint32_t magic;

  // Grab point
  int a0;
  int a2;
  int a3;
  int a4_open;
  int a1;
  int a4_close;

  // Relay off delay (ms)
  uint16_t relayOffDelay;
};

AppCfg cfg = {
  CFG_MAGIC,
  54, 51, 162, 85, 75, 45,
  20
};

void loadCfg() {
  EEPROM.get(0, cfg);
  if (cfg.magic != CFG_MAGIC) {
    cfg = { CFG_MAGIC, 54, 51, 162, 85, 75, 45, 20 };
    EEPROM.put(0, cfg);
    EEPROM.commit();
  }

  cfg.a0 = constrain(cfg.a0, 0, 180);
  cfg.a2 = constrain(cfg.a2, 0, 180);
  cfg.a3 = constrain(cfg.a3, 0, 180);
  cfg.a4_open = constrain(cfg.a4_open, 0, 180);
  cfg.a1 = constrain(cfg.a1, 0, 180);
  cfg.a4_close = constrain(cfg.a4_close, 0, 180);

  // safe range for delay
  cfg.relayOffDelay = constrain(cfg.relayOffDelay, 0, 2000);

  relayOffDelayMs = cfg.relayOffDelay;
}

void saveCfg() {
  cfg.magic = CFG_MAGIC;
  EEPROM.put(0, cfg);
  EEPROM.commit();
  relayOffDelayMs = cfg.relayOffDelay;
}

// Servo Helpers (Servo movements)
int angleToTickForServo(uint8_t idx, int angle) {
  angle = constrain(angle, 0, 180);

  int minT = SERVO_MIN_TICK;
  int maxT = SERVO_MAX_TICK;

  if (idx == 4) {
    minT = 150;
    maxT = 630;
  }

  return map(angle, 0, 180, minT, maxT);
}

//send the PWM signal to PCA9685
void writeServoAngle(uint8_t idx, int angle) {
  int tick = angleToTickForServo(idx, angle);
  pca.setPWM(SERVO_CH[idx], 0, tick);
}

void moveSmoothTo(uint8_t idx, int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);

  int step = stepSizeDeg;
  int dly  = stepDelayMs;

  if (idx == 1) { step = 1; dly = 15; }
  if (idx == 4) { step = 1; dly = 18; }

  while (s[idx].currentAngle != targetAngle) {
    int cur = s[idx].currentAngle;

    if (cur < targetAngle) cur += step;
    else cur -= step;

    if (abs(cur - targetAngle) < step) cur = targetAngle;

    s[idx].currentAngle = cur;
    writeServoAngle(idx, cur);

    delay(dly);
    yield();
  }
}

void moveAllHome() {
  for (int i = 0; i < 5; i++) moveSmoothTo((uint8_t)i, HOME_ANGLE[i]);
}

// Conveyor Helpers (IR sensor detection)
bool isObjectDetected() {
  return digitalRead(PIN_IR) == IR_DETECTED_LEVEL;
}

void relayApply(bool on) {
  if (on && !conveyorRunning) {
    digitalWrite(PIN_RELAY, RELAY_ON);
    conveyorRunning = true;
  } else if (!on && conveyorRunning) {
    if (relayOffDelayMs > 0) delay(relayOffDelayMs);
    digitalWrite(PIN_RELAY, RELAY_OFF);
    conveyorRunning = false;
  }
}

// TCS3200 Helpers 
void tcsSetScale20() {
  digitalWrite(PIN_TCS_S0, HIGH);
  digitalWrite(PIN_TCS_S1, LOW);
}

//color measure
void tcsSetFilter(TcsColorFilter f) {
  switch (f) {
    case TCSF_RED:   digitalWrite(PIN_TCS_S2, LOW);  digitalWrite(PIN_TCS_S3, LOW);  break;
    case TCSF_BLUE:  digitalWrite(PIN_TCS_S2, LOW);  digitalWrite(PIN_TCS_S3, HIGH); break;
    case TCSF_CLEAR: digitalWrite(PIN_TCS_S2, HIGH); digitalWrite(PIN_TCS_S3, LOW);  break;
    case TCSF_GREEN: digitalWrite(PIN_TCS_S2, HIGH); digitalWrite(PIN_TCS_S3, HIGH); break;
  }
}

uint32_t tcsReadFreq(TcsColorFilter f) {
  tcsSetFilter(f);
  delay(5);

  uint32_t pw = pulseIn(PIN_TCS_OUT, HIGH, 50000UL);
  if (pw == 0) return 0;
  return 1000000UL / pw;
}

//decides the final color
String detectColorSimple(uint32_t r, uint32_t g, uint32_t b) {
  if (r == 0 || g == 0 || b == 0) return "UNKNOWN";
  if (r > g && r > b) return "RED";
  if (g > r && g > b) return "GREEN";
  if (b > r && b > g) return "BLUE";
  return "UNKNOWN";
}

//  GRAB + BIN POINTS
void goToGrabPoint() {
  moveSmoothTo(0, cfg.a0);
  moveSmoothTo(2, cfg.a2);
  moveSmoothTo(3, cfg.a3);
  moveSmoothTo(4, cfg.a4_open);
  moveSmoothTo(1, cfg.a1);
  moveSmoothTo(4, cfg.a4_close);
}

void goToRedBinPoint() {
  moveSmoothTo(1, 35);
  moveSmoothTo(3, 110);
  moveSmoothTo(0, 160);
  moveSmoothTo(3, 180);
  moveSmoothTo(1, 44);
  moveSmoothTo(2, 51);
  moveSmoothTo(4, 85);
}

void goToGreenBinPoint() {
  moveSmoothTo(1, 35);
  moveSmoothTo(3, 110);
  moveSmoothTo(0, 145);
  moveSmoothTo(3, 180);
  moveSmoothTo(1, 40);
  moveSmoothTo(2, 51);
  moveSmoothTo(4, 85);
}

void goToBlueBinPoint() {
  moveSmoothTo(1, 35);
  moveSmoothTo(3, 110);
  moveSmoothTo(0, 130);
  moveSmoothTo(3, 180);
  moveSmoothTo(1, 47);
  moveSmoothTo(2, 47);
  moveSmoothTo(4, 85);
}

// Sorting Sequences 
void performRedSequence() {
  relayApply(false); // stop conveyor
  goToGrabPoint();  // pick item
  delay(300);
  goToRedBinPoint(); // move to bin
  delay(300);
  moveAllHome(); // return to home
}

void performGreenSequence() {
  relayApply(false);
  goToGrabPoint();
  delay(300);
  goToGreenBinPoint();
  delay(300);
  moveAllHome();
}

void performBlueSequence() {
  relayApply(false);
  goToGrabPoint();
  delay(300);
  goToBlueBinPoint();
  delay(300);
  moveAllHome();
}

// Simple Black and White UI
String dashboardHtml() {
  String html =
    "<!doctype html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>Dashboard</title>"
    "<style>"
    "body{margin:0;font-family:Arial,Helvetica,sans-serif;background:#fff;color:#000;}"
    ".wrap{max-width:900px;margin:0 auto;padding:16px;}"
    ".top{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:10px;}"
    ".top h1{margin:0;font-size:18px;}"
    ".card{border:1px solid #000;border-radius:10px;padding:12px;margin-top:12px;}"
    ".row{display:flex;gap:10px;flex-wrap:wrap;align-items:center;justify-content:space-between;}"
    ".btns{display:flex;gap:8px;flex-wrap:wrap;}"
    "button{border:1px solid #000;background:#fff;color:#000;padding:10px 12px;border-radius:10px;font-weight:700;cursor:pointer;}"
    "button:active{background:#000;color:#fff;}"
    ".grid{display:grid;grid-template-columns:1fr;gap:10px;}"
    ".box{border:1px solid #000;border-radius:10px;padding:10px;}"
    ".k{font-size:12px;color:#333;}"
    ".v{font-size:26px;font-weight:800;margin-top:4px;}"
    ".line{display:flex;justify-content:space-between;gap:10px;border-top:1px solid #000;padding-top:8px;margin-top:8px;}"
    "a{color:#000;text-decoration:underline;font-weight:700;}"
    "</style></head><body>"
    "<div class='wrap'>"
      "<div class='top'>"
        "<div><h1>COLOR SORTING DASHBOARD</h1></div>"
        //"<div><a href='/control'>Servo Control</a></div>"
      "</div>"

      "<div class='card'>"
        "<div class='row'>"
          "<div class='btns'>"
            "<button onclick='startAuto()'>Start</button>"
            "<button onclick='pauseAuto()'>Pause</button>"
            "<button onclick='resetCounts()'>Reset</button>"
          "</div>"
          "<div><b>Auto:</b> <span id='autoText'>PAUSED</span></div>"
        "</div>"
        "<div class='line'><div><b>Conveyor</b></div><div id='conveyorText'>OFF</div></div>"
        "<div class='line'><div><b>Color</b></div><div id='lastColor'>-</div></div>"
      "</div>"

      "<div class='card'>"
        "<div class='grid'>"
          "<div class='box'><div class='k'>Red</div><div class='v' id='r'>0</div></div>"
          "<div class='box'><div class='k'>Green</div><div class='v' id='g'>0</div></div>"
          "<div class='box'><div class='k'>Blue</div><div class='v' id='b'>0</div></div>"
          "<div class='box'><div class='k'>Total</div><div class='v' id='t'>0</div></div>"
        "</div>"
      "</div>"

    "</div>"

    "<script>"
    "function startAuto(){fetch('/start').then(()=>refresh());}"
    "function pauseAuto(){fetch('/pause').then(()=>refresh());}"
    "function resetCounts(){fetch('/reset').then(()=>refresh());}"
    "function refresh(){"
    " fetch('/stats').then(r=>r.json()).then(s=>{"
    "  t.textContent=s.total; r.textContent=s.red; g.textContent=s.green; b.textContent=s.blue;"
    "  autoText.textContent=s.auto ? 'RUNNING' : 'PAUSED';"
    "  conveyorText.textContent=s.conveyor ? 'ON' : 'OFF';"
    "  lastColor.textContent=s.lastColor || '-';"
    " }).catch(()=>{});"
    "}"
    "setInterval(refresh, 500);"
    "refresh();"
    "</script></body></html>";
  return html;
}

String controlHtml() {
  String html =
    "<!doctype html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<title>Servo Control</title>"
    "<style>"
    "body{margin:0;font-family:Arial,Helvetica,sans-serif;background:#fff;color:#000;}"
    ".wrap{max-width:900px;margin:0 auto;padding:16px;}"
    ".top{display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:10px;}"
    ".top h1{margin:0;font-size:18px;}"
    ".card{border:1px solid #000;border-radius:10px;padding:12px;margin-top:12px;}"
    ".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;}"
    "@media (max-width:700px){.grid{grid-template-columns:1fr;}}"
    ".slider{border:1px solid #000;border-radius:10px;padding:10px;}"
    ".head{display:flex;align-items:center;justify-content:space-between;gap:10px;}"
    ".name{font-weight:800;font-size:13px;}"
    ".val{font-weight:800;font-size:13px;}"
    "input[type=range]{width:100%;margin-top:10px;}"
    "input[type=number]{width:100%;margin-top:10px;padding:10px;border:1px solid #000;border-radius:10px;}"
    ".btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:10px;}"
    "button{border:1px solid #000;background:#fff;color:#000;padding:10px 12px;border-radius:10px;font-weight:700;cursor:pointer;}"
    "button:active{background:#000;color:#fff;}"
    ".status{border:1px solid #000;border-radius:10px;padding:10px;margin-top:10px;}"
    "a{color:#000;text-decoration:underline;font-weight:700;}"
    "</style></head><body>"
    "<div class='wrap'>"
      "<div class='top'>"
        "<div><h1>Servo Control</h1></div>"
        "<div><a href='/'>Back to Dashboard</a></div>"
      "</div>"

      "<div class='card'>"
        "<div style='font-size:12px;color:#333;'>Calibration: /calib?min=140&max=620</div>"
        "<div class='grid'>";

  for (int i = 0; i < 5; i++) {
    html +=
      "<div class='slider'>"
        "<div class='head'>"
          "<div class='name'>Servo " + String(i) + " (P" + String(SERVO_CH[i]) + ")</div>"
          "<div class='val'><span id='v" + String(i) + "'>" + String(HOME_ANGLE[i]) + "</span> deg</div>"
        "</div>"
        "<input id='s" + String(i) + "' type='range' min='0' max='180' value='" + String(HOME_ANGLE[i]) + "' />"
      "</div>";
  }

  html +=
        "</div>"
        "<div class='btns'>"
          "<button onclick='homeAll()'>Home All</button>"
          "<button onclick='readColor()'>Read Color</button>"
          "<button onclick='readSense()'>Sense</button>"
        "</div>"
        "<div class='status'>Status: <span id='st'>-</span></div>"
      "</div>";

  // Relay OFF delay control card
  html +=
      "<div class='card'>"
        "<div style='font-weight:800;margin-bottom:8px;'>Relay OFF Delay</div>"
        "<div style='font-size:12px;color:#333;margin-bottom:10px;'>Delay applied only when relay switches ON to OFF (ms).</div>"
        "<div class='slider'>"
          "<div class='head'><div class='name'>OFF Delay (ms)</div>"
          "<div class='val'><span id='rdv'>0</span> ms</div></div>"
          "<input id='rd' type='range' min='0' max='2000' value='0' />"
          "<input id='rdn' type='number' min='0' max='2000' value='0' />"
        "</div>"
        "<div class='btns'>"
          "<button onclick='relayDelayLoad()'>Load</button>"
          "<button onclick='relayDelaySave()'>Save</button>"
        "</div>"
        "<div class='status'>Relay Delay: <span id='rdst'>-</span></div>"
      "</div>";

  // Grab Point Editor card
  html +=
      "<div class='card'>"
        "<div style='font-weight:800;margin-bottom:8px;'>Grab Point Editor</div>"
        "<div style='font-size:12px;color:#333;margin-bottom:10px;'>Edit angles, Save to EEPROM, Apply to move arm.</div>"
        "<div class='grid'>"

          "<div class='slider'><div class='head'><div class='name'>Servo 0</div>"
          "<div class='val'><span id='ga0v'>0</span> deg</div></div>"
          "<input id='ga0' type='range' min='0' max='180' value='0' /></div>"

          "<div class='slider'><div class='head'><div class='name'>Servo 2</div>"
          "<div class='val'><span id='ga2v'>0</span> deg</div></div>"
          "<input id='ga2' type='range' min='0' max='180' value='0' /></div>"

          "<div class='slider'><div class='head'><div class='name'>Servo 3</div>"
          "<div class='val'><span id='ga3v'>0</span> deg</div></div>"
          "<input id='ga3' type='range' min='0' max='180' value='0' /></div>"

          "<div class='slider'><div class='head'><div class='name'>Servo 4 (Open)</div>"
          "<div class='val'><span id='ga4ov'>0</span> deg</div></div>"
          "<input id='ga4o' type='range' min='0' max='180' value='0' /></div>"

          "<div class='slider'><div class='head'><div class='name'>Servo 1</div>"
          "<div class='val'><span id='ga1v'>0</span> deg</div></div>"
          "<input id='ga1' type='range' min='0' max='180' value='0' /></div>"

          "<div class='slider'><div class='head'><div class='name'>Servo 4 (Close)</div>"
          "<div class='val'><span id='ga4cv'>0</span> deg</div></div>"
          "<input id='ga4c' type='range' min='0' max='180' value='0' /></div>"

        "</div>"
        "<div class='btns'>"
          "<button onclick='grabLoad()'>Load</button>"
          "<button onclick='grabSave()'>Save</button>"
          "<button onclick='grabApply()'>Apply</button>"
        "</div>"
        "<div class='status'>Grab: <span id='gstatus'>-</span></div>"
      "</div>"

    "</div>"

    "<script>"
    "function send(i,a){fetch('/set?servo='+i+'&angle='+a).catch(()=>{});}"

    "function bind(i){"
    " const s=document.getElementById('s'+i);"
    " const v=document.getElementById('v'+i);"
    " let t=null;"
    " s.addEventListener('input', ()=>{"
    "   v.textContent=s.value;"
    "   clearTimeout(t);"
    "   t=setTimeout(()=>send(i,s.value), 80);"
    " });"
    "}"
    "for(let i=0;i<5;i++) bind(i);"

    "function homeAll(){"
    " const homes=[" + String(HOME_ANGLE[0]) + "," + String(HOME_ANGLE[1]) + "," + String(HOME_ANGLE[2]) + "," + String(HOME_ANGLE[3]) + "," + String(HOME_ANGLE[4]) + "];"
    " for(let i=0;i<5;i++){"
    "   document.getElementById('s'+i).value=homes[i];"
    "   document.getElementById('v'+i).textContent=homes[i];"
    "   send(i, homes[i]);"
    " }"
    "}"

    "function readColor(){fetch('/color').then(r=>r.text()).then(t=>st.textContent=t).catch(()=>{});}"
    "function readSense(){fetch('/sense').then(r=>r.text()).then(t=>st.textContent=t).catch(()=>{});}"

    "function setGrabUI(s){"
    " ga0.value=s.a0; ga0v.textContent=s.a0;"
    " ga2.value=s.a2; ga2v.textContent=s.a2;"
    " ga3.value=s.a3; ga3v.textContent=s.a3;"
    " ga4o.value=s.a4_open; ga4ov.textContent=s.a4_open;"
    " ga1.value=s.a1; ga1v.textContent=s.a1;"
    " ga4c.value=s.a4_close; ga4cv.textContent=s.a4_close;"
    "}"

    "function bindGrab(id,valId){"
    " const el=document.getElementById(id);"
    " const v=document.getElementById(valId);"
    " el.addEventListener('input', ()=>{ v.textContent=el.value; });"
    "}"
    "bindGrab('ga0','ga0v');"
    "bindGrab('ga2','ga2v');"
    "bindGrab('ga3','ga3v');"
    "bindGrab('ga4o','ga4ov');"
    "bindGrab('ga1','ga1v');"
    "bindGrab('ga4c','ga4cv');"

    "function grabLoad(){"
    " fetch('/grab/get').then(r=>r.json()).then(s=>{setGrabUI(s); gstatus.textContent='LOADED';}).catch(()=>{gstatus.textContent='ERR';});"
    "}"

    "function grabSave(){"
    " const a0=ga0.value, a2=ga2.value, a3=ga3.value, a4o=ga4o.value, a1=ga1.value, a4c=ga4c.value;"
    " fetch('/grab/save?a0='+a0+'&a2='+a2+'&a3='+a3+'&a4o='+a4o+'&a1='+a1+'&a4c='+a4c)"
    "  .then(r=>r.text()).then(t=>{gstatus.textContent=t;}).catch(()=>{gstatus.textContent='ERR';});"
    "}"

    "function grabApply(){"
    " fetch('/grab/apply').then(r=>r.text()).then(t=>{gstatus.textContent=t;}).catch(()=>{gstatus.textContent='ERR';});"
    "}"

    "function relayDelaySetUI(ms){"
    " ms = Number(ms)||0; if(ms<0) ms=0; if(ms>2000) ms=2000;"
    " rd.value=ms; rdn.value=ms; rdv.textContent=ms;"
    "}"
    "rd.addEventListener('input', ()=>{ relayDelaySetUI(rd.value); });"
    "rdn.addEventListener('input', ()=>{ relayDelaySetUI(rdn.value); });"

    "function relayDelayLoad(){"
    " fetch('/relay/get').then(r=>r.json()).then(s=>{relayDelaySetUI(s.offDelay); rdst.textContent='LOADED';}).catch(()=>{rdst.textContent='ERR';});"
    "}"
    "function relayDelaySave(){"
    " const ms=Number(rdn.value)||0;"
    " fetch('/relay/save?off='+ms).then(r=>r.text()).then(t=>{rdst.textContent=t;}).catch(()=>{rdst.textContent='ERR';});"
    "}"

    "relayDelayLoad();"
    "grabLoad();"
    "</script></body></html>";

  return html;
}

// Web Handlers
void handleDashboard() { server.send(200, "text/html", dashboardHtml()); }
void handleControl() { server.send(200, "text/html", controlHtml()); }

void handleSet() {
  if (!server.hasArg("servo") || !server.hasArg("angle")) {
    server.send(400, "text/plain", "Missing servo or angle");
    return;
  }
  int idx = server.arg("servo").toInt();
  int angle = server.arg("angle").toInt();
  if (idx < 0 || idx > 4) {
    server.send(400, "text/plain", "Servo index must be 0..4");
    return;
  }
  angle = constrain(angle, 0, 180);
  moveSmoothTo((uint8_t)idx, angle);
  server.send(200, "text/plain", "OK");
}

//Calibration endpoint(Changes PWM range used for ALL servos)
void handleCalib() {
  if (server.hasArg("min")) SERVO_MIN_TICK = server.arg("min").toInt();
  if (server.hasArg("max")) SERVO_MAX_TICK = server.arg("max").toInt();
  SERVO_MIN_TICK = constrain(SERVO_MIN_TICK, 0, 4095);
  SERVO_MAX_TICK = constrain(SERVO_MAX_TICK, 0, 4095);
  server.send(200, "text/plain",
              "CALIB OK min=" + String(SERVO_MIN_TICK) + " max=" + String(SERVO_MAX_TICK));
}

void handleSense() {
  bool obj = isObjectDetected();
  uint32_t r = tcsReadFreq(TCSF_RED);
  uint32_t g = tcsReadFreq(TCSF_GREEN);
  uint32_t b = tcsReadFreq(TCSF_BLUE);

  String msg = "IR=" + String(obj ? "DETECTED" : "CLEAR") +
               " R=" + String(r) +
               " G=" + String(g) +
               " B=" + String(b) +
               " Conveyor=" + String(conveyorRunning ? "ON" : "OFF") +
               " Auto=" + String(autoEnabled ? "RUNNING" : "PAUSED") +
               " RelayOffDelayMs=" + String(relayOffDelayMs);
  server.send(200, "text/plain", msg);
}

//decide color
void handleColor() {
  uint32_t r = tcsReadFreq(TCSF_RED);
  uint32_t g = tcsReadFreq(TCSF_GREEN);
  uint32_t b = tcsReadFreq(TCSF_BLUE);

  String c = detectColorSimple(r, g, b);
  server.send(200, "text/plain",
              "Color=" + c + " (R=" + String(r) + " G=" + String(g) + " B=" + String(b) + ")");
}

//Start , Pause , Reset
void handleStart() {
  autoEnabled = true;
  sortState = ST_RUNNING;
  server.send(200, "text/plain", "OK START");
}

void handlePause() {
  autoEnabled = false;
  relayApply(false);
  sortState = ST_RUNNING;
  server.send(200, "text/plain", "OK PAUSE");
}

void handleReset() {
  countTotal = 0;
  countRed = 0;
  countGreen = 0;
  countBlue = 0;
  countUnknown = 0;
  lastColor = "-";
  server.send(200, "text/plain", "OK RESET");
}

//dashboard live numbers
void handleStats() {
  String json = "{";
  json += "\"auto\":" + String(autoEnabled ? "true" : "false") + ",";
  json += "\"conveyor\":" + String(conveyorRunning ? "true" : "false") + ",";
  json += "\"total\":" + String(countTotal) + ",";
  json += "\"red\":" + String(countRed) + ",";
  json += "\"green\":" + String(countGreen) + ",";
  json += "\"blue\":" + String(countBlue) + ",";
  json += "\"lastColor\":\"" + lastColor + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

// Grab point endpoints
void handleGrabGet() {
  String json = "{";
  json += "\"a0\":" + String(cfg.a0) + ",";
  json += "\"a2\":" + String(cfg.a2) + ",";
  json += "\"a3\":" + String(cfg.a3) + ",";
  json += "\"a4_open\":" + String(cfg.a4_open) + ",";
  json += "\"a1\":" + String(cfg.a1) + ",";
  json += "\"a4_close\":" + String(cfg.a4_close);
  json += "}";
  server.send(200, "application/json", json);
}

void handleGrabSave() {
  if (!server.hasArg("a0") || !server.hasArg("a2") || !server.hasArg("a3") ||
      !server.hasArg("a4o") || !server.hasArg("a1") || !server.hasArg("a4c")) {
    server.send(400, "text/plain", "Missing args: a0,a2,a3,a4o,a1,a4c");
    return;
  }

  cfg.a0 = constrain(server.arg("a0").toInt(), 0, 180);
  cfg.a2 = constrain(server.arg("a2").toInt(), 0, 180);
  cfg.a3 = constrain(server.arg("a3").toInt(), 0, 180);
  cfg.a4_open = constrain(server.arg("a4o").toInt(), 0, 180);
  cfg.a1 = constrain(server.arg("a1").toInt(), 0, 180);
  cfg.a4_close = constrain(server.arg("a4c").toInt(), 0, 180);

  saveCfg();
  server.send(200, "text/plain", "OK SAVED");
}

void handleGrabApply() {
  relayApply(false);
  goToGrabPoint();
  server.send(200, "text/plain", "OK APPLIED");
}

// Relay delay endpoints
void handleRelayGet() {
  String json = "{";
  json += "\"offDelay\":" + String(relayOffDelayMs);
  json += "}";
  server.send(200, "application/json", json);
}

void handleRelaySave() {
  if (!server.hasArg("off")) {
    server.send(400, "text/plain", "Missing arg: off");
    return;
  }
  uint16_t ms = (uint16_t)constrain(server.arg("off").toInt(), 0, 2000);
  cfg.relayOffDelay = ms;
  saveCfg();
  server.send(200, "text/plain", "OK SAVED");
}

// Setup 
void setup() {
  Serial.begin(115200);
  delay(200);

  EEPROM.begin(EEPROM_SIZE);
  loadCfg();

  Wire.begin(D2, D1);
  pca.begin();
  pca.setPWMFreq(60);
  delay(10);

  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_OFF);
  conveyorRunning = false;
  delay(100);

  pinMode(PIN_IR, INPUT);

  pinMode(PIN_TCS_OUT, INPUT);
  pinMode(PIN_TCS_S0, OUTPUT);
  pinMode(PIN_TCS_S1, OUTPUT);
  pinMode(PIN_TCS_S2, OUTPUT);
  pinMode(PIN_TCS_S3, OUTPUT);

  tcsSetScale20();
  tcsSetFilter(TCSF_CLEAR);

//ESP becomes hotspot
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

//Move servos to HOME,
  for (int i = 0; i < 5; i++) {
    s[i].currentAngle = constrain(HOME_ANGLE[i], 0, 180);
    writeServoAngle((uint8_t)i, s[i].currentAngle);
  }


  server.on("/", handleDashboard);
  server.on("/control", handleControl);
  server.on("/set", handleSet);
  server.on("/calib", handleCalib);
  server.on("/sense", handleSense);
  server.on("/color", handleColor);

  server.on("/start", handleStart);
  server.on("/pause", handlePause);
  server.on("/reset", handleReset);
  server.on("/stats", handleStats);

  server.on("/grab/get", handleGrabGet);
  server.on("/grab/save", handleGrabSave);
  server.on("/grab/apply", handleGrabApply);

  server.on("/relay/get", handleRelayGet);
  server.on("/relay/save", handleRelaySave);

  server.begin();

  autoEnabled = false;
  sortState = ST_RUNNING;
  relayApply(false);
}

// Loop
void loop() {
  server.handleClient(); //Always handle web requests

  if (!autoEnabled) {
    relayApply(false);
    delay(10);
    return;
  }

  bool obj = isObjectDetected();

  if (sortState == ST_RUNNING) {
    relayApply(!obj);

    if (obj) {
      relayApply(false);
      delay(150);

      uint32_t r = tcsReadFreq(TCSF_RED);
      uint32_t g = tcsReadFreq(TCSF_GREEN);
      uint32_t b = tcsReadFreq(TCSF_BLUE);

      String c = detectColorSimple(r, g, b);

      lastColor = c;

      countTotal++;
      if (c == "RED") countRed++;
      else if (c == "GREEN") countGreen++;
      else if (c == "BLUE") countBlue++;
      else countUnknown++;

      if (c == "RED") sortState = ST_SORTING_RED;
      else if (c == "GREEN") sortState = ST_SORTING_GREEN;
      else if (c == "BLUE") sortState = ST_SORTING_BLUE;
      else sortState = ST_WAIT_CLEAR;
    }
  }

  if (sortState == ST_SORTING_RED) {
    performRedSequence();
    sortState = ST_WAIT_CLEAR;
  }

  if (sortState == ST_SORTING_GREEN) {
    performGreenSequence();
    sortState = ST_WAIT_CLEAR;
  }

  if (sortState == ST_SORTING_BLUE) {
    performBlueSequence();
    sortState = ST_WAIT_CLEAR;
  }

  if (sortState == ST_WAIT_CLEAR) {
    relayApply(false);
    if (!obj) sortState = ST_RUNNING;
  }

  delay(10);
}
