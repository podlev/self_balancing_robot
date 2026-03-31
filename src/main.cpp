#include <Arduino.h>

#include <GyverStepper2.h>

#include <WiFi.h>
#include <WebServer.h>

#include "config.h"
#include "control.h"
#include "imu.h"

// ------------------- WiFi AP -------------------
static const char* AP_SSID = "BalBot";
static const char* AP_PASS = "12345678";
WebServer server(80);

// ------------------- Motors -------------------
static float MAX_SPEED = 1200.0f;     // steps/s
static float ACCEL = 9000.0f;         // steps/s^2
static float SLEW_RATE = 12000.0f;    // steps/s^2
static uint32_t tPID = 0;

// ------------------- PID params -------------------
static Control::Params ctrl;

// ------------------- Safety -------------------
static float FALL_ANGLE_DEG = 30.0f;
static bool safetyTripped = false;

// ------------------- Objects -------------------
GStepper2<STEPPER2WIRE> motorL(STEPS_REV, STEP_L, DIR_L);
GStepper2<STEPPER2WIRE> motorR(STEPS_REV, STEP_R, DIR_R);

// ------------------- State -------------------
static float angleX = 0.0f;
static float angleForPid = 0.0f;

static float gyroX_dps = 0.0f;

static bool motorsEnabled = true;

// Status for web
static Control::Telemetry telem;

// ------------------- Utils -------------------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void applyMotorLimits() {
  motorL.setMaxSpeed((int)MAX_SPEED);
  motorR.setMaxSpeed((int)MAX_SPEED);
  motorL.setAcceleration((int)ACCEL);
  motorR.setAcceleration((int)ACCEL);
}

void setMotorsEnabled(bool en) {
  motorsEnabled = en;
  digitalWrite(EN_L, en ? LOW : HIGH);
  digitalWrite(EN_R, en ? LOW : HIGH);

  if (!en) {
    motorL.setSpeed(0);
    motorR.setSpeed(0);
    Control::reset();
  }
}

void initMotors() {
  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);

  motorL.reverse(INVERT_DIR_L);
  motorR.reverse(INVERT_DIR_R);

  applyMotorLimits();

  motorL.setSpeed(0);
  motorR.setSpeed(0);

  setMotorsEnabled(motorsEnabled);
}

// ------------------- Control -------------------
void updateIMU() {
  Imu::update(PID_DT_MS * 0.001f);
  angleX = Imu::angleRawDeg();
  gyroX_dps = Imu::gyroXDegPerSec();
  angleForPid = Imu::angleForPidDeg();
}

bool safetyOk() {
  return fabsf(angleForPid) <= FALL_ANGLE_DEG;
}

void stopMotorsHard() {
  motorL.setSpeed(0);
  motorR.setSpeed(0);
  Control::reset();
}

void computePIDAndApply() {
  static uint32_t lastUs = 0;
  const uint32_t nowUs = micros();
  if (lastUs == 0) {
    lastUs = nowUs;
    return;
  }

  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  telem = Control::step(ctrl, angleForPid, gyroX_dps, dt, MAX_SPEED, SLEW_RATE);

  // Keep sign matching the previous working direction setup.
  const float motorSpeed = -telem.outCmd;

  motorL.setSpeed(motorSpeed);
  motorR.setSpeed(motorSpeed);
}

void tickMotors() {
  motorL.tick();
  motorR.tick();
}

// ------------------- Web -------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>BalBot Starter</title>
<style>
body{font-family:Arial,sans-serif;max-width:860px;margin:16px}
.card{border:1px solid #ddd;border-radius:10px;padding:12px;margin-bottom:12px}
.row{display:flex;gap:10px;flex-wrap:wrap}
label{display:block;font-size:13px;margin-bottom:4px}
input{width:120px;padding:8px}
button{padding:8px 12px}
pre{background:#f5f5f5;padding:10px;border-radius:8px;overflow:auto}
</style></head><body>
<h2>BalBot Starter (1/8 microstep)</h2>
<div class="card">
<div class="row">
<div><label>Kp</label><input id="kp" type="number" step="0.1"></div>
<div><label>Ki</label><input id="ki" type="number" step="0.01"></div>
<div><label>Kd</label><input id="kd" type="number" step="0.1"></div>
<div><label>Gain</label><input id="cg" type="number" step="0.1"></div>
</div>
<div class="row">
<div><label>MAX_SPEED</label><input id="ms" type="number" step="10"></div>
<div><label>ACCEL</label><input id="ac" type="number" step="100"></div>
<div><label>SLEW_RATE</label><input id="sr" type="number" step="100"></div>
<div><label>FALL_ANGLE</label><input id="fa" type="number" step="1"></div>
</div>
<div class="row">
<div><label>DEADBAND_DEG</label><input id="dd" type="number" step="0.01"></div>
<div><label>ITERM_LIMIT</label><input id="il" type="number" step="10"></div>
<div><label>I_ZONE_DEG</label><input id="iz" type="number" step="0.5"></div>
</div>
<div class="row">
<button onclick="applyCfg()">Apply</button>
<button onclick="setMotors(1)">Motors ON</button>
<button onclick="setMotors(0)">Motors OFF</button>
<button onclick="rezero()">ReZero</button>
<button onclick="clearTrip()">Clear Trip</button>
<button onclick="reloadAll()">Refresh</button>
</div>
</div>
<div class="card">
<pre id="st">loading...</pre>
</div>
<script>
function setField(id, v){ if(document.activeElement.id!==id) document.getElementById(id).value=v; }
async function getStatus(){ const r=await fetch('/status'); return await r.json(); }
async function reloadAll(){
  const j=await getStatus();
  setField('kp',j.Kp); setField('ki',j.Ki); setField('kd',j.Kd); setField('cg',j.CONTROL_GAIN);
  setField('ms',j.MAX_SPEED); setField('ac',j.ACCEL); setField('sr',j.SLEW_RATE); setField('fa',j.FALL_ANGLE_DEG);
  setField('dd',j.DEADBAND_DEG); setField('il',j.ITERM_LIMIT); setField('iz',j.I_ZONE_DEG);
  st.textContent=JSON.stringify(j,null,2);
}
async function applyCfg(){
  const q=new URLSearchParams({
    kp:kp.value,ki:ki.value,kd:kd.value,cg:cg.value,
    ms:ms.value,ac:ac.value,sr:sr.value,fa:fa.value,
    dd:dd.value,il:il.value,iz:iz.value
  });
  await fetch('/set?'+q.toString());
  await reloadAll();
}
async function setMotors(on){ await fetch('/motors?on='+on); await reloadAll(); }
async function rezero(){ await fetch('/rezero'); await reloadAll(); }
async function clearTrip(){ await fetch('/cleartrip'); await reloadAll(); }
setInterval(reloadAll,500);
reloadAll();
</script></body></html>
)HTML";

String jsonStatus() {
  String s = "{";
  s += "\"microsteps\":" + String(MICROSTEPS) + ",";
  s += "\"motorsEnabled\":" + String(motorsEnabled ? "true" : "false") + ",";
  s += "\"safetyTripped\":" + String(safetyTripped ? "true" : "false") + ",";

  s += "\"angle\":" + String(angleForPid, 3) + ",";
  s += "\"gyroX_dps\":" + String(gyroX_dps, 3) + ",";
  s += "\"offset\":" + String(Imu::offsetDeg(), 3) + ",";

  s += "\"Kp\":" + String(ctrl.Kp, 3) + ",";
  s += "\"Ki\":" + String(ctrl.Ki, 3) + ",";
  s += "\"Kd\":" + String(ctrl.Kd, 3) + ",";
  s += "\"CONTROL_GAIN\":" + String(ctrl.gain, 3) + ",";

  s += "\"MAX_SPEED\":" + String(MAX_SPEED, 1) + ",";
  s += "\"ACCEL\":" + String(ACCEL, 1) + ",";
  s += "\"SLEW_RATE\":" + String(SLEW_RATE, 1) + ",";
  s += "\"FALL_ANGLE_DEG\":" + String(FALL_ANGLE_DEG, 1) + ",";

  s += "\"DEADBAND_DEG\":" + String(ctrl.deadbandDeg, 3) + ",";
  s += "\"ITERM_LIMIT\":" + String(ctrl.iTermLimit, 1) + ",";
  s += "\"I_ZONE_DEG\":" + String(ctrl.iZoneDeg, 2) + ",";

  s += "\"err\":" + String(telem.err, 3) + ",";
  s += "\"P\":" + String(telem.p, 2) + ",";
  s += "\"I\":" + String(telem.i, 2) + ",";
  s += "\"D\":" + String(telem.d, 2) + ",";
  s += "\"outRaw\":" + String(telem.outRaw, 2) + ",";
  s += "\"outCmd\":" + String(telem.outCmd, 2);

  s += "}";
  return s;
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/status", HTTP_GET, []() {
    server.send(200, "application/json", jsonStatus());
  });

  server.on("/set", HTTP_GET, []() {
    auto setArg = [&](const char* n, float& v) {
      if (server.hasArg(n)) v = server.arg(n).toFloat();
    };

    setArg("kp", ctrl.Kp);
    setArg("ki", ctrl.Ki);
    setArg("kd", ctrl.Kd);
    setArg("cg", ctrl.gain);
    setArg("ms", MAX_SPEED);
    setArg("ac", ACCEL);
    setArg("sr", SLEW_RATE);
    setArg("fa", FALL_ANGLE_DEG);
    setArg("dd", ctrl.deadbandDeg);
    setArg("il", ctrl.iTermLimit);
    setArg("iz", ctrl.iZoneDeg);

    ctrl.Kp = clampf(ctrl.Kp, 0.0f, 400.0f);
    ctrl.Ki = clampf(ctrl.Ki, 0.0f, 120.0f);
    ctrl.Kd = clampf(ctrl.Kd, 0.0f, 80.0f);
    ctrl.gain = clampf(ctrl.gain, 0.1f, 10.0f);

    MAX_SPEED = clampf(MAX_SPEED, 0.0f, 6000.0f);
    ACCEL = clampf(ACCEL, 0.0f, 60000.0f);
    SLEW_RATE = clampf(SLEW_RATE, 0.0f, 60000.0f);
    FALL_ANGLE_DEG = clampf(FALL_ANGLE_DEG, 5.0f, 80.0f);

    ctrl.deadbandDeg = clampf(ctrl.deadbandDeg, 0.0f, 2.0f);
    ctrl.iTermLimit = clampf(ctrl.iTermLimit, 0.0f, 5000.0f);
    ctrl.iZoneDeg = clampf(ctrl.iZoneDeg, 0.0f, 45.0f);

    applyMotorLimits();

    server.send(200, "text/plain", "OK");
  });

  server.on("/motors", HTTP_GET, []() {
    if (server.hasArg("on")) {
      const bool on = server.arg("on").toInt() != 0;
      setMotorsEnabled(on);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/rezero", HTTP_GET, []() {
    Imu::rezero();
    Control::reset();
    server.send(200, "text/plain", "OK");
  });

  server.on("/cleartrip", HTTP_GET, []() {
    safetyTripped = false;
    server.send(200, "text/plain", "OK");
  });

  server.begin();
}

// ------------------- Arduino -------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  const bool imuOk = Imu::begin();
  initMotors();

  setupWeb();

  if (!imuOk) {
    setMotorsEnabled(false);
    Serial.println("IMU failed. Fix wiring and reboot.");
    return;
  }

  Imu::calibrateStartup(3000);

  tPID = millis();
  Serial.println("RUN. Open http://192.168.4.1/");
}

void loop() {
  server.handleClient();
  tickMotors();

  if (millis() - tPID >= PID_DT_MS) {
    tPID += PID_DT_MS;

    updateIMU();

    if (!safetyOk()) {
      stopMotorsHard();
      setMotorsEnabled(false);
      safetyTripped = true;
      return;
    }

    if (!motorsEnabled || safetyTripped) {
      stopMotorsHard();
      return;
    }

    computePIDAndApply();
  }
}
