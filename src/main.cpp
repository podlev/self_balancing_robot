#include <Arduino.h>

#include <GyverStepper2.h>

#include <WiFi.h>
#include <WebServer.h>

#include "config.h"
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
static float Kp = 45.0f;
static float Ki = 0.0f;
static float Kd = 2.2f;
static float CONTROL_GAIN = 2.0f;
static float DEADBAND_DEG = 0.10f;

static float ITERM_LIMIT = 500.0f;
static float I_ZONE_DEG = 10.0f;

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

static float integ = 0.0f;
static float outCmd = 0.0f;

static bool motorsEnabled = true;

// Status for web
static float lastErr = 0.0f;
static float lastP = 0.0f;
static float lastI = 0.0f;
static float lastD = 0.0f;
static float lastRaw = 0.0f;

// ------------------- Utils -------------------
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float applyDeadband(float v, float db) {
  if (fabsf(v) < db) return 0.0f;
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
    integ = 0.0f;
    outCmd = 0.0f;
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
  integ = 0.0f;
  outCmd = 0.0f;
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
  dt = clampf(dt, 0.001f, 0.02f);

  float err = -angleForPid;
  err = applyDeadband(err, DEADBAND_DEG);

  const float P = Kp * err;
  const float D = -Kd * gyroX_dps;

  if (fabsf(err) <= I_ZONE_DEG && Ki > 0.0f) {
    integ += (Ki * err) * dt;
    integ = clampf(integ, -ITERM_LIMIT, ITERM_LIMIT);
  } else {
    integ *= 0.995f;
  }

  const float I = integ;

  const float outRaw = (P + I + D) * CONTROL_GAIN;
  const float outLimited = clampf(outRaw, -MAX_SPEED, MAX_SPEED);

  const float maxDelta = SLEW_RATE * dt;
  float delta = outLimited - outCmd;
  delta = clampf(delta, -maxDelta, maxDelta);
  outCmd += delta;

  // Keep sign matching the previous working direction setup.
  const float motorSpeed = -outCmd;

  motorL.setSpeed(motorSpeed);
  motorR.setSpeed(motorSpeed);

  lastErr = err;
  lastP = P;
  lastI = I;
  lastD = D;
  lastRaw = outRaw;
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

  s += "\"Kp\":" + String(Kp, 3) + ",";
  s += "\"Ki\":" + String(Ki, 3) + ",";
  s += "\"Kd\":" + String(Kd, 3) + ",";
  s += "\"CONTROL_GAIN\":" + String(CONTROL_GAIN, 3) + ",";

  s += "\"MAX_SPEED\":" + String(MAX_SPEED, 1) + ",";
  s += "\"ACCEL\":" + String(ACCEL, 1) + ",";
  s += "\"SLEW_RATE\":" + String(SLEW_RATE, 1) + ",";
  s += "\"FALL_ANGLE_DEG\":" + String(FALL_ANGLE_DEG, 1) + ",";

  s += "\"DEADBAND_DEG\":" + String(DEADBAND_DEG, 3) + ",";
  s += "\"ITERM_LIMIT\":" + String(ITERM_LIMIT, 1) + ",";
  s += "\"I_ZONE_DEG\":" + String(I_ZONE_DEG, 2) + ",";

  s += "\"err\":" + String(lastErr, 3) + ",";
  s += "\"P\":" + String(lastP, 2) + ",";
  s += "\"I\":" + String(lastI, 2) + ",";
  s += "\"D\":" + String(lastD, 2) + ",";
  s += "\"outRaw\":" + String(lastRaw, 2) + ",";
  s += "\"outCmd\":" + String(outCmd, 2);

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

    setArg("kp", Kp);
    setArg("ki", Ki);
    setArg("kd", Kd);
    setArg("cg", CONTROL_GAIN);
    setArg("ms", MAX_SPEED);
    setArg("ac", ACCEL);
    setArg("sr", SLEW_RATE);
    setArg("fa", FALL_ANGLE_DEG);
    setArg("dd", DEADBAND_DEG);
    setArg("il", ITERM_LIMIT);
    setArg("iz", I_ZONE_DEG);

    Kp = clampf(Kp, 0.0f, 400.0f);
    Ki = clampf(Ki, 0.0f, 120.0f);
    Kd = clampf(Kd, 0.0f, 80.0f);
    CONTROL_GAIN = clampf(CONTROL_GAIN, 0.1f, 10.0f);

    MAX_SPEED = clampf(MAX_SPEED, 0.0f, 6000.0f);
    ACCEL = clampf(ACCEL, 0.0f, 60000.0f);
    SLEW_RATE = clampf(SLEW_RATE, 0.0f, 60000.0f);
    FALL_ANGLE_DEG = clampf(FALL_ANGLE_DEG, 5.0f, 80.0f);

    DEADBAND_DEG = clampf(DEADBAND_DEG, 0.0f, 2.0f);
    ITERM_LIMIT = clampf(ITERM_LIMIT, 0.0f, 5000.0f);
    I_ZONE_DEG = clampf(I_ZONE_DEG, 0.0f, 45.0f);

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
    integ = 0.0f;
    outCmd = 0.0f;
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
