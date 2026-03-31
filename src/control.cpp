#include "control.h"

namespace {
static float integ = 0.0f;
static float outCmd = 0.0f;

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float applyDeadband(float v, float db) {
  if (fabsf(v) < db) return 0.0f;
  return v;
}
}  // namespace

namespace Control {
void reset() {
  integ = 0.0f;
  outCmd = 0.0f;
}

Telemetry step(const Params& params, float angleForPidDeg, float gyroXDegPerSec, float dtSec, float maxSpeed, float slewRate) {
  Telemetry t;
  const float dt = clampf(dtSec, 0.001f, 0.02f);

  float err = -angleForPidDeg;
  err = applyDeadband(err, params.deadbandDeg);

  const float p = params.Kp * err;
  const float d = -params.Kd * gyroXDegPerSec;

  if (fabsf(err) <= params.iZoneDeg && params.Ki > 0.0f) {
    integ += (params.Ki * err) * dt;
    integ = clampf(integ, -params.iTermLimit, params.iTermLimit);
  } else {
    integ *= 0.995f;
  }

  const float outRaw = (p + integ + d) * params.gain;
  const float outLimited = clampf(outRaw, -maxSpeed, maxSpeed);

  const float maxDelta = slewRate * dt;
  float delta = outLimited - outCmd;
  delta = clampf(delta, -maxDelta, maxDelta);
  outCmd += delta;

  t.err = err;
  t.p = p;
  t.i = integ;
  t.d = d;
  t.outRaw = outRaw;
  t.outCmd = outCmd;
  return t;
}
}  // namespace Control
