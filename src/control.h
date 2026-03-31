#pragma once

#include <Arduino.h>

namespace Control {
struct Params {
  float Kp = 45.0f;
  float Ki = 0.0f;
  float Kd = 2.2f;
  float gain = 2.0f;
  float deadbandDeg = 0.10f;
  float iTermLimit = 500.0f;
  float iZoneDeg = 10.0f;
};

struct Telemetry {
  float err = 0.0f;
  float p = 0.0f;
  float i = 0.0f;
  float d = 0.0f;
  float outRaw = 0.0f;
  float outCmd = 0.0f;
};

void reset();
Telemetry step(const Params& params, float angleForPidDeg, float gyroXDegPerSec, float dtSec, float maxSpeed, float slewRate);
}  // namespace Control
