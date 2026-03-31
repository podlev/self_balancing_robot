#pragma once

#include <Arduino.h>

namespace Imu {
bool begin();
void calibrateStartup(uint32_t ms = 3000);
void update(float dt);
void rezero();

float angleRawDeg();
float angleForPidDeg();
float gyroXDegPerSec();
float offsetDeg();
}  // namespace Imu
