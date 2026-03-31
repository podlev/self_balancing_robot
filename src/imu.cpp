#include "imu.h"

#include <Wire.h>
#include <MPU6050.h>

#include "config.h"

namespace {
static const float GYRO_SENS_250DPS = 131.0f;
static const float ALPHA = 0.98f;

MPU6050 mpu;
float angleAcc = 0.0f;
float angleFused = 0.0f;
float angleOffset = 0.0f;
float gyroX_dps = 0.0f;
float gyroBias = 0.0f;
bool started = false;
}

namespace Imu {
bool begin() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  mpu.initialize();
  started = mpu.testConnection();
  Serial.println(started ? "MPU6050 OK" : "MPU6050 FAIL");
  return started;
}

void calibrateStartup(uint32_t ms) {
  if (!started) return;

  Serial.print("Calibrating IMU for ");
  Serial.print(ms);
  Serial.println(" ms. Keep robot upright and still.");

  const uint32_t t0 = millis();
  double sumGyro = 0.0;
  double sumAngle = 0.0;
  uint32_t n = 0;

  while (millis() - t0 < ms) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    const float acc = atan2f((float)ay, (float)az) * 57.2957795f;
    sumAngle += acc;
    sumGyro += (float)gx;
    n++;
    delay(2);
  }

  if (n > 0) {
    gyroBias = (float)(sumGyro / (double)n);
    angleOffset = (float)(sumAngle / (double)n);
    angleFused = angleOffset;
    angleAcc = angleOffset;
  } else {
    gyroBias = 0.0f;
    angleOffset = 0.0f;
    angleFused = 0.0f;
    angleAcc = 0.0f;
  }

  Serial.print("Offset = ");
  Serial.println(angleOffset, 2);
}

void update(float dt) {
  if (!started) return;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  angleAcc = atan2f((float)ay, (float)az) * 57.2957795f;
  gyroX_dps = ((float)gx - gyroBias) / GYRO_SENS_250DPS;

  const float gyroPred = angleFused + gyroX_dps * dt;
  angleFused = ALPHA * gyroPred + (1.0f - ALPHA) * angleAcc;
}

void rezero() {
  angleOffset = angleFused;
}

float angleRawDeg() {
  return angleFused;
}

float angleForPidDeg() {
  return angleFused - angleOffset;
}

float gyroXDegPerSec() {
  return gyroX_dps;
}

float offsetDeg() {
  return angleOffset;
}
}  // namespace Imu
