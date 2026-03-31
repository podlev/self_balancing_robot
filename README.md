# Self-Balancing Robot (ESP32)

Firmware for a 2-wheel self-balancing robot based on `ESP32 + MPU6050 + A4988`.

## Hardware

- ESP32 DevKit (`esp32dev`)
- MPU6050 (I2C)
- 2x stepper motor + 2x A4988

Default pins are defined in `src/config.h`.

## Build and Flash

This project uses PlatformIO (`platformio.ini`):

- `platform = espressif32`
- `board = esp32dev`
- `framework = arduino`

Typical workflow:

1. Build: `pio run`
2. Upload: `pio run -t upload`
3. Monitor: `pio device monitor -b 115200`

## Runtime Tuning UI

At boot, firmware starts a Wi-Fi AP:

- SSID: `BalBot`
- Password: `12345678`
- UI: `http://192.168.4.1/`

Available endpoints:

- `/status` - JSON telemetry
- `/set` - update control and motor limits
- `/motors?on=0|1` - enable/disable drivers
- `/rezero` - set current tilt as zero
- `/cleartrip` - clear safety trip flag

## Code Structure

- `src/main.cpp` - app orchestration, safety, web handlers, motor ticking
- `src/imu.cpp`, `src/imu.h` - MPU6050 init/calibration and complementary filter
- `src/control.cpp`, `src/control.h` - PID + anti-windup + slew-rate + telemetry
- `src/config.h` - pins and shared compile-time constants

## Safety Notes

- Keep robot physically supported during first tuning runs.
- Start with low `MAX_SPEED` and `ACCEL`, then increase gradually.
- If tilt exceeds `FALL_ANGLE_DEG`, motors are disabled and trip is latched.
