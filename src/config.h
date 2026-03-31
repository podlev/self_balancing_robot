#pragma once

// Pins
static const int EN_L = 25;
static const int STEP_L = 26;
static const int DIR_L = 27;
static const int EN_R = 33;
static const int STEP_R = 14;
static const int DIR_R = 12;
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

// Drive config
static const bool INVERT_DIR_L = true;
static const bool INVERT_DIR_R = false;
static const int MICROSTEPS = 8;
static const int STEPS_REV = 200 * MICROSTEPS;

// Timing
static const uint16_t PID_DT_MS = 5;  // 200 Hz
