// globals.h - Updated with automatic calibration variables

#pragma once
#include <Arduino.h>

extern int MotorSpeed;
extern bool panLeft;
extern bool panRight;
extern bool tiltUp;
extern bool tiltDown;
extern int pan_angle;
extern int tilt_angle;
extern int visitCount;
extern int gpLed;

extern String WiFiAddr;
extern bool calibration_requested;
extern float pan_offset;
extern float tilt_offset;

// Add automatic calibration variables
extern bool auto_calibration_active;
extern unsigned long auto_cal_start_time;
extern int auto_cal_step;

// Function declarations
void calibrateIMU();
void startAutomaticCalibration();
void performAutomaticCalibration();

extern float imu_pan_position;
extern float imu_tilt_position;