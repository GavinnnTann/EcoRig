#include "esp_camera.h"
#include <WiFi.h>

#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include "AS5600.h"
#include <Adafruit_SSD1306.h>
#include <DFRobot_BMI160.h>
#include <Preferences.h>

#include "Images.h"
#include "camera_config.h"

AS5600 as5600_pan;   // Pan sensor
AS5600 as5600_tilt;  // Tilt sensor

#define CAMERA_MODEL_XIAO_ESP32S3  // Has PSRAM
#include "globals.h"

// 2-state Kalman filter: [angle, gyro_bias]
struct KalmanFilter {
  float angle   = 0.0f;
  float bias    = 0.0f;
  float P[2][2] = {{0, 0}, {0, 0}};
  const float Q_angle   = 0.001f;
  const float Q_bias    = 0.003f;
  const float R_measure = 0.03f;

  float update(float meas_angle, float gyro_rate_dps, float dt) {
    // Predict
    float rate = gyro_rate_dps - bias;
    angle += rate * dt;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;
    // Update
    float S  = P[0][0] + R_measure;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float innov = meas_angle - angle;
    angle += K0 * innov;
    bias  += K1 * innov;
    float P00 = P[0][0], P01 = P[0][1];
    P[0][0] -= K0 * P00;  P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;  P[1][1] -= K1 * P01;
    return angle;
  }
};

//Motor Pins
#define IN1 1  //motor1 (TILT)
#define IN2 2
#define IN3 3  //motor2 (PAN)
#define IN4 4

#define EN1 8
#define EN2 9

// OLED and Button Definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DFRobot_BMI160 bmi160;

const int8_t i2c_dispAddr = 0x3C;
const int8_t i2c_IMUAddr = 0x69;

float imu_pan_position = 0.0f;   // Current pan position in degrees
float imu_tilt_position = 0.0f;  // Current tilt position in degrees

bool calibration_requested = false;
float pan_offset = 0.0f;   // Calibration offset for pan
float tilt_offset = 0.0f;  // Calibration offset for tilt

// Physical limits for your system
const float PAN_PHYSICAL_LIMIT_MIN = -180.0f;
const float PAN_PHYSICAL_LIMIT_MAX = 180.0f;
const float TILT_PHYSICAL_LIMIT_MIN = -30.0f;
const float TILT_PHYSICAL_LIMIT_MAX = 30.0f;

float PAN_SOFTWARE_LIMIT_MIN = -90.0f;
float PAN_SOFTWARE_LIMIT_MAX = 90.0f;
float TILT_SOFTWARE_LIMIT_MIN = -25.0f;
float TILT_SOFTWARE_LIMIT_MAX = 25.0f;

const float SAFETY_MARGIN = 2.0f;

// Calibration-discovered limits (updated after each auto-calibration run)
float discovered_pan_min  = -90.0f;
float discovered_pan_max  =  90.0f;
float discovered_tilt_min = -25.0f;
float discovered_tilt_max =  25.0f;

// Angular velocity exposed by getGyro() for stall detection (deg/s, bias-corrected)
float imu_pan_rate  = 0.0f;
float imu_tilt_rate = 0.0f;
unsigned long stall_start_time = 0;

const float         STALL_RATE_THRESHOLD = 3.0f;   // deg/s below this = stall
const unsigned long STALL_CONFIRM_MS     = 200;     // rate must stay low this long
const unsigned long MIN_MOTION_MS        = 400;     // wait before checking stall
const unsigned long MAX_SWEEP_MS         = 4000;    // timeout per sweep
const unsigned long BACKOFF_MS           = 350;     // reverse duration after stall
const unsigned long CONFIRM_SWEEP_MS     = 300;     // re-advance to confirm limit

Preferences prefs;

bool display_needs_update = false;

// Automatic calibration variables
bool auto_calibration_active = false;
unsigned long auto_cal_start_time = 0;
int auto_cal_step = 0;
const int AUTO_CAL_TOTAL_STEPS = 6;

// Automatic calibration sequence timing (in milliseconds)
const unsigned long AUTO_CAL_SHORT_MOVE = 500;   // 0.5 seconds
const unsigned long AUTO_CAL_LONG_MOVE = 1000;   // 1.0 second
const unsigned long AUTO_CAL_PAUSE_DURATION = 300;  // 0.3 second pause between movements

enum CalibrationState {
  CAL_IDLE,
  // Pan axis: find min → back off → confirm → find max → back off → confirm → center
  CAL_FIND_PAN_MIN,  CAL_BACKOFF_PAN_MIN,  CAL_CONFIRM_PAN_MIN,
  CAL_FIND_PAN_MAX,  CAL_BACKOFF_PAN_MAX,  CAL_CONFIRM_PAN_MAX,
  CAL_CENTER_PAN,
  // Tilt axis: find max → back off → confirm → find min → back off → confirm → center
  CAL_FIND_TILT_MAX, CAL_BACKOFF_TILT_MAX, CAL_CONFIRM_TILT_MAX,
  CAL_FIND_TILT_MIN, CAL_BACKOFF_TILT_MIN, CAL_CONFIRM_TILT_MIN,
  CAL_CENTER_TILT,
  CAL_COMPLETE
};

CalibrationState cal_state = CAL_IDLE;
unsigned long cal_state_entry_time = 0;


int MotorSpeed = 200;
bool panLeft = false;
bool panRight = false;
bool tiltUp = false;
bool tiltDown = false;
int pan_angle = 0;
int tilt_angle = 0;
int checkFeed = 0;

// Global variables (add at the top, near pan_angle)
float motor_pan_total_angle = 0.0;   // Accumulated motor shaft angle in degrees
int prev_pan_raw = -1;               // Last raw angle read (0–4095)
float motor_tilt_total_angle = 0.0;  // Accumulated motor shaft angle in degrees
int prev_tilt_raw = -1;              // Last raw angle from AS5600


// Gear ratios
const float PAN_GEAR_RATIO = 160.0;   // Pan encoder needs to be divided by 160
const float TILT_GEAR_RATIO = 171.5;  // Tilt encoder needs to be divided by 171.5
int visitCount = 0;
int gpLed = 9;
String WiFiAddr = "";

esp_timer_handle_t timer;

// Wi-Fi Credentials
const char* ap_ssid = "EcoRig";
const char* ap_password = "admin0000";

// Function Declarations
void Update_display();
void startCameraServer();
void setupLedFlash(int gpLed);

void performAutomaticCalibration() {
  if (!auto_calibration_active) return;

  unsigned long elapsed = millis() - cal_state_entry_time;
  CalibrationState next_state = cal_state;
  panLeft = panRight = tiltUp = tiltDown = false;

  // Returns true once angular rate has stayed below threshold for STALL_CONFIRM_MS,
  // but only after MIN_MOTION_MS has elapsed so the motor has time to start moving.
  auto stallDetected = [&](float rate_dps) -> bool {
    if (elapsed < MIN_MOTION_MS) return false;
    if (fabsf(rate_dps) < STALL_RATE_THRESHOLD) {
      if (stall_start_time == 0) stall_start_time = millis();
      return (millis() - stall_start_time >= STALL_CONFIRM_MS);
    }
    stall_start_time = 0;
    return false;
  };

  switch (cal_state) {

    // ── Pan: find min (sweep left until stall or timeout) ──────────────────
    case CAL_FIND_PAN_MIN:
      auto_cal_step = 0;
      panLeft = true;
      if (stallDetected(imu_pan_rate) || elapsed >= MAX_SWEEP_MS) {
        discovered_pan_min = imu_pan_position;
        Serial.printf("Pan min stall: %.1f°\n", discovered_pan_min);
        next_state = CAL_BACKOFF_PAN_MIN;
      }
      break;

    case CAL_BACKOFF_PAN_MIN:         // back off from limit
      auto_cal_step = 0;
      panRight = true;
      if (elapsed >= BACKOFF_MS) next_state = CAL_CONFIRM_PAN_MIN;
      break;

    case CAL_CONFIRM_PAN_MIN:         // re-advance to confirm exact limit
      auto_cal_step = 0;
      panLeft = true;
      if (elapsed >= CONFIRM_SWEEP_MS) {
        discovered_pan_min = imu_pan_position;
        Serial.printf("Pan min confirmed: %.1f°\n", discovered_pan_min);
        next_state = CAL_FIND_PAN_MAX;
      }
      break;

    // ── Pan: find max (sweep right until stall or timeout) ─────────────────
    case CAL_FIND_PAN_MAX:
      auto_cal_step = 1;
      panRight = true;
      if (stallDetected(imu_pan_rate) || elapsed >= MAX_SWEEP_MS) {
        discovered_pan_max = imu_pan_position;
        Serial.printf("Pan max stall: %.1f°\n", discovered_pan_max);
        next_state = CAL_BACKOFF_PAN_MAX;
      }
      break;

    case CAL_BACKOFF_PAN_MAX:
      auto_cal_step = 1;
      panLeft = true;
      if (elapsed >= BACKOFF_MS) next_state = CAL_CONFIRM_PAN_MAX;
      break;

    case CAL_CONFIRM_PAN_MAX:
      auto_cal_step = 1;
      panRight = true;
      if (elapsed >= CONFIRM_SWEEP_MS) {
        discovered_pan_max = imu_pan_position;
        Serial.printf("Pan max confirmed: %.1f°\n", discovered_pan_max);
        next_state = CAL_CENTER_PAN;
      }
      break;

    // ── Pan: move to midpoint of discovered range ───────────────────────────
    case CAL_CENTER_PAN: {
      auto_cal_step = 2;
      float pan_center = (discovered_pan_min + discovered_pan_max) / 2.0f;
      float pan_err    = imu_pan_position - pan_center;
      if (fabsf(pan_err) < 3.0f || elapsed >= 3000) {
        next_state = CAL_FIND_TILT_MAX;
      } else if (pan_err > 0) {
        panLeft = true;
      } else {
        panRight = true;
      }
      break;
    }

    // ── Tilt: find max (sweep up until stall or timeout) ───────────────────
    case CAL_FIND_TILT_MAX:
      auto_cal_step = 3;
      tiltUp = true;
      if (stallDetected(imu_tilt_rate) || elapsed >= MAX_SWEEP_MS) {
        discovered_tilt_max = imu_tilt_position;
        Serial.printf("Tilt max stall: %.1f°\n", discovered_tilt_max);
        next_state = CAL_BACKOFF_TILT_MAX;
      }
      break;

    case CAL_BACKOFF_TILT_MAX:
      auto_cal_step = 3;
      tiltDown = true;
      if (elapsed >= BACKOFF_MS) next_state = CAL_CONFIRM_TILT_MAX;
      break;

    case CAL_CONFIRM_TILT_MAX:
      auto_cal_step = 3;
      tiltUp = true;
      if (elapsed >= CONFIRM_SWEEP_MS) {
        discovered_tilt_max = imu_tilt_position;
        Serial.printf("Tilt max confirmed: %.1f°\n", discovered_tilt_max);
        next_state = CAL_FIND_TILT_MIN;
      }
      break;

    // ── Tilt: find min (sweep down until stall or timeout) ─────────────────
    case CAL_FIND_TILT_MIN:
      auto_cal_step = 4;
      tiltDown = true;
      if (stallDetected(imu_tilt_rate) || elapsed >= MAX_SWEEP_MS) {
        discovered_tilt_min = imu_tilt_position;
        Serial.printf("Tilt min stall: %.1f°\n", discovered_tilt_min);
        next_state = CAL_BACKOFF_TILT_MIN;
      }
      break;

    case CAL_BACKOFF_TILT_MIN:
      auto_cal_step = 4;
      tiltUp = true;
      if (elapsed >= BACKOFF_MS) next_state = CAL_CONFIRM_TILT_MIN;
      break;

    case CAL_CONFIRM_TILT_MIN:
      auto_cal_step = 4;
      tiltDown = true;
      if (elapsed >= CONFIRM_SWEEP_MS) {
        discovered_tilt_min = imu_tilt_position;
        Serial.printf("Tilt min confirmed: %.1f°\n", discovered_tilt_min);
        next_state = CAL_CENTER_TILT;
      }
      break;

    // ── Tilt: move to midpoint of discovered range ──────────────────────────
    case CAL_CENTER_TILT: {
      auto_cal_step = 5;
      float tilt_center = (discovered_tilt_min + discovered_tilt_max) / 2.0f;
      float tilt_err    = imu_tilt_position - tilt_center;
      if (fabsf(tilt_err) < 2.0f || elapsed >= 3000) {
        next_state = CAL_COMPLETE;
      } else if (tilt_err > 0) {
        tiltDown = true;
      } else {
        tiltUp = true;
      }
      break;
    }

    // ── Done: update live limits and persist to NVS ─────────────────────────
    case CAL_COMPLETE:
      PAN_SOFTWARE_LIMIT_MIN  = discovered_pan_min  + SAFETY_MARGIN;
      PAN_SOFTWARE_LIMIT_MAX  = discovered_pan_max  - SAFETY_MARGIN;
      TILT_SOFTWARE_LIMIT_MIN = discovered_tilt_min + SAFETY_MARGIN;
      TILT_SOFTWARE_LIMIT_MAX = discovered_tilt_max - SAFETY_MARGIN;

      prefs.begin("ecorig", false);
      prefs.putFloat("pan_min",  PAN_SOFTWARE_LIMIT_MIN);
      prefs.putFloat("pan_max",  PAN_SOFTWARE_LIMIT_MAX);
      prefs.putFloat("tilt_min", TILT_SOFTWARE_LIMIT_MIN);
      prefs.putFloat("tilt_max", TILT_SOFTWARE_LIMIT_MAX);
      prefs.end();

      Serial.printf("Limits saved to NVS: Pan[%.1f, %.1f] Tilt[%.1f, %.1f]\n",
                    PAN_SOFTWARE_LIMIT_MIN, PAN_SOFTWARE_LIMIT_MAX,
                    TILT_SOFTWARE_LIMIT_MIN, TILT_SOFTWARE_LIMIT_MAX);

      auto_calibration_active = false;
      cal_state  = CAL_IDLE;
      auto_cal_step = 0;
      calibrateIMU();
      display_needs_update = true;
      return;

    default:
      break;
  }

  if (next_state != cal_state) {
    Serial.printf("Cal FSM: state %d → %d\n", cal_state, next_state);
    cal_state = next_state;
    cal_state_entry_time = millis();
    stall_start_time = 0;   // reset stall tracker on every state transition
  }
  display_needs_update = true;
}

void startAutomaticCalibration() {
  Serial.println("Starting calibration FSM (stall-detect mode)...");
  auto_calibration_active = true;
  auto_cal_start_time = millis();
  cal_state = CAL_FIND_PAN_MIN;
  cal_state_entry_time = millis();
  stall_start_time = 0;
  auto_cal_step = 0;
  panLeft = panRight = tiltUp = tiltDown = false;
  display_needs_update = true;
}

void stopAutomaticCalibration() {
  Serial.println("EMERGENCY STOP: Calibration FSM halted");
  auto_calibration_active = false;
  cal_state = CAL_IDLE;
  stall_start_time = 0;
  auto_cal_step = 0;
  panLeft = panRight = tiltUp = tiltDown = false;
  display_needs_update = true;
}

// Enhanced TCA9545A function with better isolation
bool TCA9545A(uint8_t bus) {
  if (bus > 3) return false;

  // Disable all channels first
  Wire.beginTransmission(0x70);
  Wire.write(0x00);  // Disable all channels
  Wire.endTransmission();
  delay(5);

  // Enable selected channel
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    delay(10);  // Longer delay for switching
    return true;
  } else {
    return false;
  }
}

bool isPanMovementAllowed(bool movingLeft) {
  if (movingLeft) {
    // Moving left (negative direction)
    return (imu_pan_position > (PAN_SOFTWARE_LIMIT_MIN + SAFETY_MARGIN));
  } else {
    // Moving right (positive direction)
    return (imu_pan_position < (PAN_SOFTWARE_LIMIT_MAX - SAFETY_MARGIN));
  }
}

bool isTiltMovementAllowed(bool movingUp) {
  if (movingUp) {
    // Moving up (positive direction)
    return (imu_tilt_position < (TILT_SOFTWARE_LIMIT_MAX - SAFETY_MARGIN));
  } else {
    // Moving down (negative direction)
    return (imu_tilt_position > (TILT_SOFTWARE_LIMIT_MIN + SAFETY_MARGIN));
  }
}

bool isWithinPhysicalLimits() {
  return (imu_pan_position > PAN_PHYSICAL_LIMIT_MIN && imu_pan_position < PAN_PHYSICAL_LIMIT_MAX && imu_tilt_position > TILT_PHYSICAL_LIMIT_MIN && imu_tilt_position < TILT_PHYSICAL_LIMIT_MAX);
}
void updateServoPositions(void* arg) {
  const char* dir;
  bool motorActive = false;
  static const char* last_dir = "Stopped";

  // Emergency physical limit check (should never trigger if software limits work)
  if (!isWithinPhysicalLimits()) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
    panLeft = panRight = tiltUp = tiltDown = false;
    return;
  }

  // Pan Left - check if movement is allowed within software limits
  if (panLeft && isPanMovementAllowed(true)) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    dir = "Left";
    analogWrite(EN2, MotorSpeed);
    motorActive = true;
  }
  // Pan Right - check if movement is allowed within software limits
  else if (panRight && isPanMovementAllowed(false)) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    dir = "Right";
    analogWrite(EN2, MotorSpeed);
    motorActive = true;
  }
  // Tilt Up - check if movement is allowed within software limits
  else if (tiltUp && isTiltMovementAllowed(true)) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    dir = "Up";
    analogWrite(EN1, MotorSpeed);
    motorActive = true;
  }
  // Tilt Down - check if movement is allowed within software limits
  else if (tiltDown && isTiltMovementAllowed(false)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    dir = "Down";
    analogWrite(EN1, MotorSpeed);
    motorActive = true;
  }
  // Handle software limit cases
  else if (panLeft && !isPanMovementAllowed(true)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
    dir = "Pan Left Limit (-90°)";
    panLeft = false;  // Stop the command
  } else if (panRight && !isPanMovementAllowed(false)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
    dir = "Pan Right Limit (+90°)";
    panRight = false;  // Stop the command
  } else if (tiltUp && !isTiltMovementAllowed(true)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
    dir = "Tilt Up Limit (+25°)";
    tiltUp = false;  // Stop the command
  } else if (tiltDown && !isTiltMovementAllowed(false)) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
    dir = "Tilt Down Limit (-25°)";
    tiltDown = false;  // Stop the command
  }
  // Normal stop condition
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    dir = "Stopped";
    analogWrite(EN1, 0);
    analogWrite(EN2, 0);
  }

  // Only flag display update if direction changed
  if (strcmp(dir, last_dir) != 0) {
    display_needs_update = true;
    last_dir = dir;
    Serial.printf("Motor status changed to: %s\n", dir);
  }

  // Debug output for limit checking (every 2 seconds to reduce spam)
  static unsigned long lastLimitCheck = 0;
  if (millis() - lastLimitCheck > 2000) {
    if ((panLeft || panRight || tiltUp || tiltDown) && !motorActive) {
      Serial.printf("Movement blocked at software limits:\n");
      Serial.printf("Current: Pan %.1f°, Tilt %.1f°\n", imu_pan_position, imu_tilt_position);
      Serial.printf("Software limits: Pan ±90°, Tilt ±25°\n");
      Serial.printf("Physical limits: Pan ±180°, Tilt ±30°\n");
    }
    lastLimitCheck = millis();
  }
}
// Check if system is at any software limit
bool isAtSoftwareLimit() {
  return (imu_pan_position <= (PAN_SOFTWARE_LIMIT_MIN + SAFETY_MARGIN) || imu_pan_position >= (PAN_SOFTWARE_LIMIT_MAX - SAFETY_MARGIN) || imu_tilt_position <= (TILT_SOFTWARE_LIMIT_MIN + SAFETY_MARGIN) || imu_tilt_position >= (TILT_SOFTWARE_LIMIT_MAX - SAFETY_MARGIN));
}

// Check if system is at any physical limit (emergency condition)
bool isAtPhysicalLimit() {
  return (imu_pan_position <= (PAN_PHYSICAL_LIMIT_MIN + SAFETY_MARGIN) || imu_pan_position >= (PAN_PHYSICAL_LIMIT_MAX - SAFETY_MARGIN) || imu_tilt_position <= (TILT_PHYSICAL_LIMIT_MIN + SAFETY_MARGIN) || imu_tilt_position >= (TILT_PHYSICAL_LIMIT_MAX - SAFETY_MARGIN));
}

// Function to get current limit status for web interface
String getLimitStatus() {
  String status = "";

  // Check software limits first
  if (imu_pan_position <= (PAN_SOFTWARE_LIMIT_MIN + SAFETY_MARGIN)) {
    status += "Pan Left Limit (-90°) | ";
  }
  if (imu_pan_position >= (PAN_SOFTWARE_LIMIT_MAX - SAFETY_MARGIN)) {
    status += "Pan Right Limit (+90°) | ";
  }
  if (imu_tilt_position <= (TILT_SOFTWARE_LIMIT_MIN + SAFETY_MARGIN)) {
    status += "Tilt Down Limit (-25°) | ";
  }
  if (imu_tilt_position >= (TILT_SOFTWARE_LIMIT_MAX - SAFETY_MARGIN)) {
    status += "Tilt Up Limit (+25°) | ";
  }

  // Check for physical limit emergency
  if (isAtPhysicalLimit()) {
    status += "PHYSICAL LIMIT REACHED! | ";
  }

  if (status.length() > 0) {
    status = status.substring(0, status.length() - 3);  // Remove last " | "
  } else {
    status = "Within Software Limits";
  }

  return status;
}

// Optional: Function to get available movement range
String getMovementRange() {
  float panRange = PAN_SOFTWARE_LIMIT_MAX - imu_pan_position;
  float panRangeNeg = imu_pan_position - PAN_SOFTWARE_LIMIT_MIN;
  float tiltRange = TILT_SOFTWARE_LIMIT_MAX - imu_tilt_position;
  float tiltRangeNeg = imu_tilt_position - TILT_SOFTWARE_LIMIT_MIN;

  char rangeStr[100];
  snprintf(rangeStr, sizeof(rangeStr),
           "Pan: %.1f°L/%.1f°R | Tilt: %.1f°D/%.1f°U remaining",
           panRangeNeg, panRange, tiltRangeNeg, tiltRange);

  return String(rangeStr);
}

void Update_display() {
  // Try multiple times if bus selection fails
  for (int attempt = 0; attempt < 3; attempt++) {
    if (TCA9545A(3)) {
      break;  // Success
    }
    delay(5);  // Wait before retry
    if (attempt == 2) {
      Serial.println("Failed to select display bus after 3 attempts");
      return;
    }
  }

  int numClients = WiFi.softAPgetStationNum();
  const char* motor_status;

  // Determine current motor status automatically
  if (auto_calibration_active) {
    unsigned long elapsed = millis() - auto_cal_start_time;
    
    // Show specific calibration step
    if (auto_cal_step < AUTO_CAL_TOTAL_STEPS) {
      switch (auto_cal_step) {
        case 0: motor_status = "Cal: Pan Left"; break;
        case 1: motor_status = "Cal: Pan Right"; break;
        case 2: motor_status = "Cal: Pan Center"; break;
        case 3: motor_status = "Cal: Tilt Up"; break;
        case 4: motor_status = "Cal: Tilt Down"; break;
        case 5: motor_status = "Cal: Tilt Center"; break;
        default: motor_status = "Auto Calibrating"; break;
      }
    } else {
      motor_status = "Auto Calibrating";
    }
  } else if (panLeft) {
    motor_status = "Left";
  } else if (panRight) {
    motor_status = "Right";  
  } else if (tiltUp) {
    motor_status = "Up";
  } else if (tiltDown) {
    motor_status = "Down";
  } else {
    motor_status = "Stopped";
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (numClients >= 1 && visitCount <= 1) {
    display.clearDisplay();
    display.drawBitmap(0, 0, epd_bitmap_qrCode, 128, 64, WHITE);
    display.setCursor(70, 30);
    display.print("Scan to");
    display.setCursor(66, 40);
    display.print("Open Link");
    display.display();
  } else if (numClients >= 1 && visitCount >= 1) {
    display.clearDisplay();
    display.println();
    
    // Show calibration status if active
    if (auto_calibration_active) {
      unsigned long elapsed = millis() - auto_cal_start_time;
      
      display.print("AUTO CALIBRATING");
      display.println();
      display.print("Step: ");
      display.print(auto_cal_step + 1);
      display.print("/");
      display.print(AUTO_CAL_TOTAL_STEPS);
      display.println();
      display.print("Time: ");
      display.print(elapsed / 1000);
      display.print(".");
      display.print((elapsed % 1000) / 100);
      display.print("s");
      display.println();
      
      // Show timeout countdown
      if (elapsed < TOTAL_CAL_TIME) {
        unsigned long remaining = (TOTAL_CAL_TIME - elapsed) / 1000;
        display.print("Remaining: ");
        display.print(remaining);
        display.print("s");
      }
    }
    
    display.print("Motor Status: ");
    display.println();
    display.print(motor_status);
    display.println();
    display.println();
    
    if (checkFeed != 0 && !auto_calibration_active) {
      display.print("Pan Position: ");
      display.print(pan_angle);
      display.println();
      display.print("Tilt Position: ");
      display.print(tilt_angle);
      display.println();
      display.println();
    } else if (!auto_calibration_active) {
      display.print("No feedback detected");
    }
    
    if (!auto_calibration_active) {
      display.print("Motor Speed: ");
      display.print(MotorSpeed);
    }
    display.display();
  } else {
    display.clearDisplay();
    display.print("Connect via WiFi");
    display.println();
    display.print("SSID: ");
    display.print(ap_ssid);
    display.println();
    display.print("Password: ");
    display.print(ap_password);
    visitCount = 0;
    display.display();
  }
  
  // Small delay after display update
  delay(5);
}


void getGyro(void) {
  if (!TCA9545A(2)) {
    Serial.println("Failed to select IMU bus");
    return;
  }

  // Persistent state
  static bool initialized = false;
  static float pitch = 0.0f;  // radians
  static float yaw = 0.0f;    // radians
  static uint32_t last_us = 0;

  // Gyro bias (quick calibration)
  static bool bias_done = false;
  static float gbx = 0, gby = 0, gbz = 0;
  static int bias_samples = 0;

  // Read IMU
  int16_t rg[6];
  if (bmi160.getAccelGyroData(rg) != 0) return;

  // Scale data
  const float GYR_LSB_PER_DPS = 16.4f;
  const float ACC_LSB_PER_G = 16384.0f;
  const float G0 = 9.80665f;
  const float DEG2RAD = PI / 180.0f;

  float gx_dps = rg[0] / GYR_LSB_PER_DPS;
  float gy_dps = rg[1] / GYR_LSB_PER_DPS;
  float gz_dps = rg[2] / GYR_LSB_PER_DPS;

  float ax = (rg[3] / ACC_LSB_PER_G) * G0;
  float ay = (rg[4] / ACC_LSB_PER_G) * G0;
  float az = (rg[5] / ACC_LSB_PER_G) * G0;

  // Quick bias calibration
  if (!bias_done) {
    if (bias_samples < 10) {
      if (bias_samples == 0) {
        gbx = gx_dps;
        gby = gy_dps;
        gbz = gz_dps;
      } else {
        gbx = (gbx + gx_dps) / 2.0f;
        gby = (gby + gy_dps) / 2.0f;
        gbz = (gbz + gz_dps) / 2.0f;
      }
      bias_samples++;
      return;
    } else {
      bias_done = true;
      Serial.println("IMU bias calibration complete");
    }
  }

  // Timing
  uint32_t now = micros();
  if (last_us == 0) {
    last_us = now;
    return;
  }
  float dt = (now - last_us) * 1e-6f;
  last_us = now;
  if (dt <= 0 || dt > 0.5f) dt = 0.1f;

  // Bias-corrected gyro (rad/s)
  float gx = (gx_dps - gbx) * DEG2RAD;
  float gy = (gy_dps - gby) * DEG2RAD;
  float gz = (gz_dps - gbz) * DEG2RAD;

  // Expose rates for stall detection (deg/s)
  imu_pan_rate  = gx_dps - gbx;
  imu_tilt_rate = gy_dps - gby;

  // Accelerometer angles
  float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

  // Kalman filter for tilt (gyro + accelerometer fusion)
  // Pan (yaw) has no absolute reference, so bias-corrected gyro integration is used
  static KalmanFilter kf_tilt;

  if (!initialized) {
    kf_tilt.angle = pitch_acc * (180.0f / PI);
    yaw = 0.0f;
    initialized = true;
  } else {
    float pitch_deg = kf_tilt.update(pitch_acc * (180.0f / PI), gy_dps - gby, dt);
    pitch = pitch_deg * DEG2RAD;
    yaw += gx * dt;
  }

  // Handle calibration request
  if (calibration_requested) {
    Serial.println("Calibrating IMU reference point...");

    // Set current position as the new reference (0,0)
    pan_offset = yaw * 180.0f / PI;
    tilt_offset = pitch * 180.0f / PI;

    Serial.printf("Pan offset set to: %.2f°\n", pan_offset);
    Serial.printf("Tilt offset set to: %.2f°\n", tilt_offset);

    calibration_requested = false;  // Clear the flag
  }

  // Convert to degrees and apply offset
  imu_pan_position = (yaw * 180.0f / PI) - pan_offset;
  imu_tilt_position = (pitch * 180.0f / PI) - tilt_offset;

  // Apply physical limits instead of wrapping
  imu_pan_position = constrain(imu_pan_position, PAN_PHYSICAL_LIMIT_MIN, PAN_PHYSICAL_LIMIT_MAX);
  imu_tilt_position = constrain(imu_tilt_position, TILT_PHYSICAL_LIMIT_MIN, TILT_PHYSICAL_LIMIT_MAX);

  pan_angle = round(imu_pan_position);
  tilt_angle = round(imu_tilt_position);
}


void calibrateIMU() {
  calibration_requested = true;
  Serial.println("Calibration requested - will apply on next IMU reading");
}


void updatePanPosition(void) {
  if (!TCA9545A(0) || !TCA9545A(2)) {
    Serial.println("Failed to select PAN bus");
    return;
  }
  if (checkFeed == 1) {
    // Request raw angle from AS5600 (0x0C = high byte)
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);
    Wire.endTransmission(false);
    Wire.requestFrom(0x36, 2);

    if (Wire.available() >= 2) {
      uint16_t high = Wire.read();
      uint16_t low = Wire.read();
      int new_raw = (high << 8) | low;  // 0–4095

      // Convert to degrees
      float current_angle = (new_raw * 360.0f) / 4096.0f;

      if (prev_pan_raw == -1) {
        prev_pan_raw = new_raw;
        return;  // Skip delta calculation on first read
      }

      float prev_angle = (prev_pan_raw * 360.0f) / 4096.0f;
      float delta = current_angle - prev_angle;

      // Wraparound correction
      if (delta > 180.0f) delta -= 360.0f;
      if (delta < -180.0f) delta += 360.0f;

      // Accumulate total motor angle
      motor_pan_total_angle += delta;
      prev_pan_raw = new_raw;

      // Convert motor shaft angle to output shaft angle using gear ratio
      float output_angle = motor_pan_total_angle / PAN_GEAR_RATIO;

      pan_angle = (int)output_angle;  // Store for display and logic

      // Optional: Print debug info
      //Serial.printf("Δ: %.2f°, Motor: %.2f°, Output: %.2f°\n", delta, motor_pan_total_angle, output_angle);
    }
  } else if (checkFeed == 2) {
    return;
  }
}

void updateTiltPosition(void) {
  if (!TCA9545A(1) || !TCA9545A(2)) {
    Serial.println("Failed to select TILT bus");
    return;
  }
  if (checkFeed == 1) {
    // Read AS5600 raw angle
    Wire.beginTransmission(0x36);
    Wire.write(0x0C);  // Angle high byte
    Wire.endTransmission(false);
    Wire.requestFrom(0x36, 2);

    if (Wire.available() >= 2) {
      uint16_t high = Wire.read();
      uint16_t low = Wire.read();
      int new_raw = (high << 8) | low;

      // Convert to degrees (0–360)
      float current_angle = (new_raw * 360.0f) / 4096.0f;

      if (prev_tilt_raw == -1) {
        prev_tilt_raw = new_raw;
        return;  // Skip delta on first read
      }

      float prev_angle = (prev_tilt_raw * 360.0f) / 4096.0f;
      float delta = current_angle - prev_angle;

      // Handle encoder wraparound
      if (delta > 180.0f) delta -= 360.0f;
      if (delta < -180.0f) delta += 360.0f;

      // Accumulate motor shaft angle
      motor_tilt_total_angle += delta;
      prev_tilt_raw = new_raw;

      // Convert to output shaft angle using gear ratio
      float output_angle = motor_tilt_total_angle / TILT_GEAR_RATIO;

      tilt_angle = (int)output_angle;

      // Optional debug
      Serial.printf("Tilt Δ: %.2f°, Total: %.2f°, Output: %.2f°\n", delta, motor_tilt_total_angle, output_angle);
    }
  } else if (checkFeed == 2) {
    return;
  }
}

void feedStatus(int check) {
  if (check == 1) {
    Serial.println("AS5600 Encoder Detected");
  } else if (check == 2) {
    Serial.println("BMI160 IMU Detected");
  } else {
    Serial.println("No feedback device active");
  }
}

void initDevices(void) {

  // Test TCA9545A
  Serial.println("Testing TCA9545A...");
  Wire.beginTransmission(0x70);
  if (Wire.endTransmission() == 0) {
    Serial.println("TCA9545A found!");
  } else {
    Serial.println("TCA9545A not found!");
  }

  // Test AS5600 on each bus
  Serial.println("Testing AS5600 sensors...");
  for (int bus = 0; bus < 4; bus++) {
    if (TCA9545A(bus)) {
      Wire.beginTransmission(0x36);
      if (Wire.endTransmission() == 0) {
        Serial.printf("AS5600 found on bus %d\n", bus);
        checkFeed = 1;
      } else {
        Serial.printf("No AS5600 on bus %d\n", bus);
        checkFeed = 0;
      }
    }
  }

  // Initialize display
  if (TCA9545A(3)) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, i2c_dispAddr)) {
      Serial.println(F("SSD1306 allocation failed"));
    }
    Serial.println("Display Initialized");
  }

  //init the hardware bmin160
  if (TCA9545A(2)) {
    if (bmi160.softReset() != BMI160_OK) {
      Serial.println("reset false");
    }
    //set and init the bmi160 i2c address
    if (bmi160.I2cInit(i2c_IMUAddr) != BMI160_OK) {
      Serial.println("BMI160 IMU Failed");
      checkFeed = 0;
    } else {
      Serial.println("BMI160 IMU Initialized");
      checkFeed = 2;
    }
  }
  feedStatus(checkFeed);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting dual AS5600 setup...");

  Wire.begin();
  Wire.setClock(100000);  // 100kHz for stability

  initDevices();

  ledcAttach(EN1, 5000, 8);
  ledcAttach(EN2, 5000, 8);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(gpLed, OUTPUT);
  digitalWrite(gpLed, LOW);

  camera_setup();

#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  if (TCA9545A(3)) {
    display.clearDisplay();
    display.drawBitmap(0, 0, epd_bitmap_Gavin_Logo, 128, 64, WHITE);
    display.display();
    delay(3000);
  }

  // WiFi setup
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_password);
  WiFiAddr = WiFi.softAPIP().toString();

  Serial.println("\nAccess Point started");
  Serial.print("SSID: ");
  Serial.println(ap_ssid);
  Serial.print("IP address: ");
  Serial.println(WiFiAddr);

  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFiAddr);
  Serial.println("' to connect");

  // Load previously discovered limits from NVS (skip if never calibrated)
  prefs.begin("ecorig", true);
  if (prefs.isKey("pan_min")) {
    PAN_SOFTWARE_LIMIT_MIN  = prefs.getFloat("pan_min",  PAN_SOFTWARE_LIMIT_MIN);
    PAN_SOFTWARE_LIMIT_MAX  = prefs.getFloat("pan_max",  PAN_SOFTWARE_LIMIT_MAX);
    TILT_SOFTWARE_LIMIT_MIN = prefs.getFloat("tilt_min", TILT_SOFTWARE_LIMIT_MIN);
    TILT_SOFTWARE_LIMIT_MAX = prefs.getFloat("tilt_max", TILT_SOFTWARE_LIMIT_MAX);
    Serial.printf("NVS limits loaded: Pan[%.1f, %.1f] Tilt[%.1f, %.1f]\n",
                  PAN_SOFTWARE_LIMIT_MIN, PAN_SOFTWARE_LIMIT_MAX,
                  TILT_SOFTWARE_LIMIT_MIN, TILT_SOFTWARE_LIMIT_MAX);
  } else {
    Serial.println("No NVS limits found — using defaults. Run auto-calibration.");
  }
  prefs.end();

  // Timer for motor control only
  const esp_timer_create_args_t timer_args = {
    .callback = &updateServoPositions,
    .name = "servo_update"
  };
  esp_timer_create(&timer_args, &timer);
  esp_timer_start_periodic(timer, 100 * 1000);  // 100ms interval

  Update_display();
  Serial.println("Setup complete!");
}

void loop() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastDisplayUpdate = 0;

  // Handle automatic calibration sequence
  if (auto_calibration_active) {
    performAutomaticCalibration();
  }

  // Update sensor readings every 200ms — always, including during calibration for stall detection
  if (millis() - lastUpdate > 200) {
    getGyro();
    lastUpdate = millis();
  }

  // Update display less frequently and only when needed
  if (millis() - lastDisplayUpdate > 500) {  // Every 500ms
    if (display_needs_update || (millis() - lastDisplayUpdate > 2000)) {  // Force update every 2 seconds
      
      // Small delay to ensure I2C bus is free
      delay(10);
      
      Update_display();
      display_needs_update = false;
      lastDisplayUpdate = millis();
    }
  }

  delay(10);
}