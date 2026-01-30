#include <Arduino.h>
#line 1 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
#include <math.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <PID_v1_bc.h>

// ────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ────────────────────────────────────────────────────────────────
#define ESC1 25 // Front-left motor (X configuration)
#define ESC2 26 // Front-right motor
#define ESC3 27 // Rear-right motor
#define ESC4 14 // Rear-left motor

#define CH_ROLL 32 // Flysky Channel 1 – Roll
#define CH_PITCH 33 // Flysky Channel 2 – Pitch
#define CH_THROTTLE 34 // Flysky Channel 3 – Throttle
#define CH_YAW 35 // Flysky Channel 4 – Yaw
#define CH_ARM 25 // Flysky Channel 5 – Arm/disarm switch
#define CH_AUX 26 // Flysky Channel 6 – Altitude hold toggle

#define BAT_ADC_PIN 34 // ADC input from battery voltage divider
#define BUZZER_PIN 13 // Buzzer for alerts

// ────────────────────────────────────────────────────────────────
//  CONSTANTS
// ────────────────────────────────────────────────────────────────
#define VDIV_RATIO 0.233f // Voltage divider ratio (calibrate!)
#define LOW_VOLT_CUTOFF 9.3f // Battery pack cutoff voltage (3.1 V/cell × 3)
#define VOLT_WARNING 10.2f // Early low-battery warning threshold
#define BAT_CHECK_MS 1000 // Battery check interval (ms)

#define FAILSAFE_TIMEOUT_MS 500 // No valid signal → disarm after this time
#define FAILSAFE_MIN_PULSE 800 // Pulse width considered invalid

// Complementary filter weight (higher = trust gyro more, less drift but slower correction)
const float alpha = 0.98f;

// Conversion factors
const float GYRO_SCALE = (180.0f / M_PI); // rad/s → deg/s

// ────────────────────────────────────────────────────────────────
//  PID OBJECTS
// ────────────────────────────────────────────────────────────────
double rollSetpoint = 0, rollInput, rollOutput;
double pitchSetpoint = 0, pitchInput, pitchOutput;
double yawRateSetpoint = 0, yawRateInput, yawRateOutput;
double altSetpoint = 0.5; // Initial target altitude (m)
double altInput, altOutput;

PID pidRoll(&rollInput, &rollOutput, &rollSetpoint, 4.0, 0.05, 1.0, DIRECT);
PID pidPitch(&pitchInput, &pitchOutput, &pitchSetpoint, 4.0, 0.05, 1.0, DIRECT);
PID pidYawRate(&yawRateInput, &yawRateOutput, &yawRateSetpoint, 5.0, 0.03, 0.2, DIRECT);
PID pidAlt(&altInput, &altOutput, &altSetpoint, 2.5, 0.2, 1.2, DIRECT);

// ────────────────────────────────────────────────────────────────
//  SENSOR OBJECTS
// ────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu; // 6-axis IMU (accel + gyro)
Adafruit_BMP280 bmp; // Barometric pressure / altitude sensor

// ────────────────────────────────────────────────────────────────
//  CALIBRATION OFFSETS (set once at startup)
// ────────────────────────────────────────────────────────────────
float gyroOffsetX = 0; // Gyro X bias (deg/s)
float gyroOffsetY = 0; // Gyro Y bias (deg/s)
float gyroOffsetZ = 0; // Gyro Z bias (deg/s)

float accelOffsetX = 0; // Accel X offset (m/s²)
float accelOffsetY = 0; // Accel Y offset (m/s²)
float accelOffsetZ = 0; // Accel Z offset (removes gravity ~9.81 m/s²)

float baroOffset = 0.0f; // Barometer altitude offset (makes startup height ≈ 0 m)

// ────────────────────────────────────────────────────────────────
//  FLIGHT STATE VARIABLES
// ────────────────────────────────────────────────────────────────
float roll = 0; // Estimated roll angle (degrees)
float pitch = 0; // Estimated pitch angle (degrees)
float yawRate = 0; // Yaw angular rate (deg/s)

float height = 0; // Estimated altitude above takeoff (meters)
float velocity = 0; // Estimated vertical velocity (m/s)

float gyroRollRate = 0; // Bias-corrected gyro roll rate (deg/s)
float gyroPitchRate = 0; // Bias-corrected gyro pitch rate (deg/s)

float accelX = 0, accelY = 0, accelZ = 0; // Bias-corrected acceleration (m/s²)

float baroAlt = 0; // Barometer altitude after offset correction (m)
float vbat = 0.0f; // Measured battery voltage (V)

unsigned long prevTime = 0; // Last loop timestamp (for dt calculation)

bool armed = false; // True when motors are allowed to spin
bool altitudeHoldEnabled = false; // True when altitude hold mode is active

float rcRoll = 0; // RC roll command (-45..45°)
float rcPitch = 0; // RC pitch command (-45..45°)
float rcYawRate = 0; // RC yaw rate command (deg/s)
float rcThrottle = 0; // RC throttle (0–1000)

int currentPwmAux = 0; // Raw pulse width from CH6 (altitude hold switch)

unsigned long lastValidSignalTime = 0; // Timestamp of last valid receiver pulse
bool receiverFailsafeActive = false; // True when receiver signal is lost

float m1, m2, m3, m4; // Final PWM values sent to each ESC (µs)

// ────────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────────
#line 114 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void setup();
#line 195 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void calibrateSensors();
#line 229 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void calibrateBarometer();
#line 253 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void loop();
#line 282 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void readReceiver();
#line 305 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void checkReceiverFailsafe();
#line 323 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void checkBatteryVoltage();
#line 354 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void updateArmingAndHoldMode();
#line 383 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void readSensors();
#line 402 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void fuseAttitude(float dt);
#line 413 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void fuseAltitude(float dt);
#line 428 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void runPIDs();
#line 443 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void mixAndWriteMotors();
#line 471 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void stopMotors();
#line 482 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void writeESC(uint8_t pin, float us);
#line 491 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void buzzerAlert(int count, int duration_ms);
#line 500 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void buzzerAlertContinuous();
#line 507 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void debugOutput();
#line 114 "C:\\Users\\uurae\\Documents\\ESP32-Drone-Project\\Motor_Radio_Control_Combined_v2\\Motor_Radio_Control_Combined_v2.ino"
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(BAT_ADC_PIN, INPUT);

  Serial.println("Starting sensor calibration...");
  Serial.println("Place quad FLAT and STILL on a table. Do NOT move it!");

  // MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 failed!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateSensors(); // Calibrate Gyro and Accel

  // BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 failed!");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  calibrateBarometer(); // Calibrate Barometer

  Serial.println("All sensors calibrated.");
  Serial.printf("Gyro offsets (deg/s): X=%.3f Y=%.3f Z=%.3f\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("Accel offsets (g):    X=%.3f Y=%.3f Z=%.3f\n", accelOffsetX, accelOffsetY, accelOffsetZ);
  Serial.printf("Baro offset: %.2f m\n", baroOffset);

  // Receiver pins
  pinMode(CH_ROLL, INPUT);
  pinMode(CH_PITCH, INPUT);
  pinMode(CH_THROTTLE, INPUT);
  pinMode(CH_YAW, INPUT);
  pinMode(CH_ARM, INPUT);
  pinMode(CH_AUX, INPUT);

  // ESC PWM setup (Arduino-ESP32 core 3.x API)
  const uint32_t PWM_FREQ = 50;
  const uint8_t  PWM_RES  = 16;

  if (!ledcAttach(ESC1, PWM_FREQ, PWM_RES)) Serial.println("ESC1 failed");
  if (!ledcAttach(ESC2, PWM_FREQ, PWM_RES)) Serial.println("ESC2 failed");
  if (!ledcAttach(ESC3, PWM_FREQ, PWM_RES)) Serial.println("ESC3 failed");
  if (!ledcAttach(ESC4, PWM_FREQ, PWM_RES)) Serial.println("ESC4 failed");

  writeESC(ESC1, 1000);
  writeESC(ESC2, 1000);
  writeESC(ESC3, 1000);
  writeESC(ESC4, 1000);
  delay(3500);

  // PID configuration
  pidRoll.SetMode(AUTOMATIC); pidRoll.SetOutputLimits(-250, 250);
  pidPitch.SetMode(AUTOMATIC); pidPitch.SetOutputLimits(-250, 250);
  pidYawRate.SetMode(AUTOMATIC); pidYawRate.SetOutputLimits(-180, 180);
  pidAlt.SetMode(AUTOMATIC); pidAlt.SetOutputLimits(-300, 400);

  Serial.println("Quad ready. Bind FS-i6. CH6 = altitude hold.");
  delay(2000);

  lastValidSignalTime = millis();
}

// ────────────────────────────────────────────────────────────────
// One-time IMU calibration (gyro + accel)
// ────────────────────────────────────────────────────────────────
void calibrateSensors() {
  const int numSamples = 200;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  float sumAx = 0, sumAy = 0, sumAz = 0;

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumGx += g.gyro.x;
    sumGy += g.gyro.y;
    sumGz += g.gyro.z;

    sumAx += a.acceleration.x;
    sumAy += a.acceleration.y;
    sumAz += a.acceleration.z;

    delay(10);
  }

  gyroOffsetX = (sumGx / numSamples) * GYRO_SCALE;
  gyroOffsetY = (sumGy / numSamples) * GYRO_SCALE;
  gyroOffsetZ = (sumGz / numSamples) * GYRO_SCALE;

  accelOffsetX = sumAx / numSamples;
  accelOffsetY = sumAy / numSamples;
  accelOffsetZ = (sumAz / numSamples) - 9.81f;

  tone(BUZZER_PIN, 1500, 300);
}

// ────────────────────────────────────────────────────────────────
// One-time barometer calibration (sets startup height ≈ 0 m)
// ────────────────────────────────────────────────────────────────
void calibrateBarometer() {
  const int numSamples = 100;
  float sumAlt = 0.0f;

  Serial.print("Calibrating barometer... ");

  for (int i = 0; i < numSamples; i++) {
    float alt = bmp.readAltitude(1013.25f);
    sumAlt += alt;
    delay(10);
  }

  baroOffset = sumAlt / numSamples;

  Serial.print("done. Offset set to ");
  Serial.print(baroOffset, 2);
  Serial.println(" m");

  tone(BUZZER_PIN, 1800, 200);
}

// ────────────────────────────────────────────────────────────────
// MAIN LOOP
// ────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt < 0.005f) return;
  prevTime = now;

  readReceiver();
  checkReceiverFailsafe();
  updateArmingAndHoldMode();
  checkBatteryVoltage();

  if (!armed) {
    stopMotors();
    delay(20);
    return;
  }

  readSensors();
  fuseAttitude(dt);
  fuseAltitude(dt);
  runPIDs();
  mixAndWriteMotors();

  debugOutput();
}

// ────────────────────────────────────────────────────────────────
// Read Flysky receiver channels
// ────────────────────────────────────────────────────────────────
void readReceiver() {
  int pwmRoll = pulseIn(CH_ROLL, HIGH, 30000);
  int pwmPitch = pulseIn(CH_PITCH, HIGH, 30000);
  int pwmThrottle = pulseIn(CH_THROTTLE, HIGH, 30000);
  int pwmYaw = pulseIn(CH_YAW, HIGH, 30000);
  int pwmArm = pulseIn(CH_ARM, HIGH, 30000);
  int pwmAux = pulseIn(CH_AUX, HIGH, 30000);

  if (pwmThrottle > 900 && pwmThrottle < 2100) {
    lastValidSignalTime = millis();
  }

  rcRoll = constrain(map(pwmRoll, 1000, 2000, -45, 45), -45, 45);
  rcPitch = constrain(map(pwmPitch, 1000, 2000, -45, 45), -45, 45);
  rcYawRate = constrain(map(pwmYaw, 1000, 2000, -220, 220), -220, 220);
  rcThrottle = constrain(map(pwmThrottle, 1000, 2000, 0, 1000), 0, 1000);

  currentPwmAux = pwmAux;
}

// ────────────────────────────────────────────────────────────────
// Check for receiver signal loss
// ────────────────────────────────────────────────────────────────
void checkReceiverFailsafe() {
  if (millis() - lastValidSignalTime > FAILSAFE_TIMEOUT_MS) {
    if (!receiverFailsafeActive) {
      receiverFailsafeActive = true;
      armed = false;
      altitudeHoldEnabled = false;
      stopMotors();
      Serial.println("FAILSAFE: No receiver signal – DISARMED!");
      buzzerAlert(4, 250);
    }
  } else {
    receiverFailsafeActive = false;
  }
}

// ────────────────────────────────────────────────────────────────
// Battery voltage monitoring + cutoff / warning
// ────────────────────────────────────────────────────────────────
void checkBatteryVoltage() {
  static unsigned long lastBatCheck = 0;
  if (millis() - lastBatCheck < BAT_CHECK_MS) return;
  lastBatCheck = millis();

  const int samples = 16;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(BAT_ADC_PIN);
    delay(2);
  }
  float adc_avg = sum / (float)samples;
  float v_divided = (adc_avg / 4095.0f) * 3.3f;
  vbat = v_divided / VDIV_RATIO;

  if (vbat < LOW_VOLT_CUTOFF && armed) {
    armed = false;
    altitudeHoldEnabled = false;
    stopMotors();
    Serial.printf("LOW VOLTAGE CUTOFF! VBat = %.2f V\n", vbat);
    buzzerAlertContinuous();
  }
  else if (vbat < VOLT_WARNING) {
    Serial.printf("LOW BATTERY WARNING! VBat = %.2f V\n", vbat);
    buzzerAlert(2, 200);
  }
}

// ────────────────────────────────────────────────────────────────
// Arm/disarm + altitude hold toggle
// ────────────────────────────────────────────────────────────────
void updateArmingAndHoldMode() {
  bool newHoldMode = (currentPwmAux > 1500);

  if (newHoldMode != altitudeHoldEnabled) {
    double savedKi = pidAlt.GetKi();
    pidAlt.SetTunings(pidAlt.GetKp(), 0.0, pidAlt.GetKd());
    pidAlt.Compute();
    pidAlt.SetTunings(pidAlt.GetKp(), savedKi, pidAlt.GetKd());

    if (newHoldMode) {
      altSetpoint = height;
    }
    altitudeHoldEnabled = newHoldMode;
  }

  if (currentPwmAux > 1800 && rcThrottle < 1100) {
    armed = true;
    if (altitudeHoldEnabled) altSetpoint = height;
    noTone(BUZZER_PIN);  // Stop any continuous tone on re-arm
  }
  if (currentPwmAux < 1200 || rcThrottle < 1050) {
    armed = false;
    altitudeHoldEnabled = false;
  }
}

// ────────────────────────────────────────────────────────────────
// Read sensors with calibration offsets applied
// ────────────────────────────────────────────────────────────────
void readSensors() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyroPitchRate = (g.gyro.y * GYRO_SCALE) - gyroOffsetY;
  gyroRollRate = (g.gyro.x * GYRO_SCALE) - gyroOffsetX;
  yawRate = (-g.gyro.z * GYRO_SCALE) - gyroOffsetZ;

  accelX = a.acceleration.x - accelOffsetX;
  accelY = a.acceleration.y - accelOffsetY;
  accelZ = a.acceleration.z - accelOffsetZ;

  static float seaLevel = 1013.25f;
  baroAlt = bmp.readAltitude(seaLevel) - baroOffset;
}

// ────────────────────────────────────────────────────────────────
// Complementary filter for roll & pitch
// ────────────────────────────────────────────────────────────────
void fuseAttitude(float dt) {
  float accelPitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * GYRO_SCALE;
  float accelRoll = atan2(-accelY, accelZ) * GYRO_SCALE;

  pitch = alpha * (pitch + gyroPitchRate * dt) + (1.0f - alpha) * accelPitch;
  roll = alpha * (roll + gyroRollRate  * dt) + (1.0f - alpha) * accelRoll;
}

// ────────────────────────────────────────────────────────────────
// Complementary filter for altitude & vertical velocity
// ────────────────────────────────────────────────────────────────
void fuseAltitude(float dt) {
  float cr = cos(roll * DEG_TO_RAD);
  float cp = cos(pitch * DEG_TO_RAD);
  float sr = sin(roll * DEG_TO_RAD);
  float sp = sin(pitch * DEG_TO_RAD);

  float zAccel = (accelZ * cr * cp - accelY * sr - accelX * sp) - 9.81f;

  velocity = 0.92f * (velocity + zAccel * dt) + 0.08f * ((baroAlt - height) / dt);
  height = 0.92f * (height + velocity * dt) + 0.08f * baroAlt;
}

// ────────────────────────────────────────────────────────────────
// Run PID controllers
// ────────────────────────────────────────────────────────────────
void runPIDs() {
  rollInput = roll;
  pitchInput = pitch;
  yawRateInput = yawRate;
  altInput = height;

  pidRoll.Compute();
  pidPitch.Compute();
  pidYawRate.Compute();
  pidAlt.Compute();
}

// ────────────────────────────────────────────────────────────────
// Motor mixing (X configuration) & ESC output
// ────────────────────────────────────────────────────────────────
void mixAndWriteMotors() {
  float throttle;
  if (altitudeHoldEnabled) {
    throttle = constrain(altOutput + 1500, 1000, 2000);
  } else {
    throttle = map(rcThrottle, 0, 1000, 1000, 1800);
    throttle = constrain(throttle, 1000, 1800);
  }

  m1 = throttle - rollOutput + pitchOutput - yawRateOutput;
  m2 = throttle + rollOutput + pitchOutput + yawRateOutput;
  m3 = throttle + rollOutput - pitchOutput - yawRateOutput;
  m4 = throttle - rollOutput - pitchOutput + yawRateOutput;

  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  writeESC(ESC1, m1);
  writeESC(ESC2, m2);
  writeESC(ESC3, m3);
  writeESC(ESC4, m4);
}

// ────────────────────────────────────────────────────────────────
// Emergency motor stop
// ────────────────────────────────────────────────────────────────
void stopMotors() {
  writeESC(ESC1, 1000);
  writeESC(ESC2, 1000);
  writeESC(ESC3, 1000);
  writeESC(ESC4, 1000);
  noTone(BUZZER_PIN);
}

// ────────────────────────────────────────────────────────────────
// Write PWM pulse to ESC (microseconds)
// ────────────────────────────────────────────────────────────────
void writeESC(uint8_t pin, float us) {
  us = constrain(us, 800, 2200);
  uint32_t duty = (uint32_t)((us * 65535UL) / 20000UL);
  ledcWrite(pin, duty);
}

// ────────────────────────────────────────────────────────────────
// Buzzer alert patterns
// ────────────────────────────────────────────────────────────────
void buzzerAlert(int count, int duration_ms) {
  for (int i = 0; i < count; i++) {
    tone(BUZZER_PIN, 1200);
    delay(duration_ms);
    noTone(BUZZER_PIN);
    if (i < count - 1) delay(150);
  }
}

void buzzerAlertContinuous() {
  tone(BUZZER_PIN, 800);
}

// ────────────────────────────────────────────────────────────────
// Periodic debug output to Serial
// ────────────────────────────────────────────────────────────────
void debugOutput() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 250) return;
  lastPrint = millis();

  float throttleDisplay = altitudeHoldEnabled ? (altOutput + 1500) : map(rcThrottle, 0, 1000, 1000, 1800);

  Serial.printf("Arm:%d Hold:%d FSafe:%d | R:%.1f P:%.1f YR:%.0f H:%.2f Thr:%.0f SP:%.2f VBat:%.2f V\n",
                armed, altitudeHoldEnabled, receiverFailsafeActive,
                roll, pitch, yawRate, height, throttleDisplay, altSetpoint, vbat);
}

