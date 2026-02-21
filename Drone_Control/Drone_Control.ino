#include <math.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <PID_v1_bc.h>
#include <WiFi.h>
#include <WebServer.h>

// ────────────────────────────────────────────────────────────────
// PIN DEFINITIONS
// ────────────────────────────────────────────────────────────────
#define ESC1 12 // Front-left motor (X configuration) (CW)
#define ESC2 19 // Front-right motor (CCW)
#define ESC3 27 // Rear-right motor (CW)
#define ESC4 14 // Rear-left motor (CCW)

#define CH_ROLL 32
#define CH_PITCH 33
#define CH_THROTTLE 34
#define CH_YAW 35
#define CH_ARM 25 // Arm/disarm switch
#define CH_AUX 26 // Altitude hold toggle

#define BAT_ADC_PIN 36 // Battery voltage divider
#define BUZZER_PIN 13 // Buzzer

#define LED_ARMED 15 // Green – Armed status
#define LED_LOWBAT 2 // Red – Low battery / cutoff
#define LED_ALTHOLD 4 // Blue – Altitude hold active
#define LED_FAILSAFE 16 // Yellow – Receiver failsafe

// ────────────────────────────────────────────────────────────────
// CONSTANTS
// ────────────────────────────────────────────────────────────────
#define VDIV_M 3.672f
#define VDIV_C 1.974f
#define LOW_VOLT_CUTOFF 9.3f
#define VOLT_WARNING 10.2f
#define BAT_CHECK_MS 1000

#define FAILSAFE_TIMEOUT_MS 500
#define FAILSAFE_MIN_PULSE 800

#define G_TO_MS2 9.81f

const float alpha = 0.98f;
const float GYRO_SCALE = (180.0f / M_PI); // rad/s → deg/s

// WiFi AP settings
const char* apSSID = "QuadTelemetry";
const char* apPassword = "flysafe123";

// ────────────────────────────────────────────────────────────────
// GLOBAL STATE VARIABLES
// ────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
WebServer server(80);

double rollSetpoint = 0, rollInput, rollOutput;
double pitchSetpoint = 0, pitchInput, pitchOutput;
double yawRateSetpoint = 0, yawRateInput, yawRateOutput;
double altSetpoint = 0.5;
double altInput, altOutput;

PID pidRoll(&rollInput, &rollOutput, &rollSetpoint, 2.0, 0.05, 1.0, DIRECT);
PID pidPitch(&pitchInput, &pitchOutput, &pitchSetpoint, 2.0, 0.05, 1.0, DIRECT);
PID pidYawRate(&yawRateInput, &yawRateOutput, &yawRateSetpoint, 5.0, 0.03, 0.2, DIRECT);
PID pidAlt(&altInput, &altOutput, &altSetpoint, 2.5, 0.2, 1.2, DIRECT);
double originalAltKi = 0;

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float baroOffset = 0.0f;

float roll = 0, pitch = 0, yawRate = 0;
float height = 0, velocity = 0;
float gyroRollRate = 0, gyroPitchRate = 0;
float accelPitch = 0, accelRoll  = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float baroAlt = 0;
float vbat = 0.0f;

unsigned long prevTime = 0;
unsigned long lastValidSignalTime = 0;

bool armed = false;
bool altitudeHoldEnabled = false;
bool receiverFailsafeActive = false;

int pwmRoll = 0, pwmPitch = 0, pwmThrottle = 0;
int pwmYaw = 0, pwmArm = 0, pwmAux = 0;
float rcRoll = 0, rcPitch = 0, rcYawRate = 0, rcThrottle = 0;
int currentPwmAux = 0;

const float motorMin[4] = {1000, 1000, 1000, 1000};  // Tune these for your motors
const float motorMax = 2000.0f;  // Same for all (full power)
float m1, m2, m3, m4;

// Global dt (updated every loop iteration)
float global_dt = 0.0f;

// ────────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin();

  initPins();

  Serial.println("Starting sensor calibration...");
  Serial.println("Place quad FLAT and STILL on a table. Do NOT move it!");

  initSensors();
  calibrateAllSensors();

  initReceiver();
  initESCs();
  initPIDs();
  initHTTPTelemetry();

  Serial.println("Quad ready. Connect to WiFi: QuadTelemetry / flysafe123");
  Serial.println("Open browser at IP shown below for live telemetry.");
  Serial.println("Bind FS-i6. CH6 = altitude hold.");
  delay(2000);

  lastValidSignalTime = millis();
  prevTime = millis(); // Initialize dt reference
}

// ────────────────────────────────────────────────────────────────
// MAIN LOOP
// ────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  global_dt = (now - prevTime) / 1000.0f;
  if (global_dt < 0.005f) return;
  prevTime = now;

  readReceiver();
  checkReceiverFailsafe();
  updateArmingAndHoldMode();
  checkBatteryVoltage();

  updateLEDs();
  server.handleClient(); // Update telemetry

  if (!armed) {
    stopMotors();
    delay(20);
    return;
  }

  readSensors();
  fuseAttitude(global_dt);
  fuseAltitude(global_dt);
  runPIDs();
  mixAndWriteMotors();

  debugOutput();
}

// ────────────────────────────────────────────────────────────────
// INITIALIZATION FUNCTIONS
// ────────────────────────────────────────────────────────────────
void initPins() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LED_ARMED, OUTPUT);
  pinMode(LED_LOWBAT, OUTPUT);
  pinMode(LED_ALTHOLD, OUTPUT);
  pinMode(LED_FAILSAFE, OUTPUT);
  digitalWrite(LED_ARMED, LOW);
  digitalWrite(LED_LOWBAT, LOW);
  digitalWrite(LED_ALTHOLD, LOW);
  digitalWrite(LED_FAILSAFE, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(BAT_ADC_PIN, INPUT);
}

void initSensors() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 failed!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 failed!");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

void initReceiver() {
  pinMode(CH_ROLL, INPUT);
  pinMode(CH_PITCH, INPUT);
  pinMode(CH_THROTTLE, INPUT);
  pinMode(CH_YAW, INPUT);
  pinMode(CH_ARM, INPUT);
  pinMode(CH_AUX, INPUT);
}

void initESCs() {
  const uint32_t PWM_FREQ = 50;
  const uint8_t PWM_RES = 16;

  ledcAttach(ESC1, PWM_FREQ, PWM_RES);
  ledcAttach(ESC2, PWM_FREQ, PWM_RES);
  ledcAttach(ESC3, PWM_FREQ, PWM_RES);
  ledcAttach(ESC4, PWM_FREQ, PWM_RES);

  writeESC(ESC1, 1000);
  writeESC(ESC2, 1000);
  writeESC(ESC3, 1000);
  writeESC(ESC4, 1000);
  delay(3500);
}

void initPIDs() {
  pidRoll.SetMode(AUTOMATIC); pidRoll.SetOutputLimits(-250, 250);
  pidPitch.SetMode(AUTOMATIC); pidPitch.SetOutputLimits(-250, 250);
  pidYawRate.SetMode(AUTOMATIC); pidYawRate.SetOutputLimits(-180, 180);
  pidAlt.SetMode(AUTOMATIC); pidAlt.SetOutputLimits(-300, 400);
  originalAltKi = pidAlt.GetKi();
}

void initHTTPTelemetry() {
  WiFi.softAP(apSSID, apPassword);
  IPAddress IP = WiFi.softAPIP();

  Serial.println("WiFi AP started");
  Serial.print("SSID: "); Serial.println(apSSID);
  Serial.print("Password: "); Serial.println(apPassword);
  Serial.print("Open browser at: http://"); Serial.println(IP);

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  Serial.println("HTTP telemetry server started.");
}

void calibrateAllSensors() {
  Serial.println("Calibrating sensors...");
  calibrateMPU6050(); // Gyro + Accel
  calibrateBarometer(); // Baro
  Serial.println("Calibration complete.");
  Serial.printf("Gyro offsets: X=%.3f Y=%.3f Z=%.3f deg/s\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("Accel offsets: X=%.3f Y=%.3f Z=%.3f m/s²\n", accelOffsetX, accelOffsetY, accelOffsetZ);
  Serial.printf("Baro offset: %.2f m\n", baroOffset);
}

// ────────────────────────────────────────────────────────────────
// CALIBRATION FUNCTIONS
// ────────────────────────────────────────────────────────────────
void calibrateMPU6050() {
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
  accelOffsetZ = (sumAz / numSamples) + 9.81f; // For Z-up, gravity = -g, offset adds g to zero

  tone(BUZZER_PIN, 1500, 300);
}

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
// CORE FLIGHT FUNCTIONS
// ────────────────────────────────────────────────────────────────
void readReceiver() {
  pwmRoll = pulseIn(CH_ROLL, HIGH, 30000); if (pwmRoll < 800 || pwmRoll > 2200) pwmRoll = 1500;
  pwmPitch = pulseIn(CH_PITCH, HIGH, 30000); if (pwmPitch < 800 || pwmPitch > 2200) pwmPitch = 1500;
  pwmThrottle = pulseIn(CH_THROTTLE, HIGH, 30000);
  pwmYaw = pulseIn(CH_YAW, HIGH, 30000); if (pwmYaw < 800 || pwmYaw > 2200) pwmYaw = 1500;
  pwmArm = pulseIn(CH_ARM, HIGH, 30000); if (pwmArm < 800 || pwmArm > 2200) pwmArm = 1000;
  pwmAux = pulseIn(CH_AUX, HIGH, 30000); if (pwmAux < 800 || pwmAux > 2200) pwmAux = 1000;

  // Update failsafe timer
  if (pwmThrottle > 900 && pwmThrottle < 2100) {
    lastValidSignalTime = millis();
  } else if (pwmThrottle < 800 || pwmThrottle > 2200) {
    pwmThrottle = 1000;
  }

  rcRoll = constrain(map(pwmRoll, 1000, 2000, -45, 45), -45, 45);
  rcPitch = constrain(map(pwmPitch, 1000, 2000, -45, 45), -45, 45);
  rcYawRate = constrain(map(pwmYaw, 1000, 2000, -220, 220), -220, 220);
  rcThrottle = constrain(map(pwmThrottle, 1000, 2000, 0, 1000), 0, 1000);

  currentPwmAux = pwmAux;
}

void readSensors() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Standard drone body-frame (your description):
  // X = left-right (positive right, pitch rotation axis)
  // Y = front-back (positive forward, roll rotation axis)
  // Z = down-up (positive up, yaw rotation axis)

  gyroPitchRate = (g.gyro.x * GYRO_SCALE) - gyroOffsetX; // gyro.x = left-right → pitch rate
  gyroRollRate = (g.gyro.y * GYRO_SCALE) - gyroOffsetY; // gyro.y = front-back → roll rate
  yawRate = (-g.gyro.z * GYRO_SCALE) - gyroOffsetZ; // gyro.z = up → yaw (negative for CW)

  accelX = (a.acceleration.x - accelOffsetX) * G_TO_MS2; // left-right
  accelY = (a.acceleration.y - accelOffsetY) * G_TO_MS2; // front-back
  accelZ = (a.acceleration.z - accelOffsetZ) * G_TO_MS2; // up

  // ── Y-axis offset compensation (IMU forward of center) ──
  // gyroOffsetY_meters = positive forward offset
  // Measured offsets from center of rotation (in meters)
  const float offsetX = 0.0f;
  const float offsetY = (27.0f / 1000.0f);

  // Centripetal acceleration: ω² × r (always toward center of rotation)
  // Tangential acceleration: α × r (perpendicular to radius, direction depends on rotation)

  // ── Corrections from roll rotation (ω = gyroRollRate around Y-axis) ──
  float centripetal_from_roll_X = gyroRollRate * gyroRollRate * offsetX; // in X direction
  float centripetal_from_roll_Z = gyroRollRate * gyroRollRate * offsetY; // in Z direction

  // ── Corrections from pitch rotation (ω = gyroPitchRate around X-axis) ──
  float centripetal_from_pitch_Y = gyroPitchRate * gyroPitchRate * offsetY; // in Y direction
  float centripetal_from_pitch_Z = gyroPitchRate * gyroPitchRate * offsetX; // in Z direction

  // Apply corrections (signs depend on coordinate conventions)
  float correctedAccelX = accelX - centripetal_from_roll_X; // centripetal pulls toward center
  float correctedAccelY = accelY - centripetal_from_pitch_Y;
  float correctedAccelZ = accelZ + centripetal_from_roll_Z + centripetal_from_pitch_Z; // tangential adds outward

  // Gravity-referenced tilt angles (Z-up)
  accelPitch = atan2(correctedAccelX, sqrt(correctedAccelY * correctedAccelY + correctedAccelZ * correctedAccelZ)) * GYRO_SCALE;
  accelRoll = atan2(-correctedAccelY, -correctedAccelZ) * GYRO_SCALE;

  // Overwrite raw values for consistency in fusion/telemetry
  accelX = correctedAccelX;
  accelY = correctedAccelY;
  accelZ = correctedAccelZ;

  static float seaLevel = 1013.25f;
  baroAlt = bmp.readAltitude(seaLevel) - baroOffset;
}

// ────────────────────────────────────────────────────────────────
// FLIGHT LOGIC FUNCTIONS
// ────────────────────────────────────────────────────────────────
void checkReceiverFailsafe() {
  if (millis() - lastValidSignalTime > FAILSAFE_TIMEOUT_MS) {
    if (!receiverFailsafeActive) {
      receiverFailsafeActive = true;
      armed = false;
      altitudeHoldEnabled = false;
      stopMotors();
      Serial.println("FAILSAFE: No receiver signal - DISARMED!");
      buzzerAlert(4, 250);
    }
  } else {
    receiverFailsafeActive = false;
  }
}

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
  vbat = (v_divided * VDIV_M) + VDIV_C;

  if (vbat < LOW_VOLT_CUTOFF && armed) {
    armed = false;
    altitudeHoldEnabled = false;
    stopMotors();
    Serial.printf("LOW VOLTAGE CUTOFF! VBat = %.2f V\n", vbat);
    buzzerAlertContinuous();
  } else if (vbat < VOLT_WARNING) {
    Serial.printf("LOW BATTERY WARNING! VBat = %.2f V\n", vbat);
    buzzerAlert(2, 200);
  }
}

void updateArmingAndHoldMode() {
  // Altitude hold toggle (CH6 – independent of arming)
  bool auxHigh = (pwmAux > 1500);

  if (auxHigh != altitudeHoldEnabled) {
    if (auxHigh) {
      // Enabling altitude hold → reset integral and set current height as target
      pidAlt.SetTunings(pidAlt.GetKp(), 0.0, pidAlt.GetKd());
      altSetpoint = height;
      Serial.println("Altitude hold ENABLED");
      tone(BUZZER_PIN, 1400, 200); // Short confirmation beep
    } else {
      // Disabling hold → restore full integral term
      pidAlt.SetTunings(pidAlt.GetKp(), originalAltKi, pidAlt.GetKd());
      Serial.println("Altitude hold DISABLED - integral restored");
    }
    altitudeHoldEnabled = auxHigh;
  }

  // Arming logic (CH5 – 2-position switch with throttle safety)
  bool armSwitchHigh = (pwmArm > 1500); // Arm switch high = request arm
  bool throttleSafe = (rcThrottle < 1100); // Throttle must be low to allow arming

  // Arm ONLY when switch high + throttle safe + currently disarmed
  bool shouldArm = armSwitchHigh && throttleSafe && !armed;

  // Disarm ONLY when arm switch is explicitly turned low
  bool shouldDisarm = !armSwitchHigh && armed;

  // Arm transition
  if (shouldArm) {
    armed = true;
    Serial.println("ARMED - Motors active");
    tone(BUZZER_PIN, 1200, 300);
    digitalWrite(LED_ARMED, HIGH);
  }

  // Disarm transition (only on switch low)
  if (shouldDisarm) {
    armed = false;
    altitudeHoldEnabled = false; // Safety: force hold off on disarm
    stopMotors();
    Serial.println("DISARMED - Motors stopped");
    tone(BUZZER_PIN, 600, 500);
    digitalWrite(LED_ARMED, LOW);
  }
}

void fuseAttitude(float dt) {
  pitch = alpha * (pitch + gyroPitchRate * dt) + (1.0f - alpha) * accelPitch;
  roll = alpha * (roll + gyroRollRate * dt) + (1.0f - alpha) * accelRoll;
}

void fuseAltitude(float dt) {
  float cr = cos(roll * DEG_TO_RAD);
  float cp = cos(pitch * DEG_TO_RAD);
  float sr = sin(roll * DEG_TO_RAD);
  float sp = sin(pitch * DEG_TO_RAD);

  // For Z-up, gravity is -G → net = measured - (-G) = measured + G
  float zAccel = (accelZ * cr * cp - accelX * sr - accelY * sp) + G_TO_MS2;

  velocity = 0.92f * (velocity + zAccel * dt) + 0.08f * ((baroAlt - height) / dt);
  height = 0.92f * (height + velocity * dt) + 0.08f * baroAlt;
}

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

void mixAndWriteMotors() {
  float throttle;
  if (altitudeHoldEnabled) {
    throttle = constrain(altOutput + 1500, 1000, 2000);
  } else {
    throttle = map(rcThrottle, 0, 1000, 1000, 1800);
    throttle = constrain(throttle, 1000, 1800);
  }

  // Calculate base mixing (same as before)
  float m1_raw = throttle - rollOutput + pitchOutput + yawRateOutput;
  float m2_raw = throttle + rollOutput + pitchOutput - yawRateOutput;
  float m3_raw = throttle + rollOutput - pitchOutput + yawRateOutput;
  float m4_raw = throttle - rollOutput - pitchOutput - yawRateOutput;

  // Linear scale each motor from its own min → 2000 µs
  m1 = motorMin[0] + (m1_raw - 1000) * (motorMax - motorMin[0]) / (2000 - 1000);
  m2 = motorMin[1] + (m2_raw - 1000) * (motorMax - motorMin[1]) / (2000 - 1000);
  m3 = motorMin[2] + (m3_raw - 1000) * (motorMax - motorMin[2]) / (2000 - 1000);
  m4 = motorMin[3] + (m4_raw - 1000) * (motorMax - motorMin[3]) / (2000 - 1000);

  // Safety constrain
  m1 = constrain(m1, motorMin[0], 2000);
  m2 = constrain(m2, motorMin[1], 2000);
  m3 = constrain(m3, motorMin[2], 2000);
  m4 = constrain(m4, motorMin[3], 2000);

  writeESC(ESC1, m1);
  writeESC(ESC2, m2);
  writeESC(ESC3, m3);
  writeESC(ESC4, m4);
}

// ────────────────────────────────────────────────────────────────
// UTILITY / OUTPUT FUNCTIONS
// ────────────────────────────────────────────────────────────────
void stopMotors() {
  writeESC(ESC1, 1000);
  writeESC(ESC2, 1000);
  writeESC(ESC3, 1000);
  writeESC(ESC4, 1000);
  noTone(BUZZER_PIN);

  digitalWrite(LED_ARMED, LOW);
  digitalWrite(LED_LOWBAT, LOW);
  digitalWrite(LED_ALTHOLD, LOW);
  digitalWrite(LED_FAILSAFE, LOW);
}

void writeESC(uint8_t pin, float us) {
  us = constrain(us, 800, 2200);
  uint32_t duty = (uint32_t)((us * 65535UL) / 20000UL);
  ledcWrite(pin, duty);
}

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

void updateLEDs() {
  digitalWrite(LED_ARMED, armed ? HIGH : LOW);
  digitalWrite(LED_ALTHOLD, altitudeHoldEnabled ? HIGH : LOW);

  static unsigned long lastFsBlink = 0;
  if (receiverFailsafeActive) {
    if (millis() - lastFsBlink > 150) {
      digitalWrite(LED_FAILSAFE, !digitalRead(LED_FAILSAFE));
      lastFsBlink = millis();
    }
  } else {
    digitalWrite(LED_FAILSAFE, LOW);
  }

  static unsigned long lastLbBlink = 0;
  if (vbat < LOW_VOLT_CUTOFF) {
    digitalWrite(LED_LOWBAT, HIGH);
  } else if (vbat < VOLT_WARNING) {
    if (millis() - lastLbBlink > 200) {
      digitalWrite(LED_LOWBAT, !digitalRead(LED_LOWBAT));
      lastLbBlink = millis();
    }
  } else {
    digitalWrite(LED_LOWBAT, LOW);
  }
}

void debugOutput() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 250) return;
  lastPrint = millis();

  float throttleDisplay = altitudeHoldEnabled ? (altOutput + 1500) : map(rcThrottle, 0, 1000, 1000, 1800);

  Serial.printf("Arm:%d Hold:%d FSafe:%d | R:%.1f P:%.1f YR:%.0f H:%.2f Thr:%.0f SP:%.2f VBat:%.2f V\n",
                armed, altitudeHoldEnabled, receiverFailsafeActive,
                roll, pitch, yawRate, height, throttleDisplay, altSetpoint, vbat);
}

// ────────────────────────────────────────────────────────────────
// HTTP TELEMETRY SERVER
// ────────────────────────────────────────────────────────────────
void handleData() {
  // Re-read latest sensor data for freshness
  readSensors();

  // Rotational acceleration (smoothed derivative using global_dt)
  static float prevRollRate = 0, prevPitchRate = 0, prevYawRate = 0;
  static float rotAccRoll = 0, rotAccPitch = 0, rotAccYaw = 0;
  if (global_dt > 0.001f) {  // Prevent division by zero
    rotAccRoll = 0.7f * rotAccRoll + 0.3f * ((gyroRollRate - prevRollRate) / global_dt);
    rotAccPitch = 0.7f * rotAccPitch + 0.3f * ((gyroPitchRate - prevPitchRate) / global_dt);
    rotAccYaw = 0.7f * rotAccYaw + 0.3f * ((yawRate - prevYawRate) / global_dt);
  }
  prevRollRate = gyroRollRate;
  prevPitchRate = gyroPitchRate;
  prevYawRate = yawRate;

  // Build comprehensive JSON
  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";

  // Receiver channels (raw PWM)
  json += "\"rcRoll\":" + String(pwmRoll) + ",";
  json += "\"rcPitch\":" + String(pwmPitch) + ",";
  json += "\"rcThrottle\":" + String(pwmThrottle) + ",";
  json += "\"rcYaw\":" + String(pwmYaw) + ",";
  json += "\"rcArm\":" + String(pwmArm) + ",";
  json += "\"rcAux\":" + String(pwmAux) + ",";

  // Attitude & rates
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"yawRate\":" + String(yawRate, 2) + ",";

  // Altitude & velocity
  json += "\"height\":" + String(height, 2) + ",";
  json += "\"velocity\":" + String(velocity, 2) + ",";

  // Acceleration (m/s²)
  json += "\"accelX\":" + String(accelX, 2) + ",";
  json += "\"accelY\":" + String(accelY, 2) + ",";
  json += "\"accelZ\":" + String(accelZ, 2) + ",";

  // Rotational velocity (gyro rates deg/s)
  json += "\"rotVelRoll\":" + String(gyroRollRate, 2) + ",";
  json += "\"rotVelPitch\":" + String(gyroPitchRate, 2) + ",";
  json += "\"rotVelYaw\":" + String(yawRate, 2) + ",";

  // Rotational acceleration (deg/s², smoothed using global_dt)
  json += "\"rotAccRoll\":" + String(rotAccRoll, 2) + ",";
  json += "\"rotAccPitch\":" + String(rotAccPitch, 2) + ",";
  json += "\"rotAccYaw\":" + String(rotAccYaw, 2) + ",";

  // Temperature from BMP280
  json += "\"temperature\":" + String(bmp.readTemperature(), 2) + ",";

  // Flight state
  json += "\"vbat\":" + String(vbat, 2) + ",";
  json += "\"armed\":" + String(armed ? "true" : "false") + ",";
  json += "\"altitudeHoldEnabled\":" + String(altitudeHoldEnabled ? "true" : "false");

  json += "}";

  server.send(200, "application/json", json);
}

// ────────────────────────────────────────────────────────────────
// HTTP TELEMETRY SERVER – Root page
// ────────────────────────────────────────────────────────────────
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Quad Telemetry</title>
  <style>
    body { font-family: Arial; background: #111; color: #0f0; margin: 20px; }
    h1 { color: #0f0; }
    pre { background: #222; padding: 15px; border-radius: 8px; font-size: 1.1em; }
  </style>
</head>
<body>
  <h1>ESP32 Quadcopter Live Data</h1>
  <p>Connected to: QuadTelemetry / flysafe123</p>
  <pre id="data">Loading...</pre>
  <script>
    setInterval(() => {
      fetch('/data')
        .then(r => r.json())
        .then(d => {
          document.getElementById('data').innerText = JSON.stringify(d, null, 2);
        });
    }, 1000);
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}
