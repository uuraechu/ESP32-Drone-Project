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
#define FAILSAFE_TIMEOUT_MS 500
#define GRAVITY_CON 9.81f
#define LOOP_INTERVAL_MS 5

const float seaLevel = 1013.25f;
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

PID pidRoll (&rollInput, &rollOutput, &rollSetpoint, 2.5, 0.02, 0.8, DIRECT);
PID pidPitch (&pitchInput, &pitchOutput, &pitchSetpoint, 2.5, 0.02, 0.8, DIRECT);
PID pidYawRate (&yawRateInput, &yawRateOutput, &yawRateSetpoint, 3.5, 0.02, 0.1, DIRECT);
PID pidAlt (&altInput, &altOutput, &altSetpoint, 2.0, 0.15, 1.0, DIRECT);
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
float temperature = 0.0f;

bool armed = false;
bool altitudeHoldEnabled = false;
bool receiverFailsafeActive = false;
bool lowBatCutoff = false; // latched cutoff state
bool lowBatWarning = false; // separate warning flag

unsigned long prevTime = 0;
unsigned long lastValidSignalTime = 0;
uint16_t pwmRoll = 0, pwmPitch = 0, pwmThrottle = 0;
uint16_t pwmYaw = 0, pwmArm = 0, pwmAux = 0;
uint16_t m1, m2, m3, m4;

float rcRoll = 0, rcPitch = 0, rcYawRate = 0, rcThrottle = 0;

const float motorMin[4] = {1000, 1040, 1050, 1070}; // Tune these for your motors
const float motorMax = 2000.0f;  // Same for all (full power)

struct BuzzerState {
  bool active = false;
  int beepsRemaining = 0; // 0 = continuous mode
  int frequency = 0;
  int duration_ms = 0;
  int pause_ms = 150; // time between beeps (or silence in continuous)
  bool isTone = false;
  unsigned long lastToggle = 0;
  bool continuous = false; // flag for continuous mode
};

BuzzerState buzzer;

// Mutex for shared data between cores:
SemaphoreHandle_t dataMutex;

const float alpha = 0.98f;
const float alpha_inv = 1.0f - alpha;

// Global dt (updated every loop iteration)
float global_dt = 0.0f;

// ── FORWARD DECLARATIONS ─────────────────────────────────────────
void handleRoot();
void handleData();
void buzzerAlert(int count, int duration_ms, int freq = 1200, int pause_ms = 150);
void buzzerAlertContinuous(int freq = 800, int duration_ms = 200, int pause_ms = 100);
void updateBuzzer();
void stopMotors();
void writeESC(uint8_t pin, uint16_t us);

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

  unsigned long startTime = millis();
  lastValidSignalTime = startTime;
  prevTime = startTime;
}

// ────────────────────────────────────────────────────────────────
// MAIN LOOP
// ────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - prevTime;
  if (elapsed < LOOP_INTERVAL_MS) return;

  readReceiver();
  checkReceiverFailsafe();
  updateArmingAndHoldMode();
  checkBatteryVoltage();
  updateBuzzer();

  if (!armed) {
    prevTime = now;
    stopMotors();
    updateLEDs();
    return;
  }

  readSensors();

  unsigned long now2 = millis();
  unsigned long elapsed2 = now2 - prevTime;
  prevTime = now2;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  global_dt = elapsed2 / 1000.0f;

  // Setpoints applied inside mutex — consistent with PID compute
  rollSetpoint = rcRoll;
  pitchSetpoint = rcPitch;
  yawRateSetpoint = rcYawRate;

  if (!altitudeHoldEnabled) {
    const float track_alpha = 0.01f;
    altSetpoint = track_alpha * height + (1.0f - track_alpha) * altSetpoint;
  }

  float cr, cp, sr, sp;
  fuseAttitude(global_dt, cr, cp, sr, sp);
  fuseAltitude(global_dt, cr, cp, sr, sp);
  runPIDs();
  xSemaphoreGive(dataMutex);

  mixAndWriteMotors();
  updateLEDs();
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
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 failed!");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_63);

  Wire.setClock(400000);
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
  pidRoll.SetMode(AUTOMATIC); pidRoll.SetOutputLimits(-250, 250); pidRoll.SetSampleTime(LOOP_INTERVAL_MS);
  pidPitch.SetMode(AUTOMATIC); pidPitch.SetOutputLimits(-250, 250); pidPitch.SetSampleTime(LOOP_INTERVAL_MS);
  pidYawRate.SetMode(AUTOMATIC); pidYawRate.SetOutputLimits(-180, 180); pidYawRate.SetSampleTime(LOOP_INTERVAL_MS);
  pidAlt.SetMode(AUTOMATIC); pidAlt.SetOutputLimits(-300, 400); pidAlt.SetSampleTime(LOOP_INTERVAL_MS);
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

  dataMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(
    [](void*) {
      while (1) {
        server.handleClient();
        vTaskDelay(1);
      }
    },
    "WiFiTask",
    8192,
    nullptr,
    1,
    nullptr,
    0
  );
}

void calibrateAllSensors() {
  Serial.println("Calibrating sensors...");
  calibrateMPU6050(); // Gyro + Accel
  calibrateBarometer(); // Baro
  Serial.println("Calibration complete.");
  Serial.printf("Gyro offsets: X=%.3f Y=%.3f Z=%.3f rad/s\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
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

  gyroOffsetX = (sumGx / numSamples);
  gyroOffsetY = (sumGy / numSamples);
  gyroOffsetZ = (sumGz / numSamples);

  accelOffsetX = sumAx / numSamples;
  accelOffsetY = sumAy / numSamples;
  accelOffsetZ = (sumAz / numSamples) - GRAVITY_CON;

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
  tone(BUZZER_PIN, 1800, 200);
}

// ────────────────────────────────────────────────────────────────
// CORE FLIGHT FUNCTIONS
// ────────────────────────────────────────────────────────────────
void readReceiver() {
  const uint32_t timeout_us = 30000; // 30 ms

  pwmRoll = pulseIn(CH_ROLL, HIGH, timeout_us); if (pwmRoll < 800 || pwmRoll > 2200) pwmRoll = 1500;
  pwmPitch = pulseIn(CH_PITCH, HIGH, timeout_us); if (pwmPitch < 800 || pwmPitch > 2200) pwmPitch = 1500;
  pwmThrottle = pulseIn(CH_THROTTLE, HIGH, timeout_us); 
  pwmYaw = pulseIn(CH_YAW, HIGH, timeout_us); if (pwmYaw < 800 || pwmYaw > 2200) pwmYaw = 1500;
  pwmArm = pulseIn(CH_ARM, HIGH, timeout_us); if (pwmArm < 800 || pwmArm > 2200) pwmArm = 1000;
  pwmAux = pulseIn(CH_AUX, HIGH, timeout_us); if (pwmAux < 800 || pwmAux > 2200) pwmAux = 1000;

  if (pwmThrottle > 900 && pwmThrottle < 2100) lastValidSignalTime = millis();

  if (pwmThrottle == 0 || pwmThrottle < 800 || pwmThrottle > 2200) pwmThrottle = 1000;

  rcRoll = constrain((pwmRoll - 1500) / 500.0f * 45.0f, -45.0f, 45.0f);
  rcPitch = constrain((pwmPitch - 1500) / 500.0f * 45.0f, -45.0f, 45.0f);
  rcYawRate = constrain((pwmYaw - 1500) / 500.0f * 220.0f, -220.0f, 220.0f);
  rcThrottle = constrain((pwmThrottle - 1000) / 1000.0f * 1000.0f, 0.0f, 1000.0f);
}

void readSensors() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Standard drone body-frame (your description):
  // X = left-right (positive right, pitch rotation axis)
  // Y = front-back (positive forward, roll rotation axis)
  // Z = down-up (positive up, yaw rotation axis)

  gyroPitchRate = g.gyro.x - gyroOffsetX; // gyro.x = left-right → pitch rate (rad/s)
  gyroRollRate = g.gyro.y  - gyroOffsetY; // gyro.y = front-back → roll rate (rad/s)
  yawRate = -(g.gyro.z - gyroOffsetZ); // gyro.z = up → yaw (negative for CW) (rad/s)

  accelX = a.acceleration.x - accelOffsetX; // left-right
  accelY = a.acceleration.y - accelOffsetY; // front-back
  accelZ = a.acceleration.z - accelOffsetZ; // up

  // Physics: Rotation creates an INWARD (negative) acceleration component.
  // To get the true linear motion, we must REMOVE this component.
  // Vector Math: a_real = a_meas - (omega x (omega x r))
  // Result: We must ADD the magnitude (w^2 * r) to cancel the inward read.
  // Measured offsets from center of rotation (in meters)
  const float offsetX = 0.0f; // 0 mm X offset
  const float offsetY = (27.0f / 1000.0f); // 27 mm forward (+Y)
  const float offsetZ = (30.0f / 1000.0f); // 30 mm above (+Z)

  // Centripetal acceleration: ω² × r (always toward center of rotation)
  // Centripetal corrections (subtract inward pull - sign depends on your negation)
  float correctedAccelX = accelX + (gyroRollRate * gyroRollRate * offsetX);
  float correctedAccelY = accelY + (gyroPitchRate * gyroPitchRate * offsetY);
  float correctedAccelZ = accelZ + (gyroPitchRate * gyroPitchRate * offsetZ + gyroRollRate * gyroRollRate * offsetZ);

  // Gravity-referenced tilt angles (Z-up)
  accelPitch = -atan2(correctedAccelY, correctedAccelZ) * GYRO_SCALE;
  accelRoll = atan2(correctedAccelX, sqrt(pow(correctedAccelY, 2) + pow(correctedAccelZ, 2))) * GYRO_SCALE;

  // Overwrite raw values for consistency in fusion/telemetry
  accelX = correctedAccelX;
  accelY = correctedAccelY;
  accelZ = correctedAccelZ;

  baroAlt = bmp.readAltitude(seaLevel) - baroOffset;
  temperature = bmp.readTemperature();
}

// ────────────────────────────────────────────────────────────────
// FLIGHT LOGIC FUNCTIONS
// ────────────────────────────────────────────────────────────────
void checkReceiverFailsafe() {
  static int timeoutCounter = 0;
  const int TIMEOUT_CONFIRM_LOOPS = 5;  // ~25 ms at 5 ms loop
  static int goodSignalCounter = 0;
  const int GOOD_CONFIRM_LOOPS = 10;
  unsigned long now = millis();

  if (now - lastValidSignalTime > FAILSAFE_TIMEOUT_MS) {
    timeoutCounter++;
    goodSignalCounter = 0;

    if (timeoutCounter >= TIMEOUT_CONFIRM_LOOPS && !receiverFailsafeActive) {
      receiverFailsafeActive = true;
      armed = false;
      altitudeHoldEnabled = false;
      stopMotors();
      Serial.println("FAILSAFE: No receiver signal - DISARMED!");
      buzzerAlert(4, 250);
    }
  } else {
    timeoutCounter = 0;
    goodSignalCounter++;

    if (goodSignalCounter >= GOOD_CONFIRM_LOOPS && receiverFailsafeActive) {
      receiverFailsafeActive = false;
      Serial.println("Signal recovered - failsafe cleared");
    }
  }
}

void checkBatteryVoltage() {
  static unsigned long lastSampleTime = 0;
  static long sum = 0;
  static int sampleCount = 0;
  const int samples = 16;

  unsigned long now = millis();

  if (now - lastSampleTime >= 2) {
    lastSampleTime = now;
    int s = 0;
    for (int i = 0; i < 4; i++) s += analogRead(BAT_ADC_PIN);
    sum += s / 4;
    sampleCount++;
  }

  if (sampleCount >= samples) {
    float adc_avg = sum / (float)samples;
    float v_divided = (adc_avg / 4095.0f) * 3.3f;
    vbat = (v_divided * VDIV_M) + VDIV_C;

    sum = 0;
    sampleCount = 0;

    // Battery cutoff with hysteresis
    if (vbat < LOW_VOLT_CUTOFF && !lowBatCutoff) {
      lowBatCutoff = true;
      lowBatWarning = true;
      armed = false;
      altitudeHoldEnabled = false;
      stopMotors();
      Serial.printf("LOW VOLTAGE CUTOFF! VBat = %.2f V - LATCHED\n", vbat);
      buzzerAlertContinuous(800, 250, 100);  // continuous warning tone
    } else if (lowBatCutoff && vbat > 9.6f) {  // recovery threshold (hysteresis)
      lowBatCutoff = false;
      Serial.printf("Battery recovered: VBat = %.2f V - cutoff cleared\n", vbat);
      buzzerStop();
    } else if (vbat < VOLT_WARNING && !lowBatWarning) {
      lowBatWarning = true;
      Serial.printf("LOW BATTERY WARNING! VBat = %.2f V\n", vbat);
      buzzerAlert(2, 200, 1000, 150);
    } else if (vbat >= VOLT_WARNING) {
      lowBatWarning = false;
    }
  }
}

void updateArmingAndHoldMode() {
  // Altitude hold toggle (CH6 – independent of arming)
  bool auxHigh = (pwmAux > 1500);

  if (auxHigh != altitudeHoldEnabled) {
    if (auxHigh) {
      pidAlt.SetTunings(pidAlt.GetKp(), 0.0, pidAlt.GetKd());

      const float feedforward_factor = 0.001f;
      altSetpoint = height + (rcThrottle - 500.0f) * feedforward_factor;

      Serial.println("Altitude hold ENABLED");
      buzzerStop();
      tone(BUZZER_PIN, 1400, 200);
    } else {
      // Disabling hold → restore full integral term
      pidAlt.SetTunings(pidAlt.GetKp(), originalAltKi, pidAlt.GetKd());
      Serial.println("Altitude hold DISABLED - integral restored");
    }
    altitudeHoldEnabled = auxHigh;
  }

  // Arming logic (CH5 – 2-position switch with throttle safety)
  bool armSwitchHigh = (pwmArm > 1500); // Arm switch high = request arm
  bool throttleSafe = (rcThrottle < 200); // Throttle must be low to allow arming

  // Can only arm if:
  // - switch high
  // - throttle low
  // - NOT in low battery cutoff
  bool canArm = armSwitchHigh && throttleSafe && !lowBatCutoff;

  // Disarm if:
  // - switch low, OR
  // - low battery cutoff active
  bool forceDisarm = !armSwitchHigh || lowBatCutoff;

  // Arm transition (only if allowed)
  if (canArm && !armed) {
    armed = true;
    Serial.println("ARMED - Motors active");
    buzzerStop();
    tone(BUZZER_PIN, 1200, 300);
    digitalWrite(LED_ARMED, HIGH);
  }

  // Disarm transition
  if (forceDisarm && armed) {
    armed = false;
    altitudeHoldEnabled = false;  // force off
    stopMotors();
    Serial.println("DISARMED - Motors stopped");
    buzzerStop();
    tone(BUZZER_PIN, 600, 500);
    digitalWrite(LED_ARMED, LOW);
  }
}

void fuseAttitude(float dt, float &cr_out, float &cp_out, float &sr_out, float &sp_out) {
  pitch = alpha * (pitch + gyroPitchRate * GYRO_SCALE * dt) + alpha_inv * accelPitch;
  roll  = alpha * (roll + gyroRollRate * GYRO_SCALE * dt) + alpha_inv * accelRoll;

  float cr = cos(roll  * DEG_TO_RAD);
  float cp = cos(pitch * DEG_TO_RAD);
  float sr = sin(roll  * DEG_TO_RAD);
  float sp = sin(pitch * DEG_TO_RAD);

  cr_out = cr;
  cp_out = cp;
  sr_out = sr;
  sp_out = sp;
}

void fuseAltitude(float dt, float cr, float cp, float sr, float sp) {
  float zAccel = accelZ * (cp * cr) + accelY * (sp * cr) - accelX * sr - GRAVITY_CON;

  velocity = 0.92f * (velocity + zAccel * dt) + 0.08f * ((baroAlt - height) / dt);
  height = 0.92f * (height + velocity * dt) + 0.08f * baroAlt;
}

void runPIDs() {
  rollInput = roll;
  pitchInput = pitch;
  yawRateInput = yawRate * GYRO_SCALE;
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
    throttle = constrain((rcThrottle / 1000.0f) * 1000.0f + 1000.0f, 1000.0f, 2000.0f);
  }

  float m1_raw = throttle + rollOutput + pitchOutput - yawRateOutput;
  float m2_raw = throttle - rollOutput + pitchOutput + yawRateOutput;
  float m3_raw = throttle - rollOutput - pitchOutput - yawRateOutput;
  float m4_raw = throttle + rollOutput - pitchOutput + yawRateOutput;

  float m1_scaled = motorMin[0] + (m1_raw - 1000) * (motorMax - motorMin[0]) / (2000 - 1000);
  float m2_scaled = motorMin[1] + (m2_raw - 1000) * (motorMax - motorMin[1]) / (2000 - 1000);
  float m3_scaled = motorMin[2] + (m3_raw - 1000) * (motorMax - motorMin[2]) / (2000 - 1000);
  float m4_scaled = motorMin[3] + (m4_raw - 1000) * (motorMax - motorMin[3]) / (2000 - 1000);

  m1 = (uint16_t)constrain(m1_scaled, motorMin[0], 2000);
  m2 = (uint16_t)constrain(m2_scaled, motorMin[1], 2000);
  m3 = (uint16_t)constrain(m3_scaled, motorMin[2], 2000);
  m4 = (uint16_t)constrain(m4_scaled, motorMin[3], 2000);

  writeESC(ESC1, m1);
  writeESC(ESC2, m2);
  writeESC(ESC3, m3);
  writeESC(ESC4, m4);
}

// ────────────────────────────────────────────────────────────────
// UTILITY / OUTPUT FUNCTIONS
// ────────────────────────────────────────────────────────────────
void stopMotors() {
  writeESC(ESC1, (uint16_t) 1000);
  writeESC(ESC2, (uint16_t) 1000);
  writeESC(ESC3, (uint16_t) 1000);
  writeESC(ESC4, (uint16_t) 1000);
}

void writeESC(uint8_t pin, uint16_t us) {
  us = (uint16_t) constrain(us, 800, 2200);
  uint32_t duty = (uint32_t) ((us * 65535UL) / 20000UL);
  ledcWrite(pin, duty);
}

void buzzerAlert(int count, int duration_ms, int freq, int pause_ms) {
  if (buzzer.active) return; // don't interrupt running sequence

  buzzer.active = true;
  buzzer.beepsRemaining = count;
  buzzer.frequency = freq;
  buzzer.duration_ms = duration_ms;
  buzzer.pause_ms = pause_ms;
  buzzer.continuous = false;
  buzzer.isTone = false;
  buzzer.lastToggle = millis();
}

void buzzerAlertContinuous(int freq, int duration_ms, int pause_ms) {
  if (buzzer.active) return; // don't interrupt running sequence

  buzzer.active = true;
  buzzer.beepsRemaining = 0;
  buzzer.frequency = freq;
  buzzer.duration_ms = duration_ms;
  buzzer.pause_ms = pause_ms;
  buzzer.continuous = true;
  buzzer.isTone = false;
  buzzer.lastToggle = millis();
}

void buzzerStop() {
  noTone(BUZZER_PIN);
  buzzer.active = false;
  buzzer.isTone = false;
  buzzer.beepsRemaining = 0;
  buzzer.continuous = false;
}

void updateBuzzer() {
  if (!buzzer.active) return;

  unsigned long now = millis();
  unsigned long elapsed = now - buzzer.lastToggle;

  if (!buzzer.isTone) {
    // Waiting for next beep start (pause phase)
    if (elapsed >= (unsigned long)buzzer.pause_ms) {
      tone(BUZZER_PIN, buzzer.frequency);
      buzzer.isTone = true;
      buzzer.lastToggle = now;
    }
  } else {
    // Beep is playing
    if (elapsed >= (unsigned long)buzzer.duration_ms) {
      noTone(BUZZER_PIN);
      buzzer.isTone = false;
      buzzer.lastToggle = now;

      // If finite sequence, count down
      if (!buzzer.continuous && buzzer.beepsRemaining > 0) {
        buzzer.beepsRemaining--;
        if (buzzer.beepsRemaining <= 0) {
          buzzer.active = false;
        }
      }
    }
  }
}

void updateLEDs() {
  digitalWrite(LED_ARMED, armed ? HIGH : LOW);
  digitalWrite(LED_ALTHOLD, altitudeHoldEnabled ? HIGH : LOW);

  static unsigned long lastFsBlink = 0;
  static bool fsBlinkState = false;
  if (receiverFailsafeActive) {
    if (millis() - lastFsBlink > 150) {
      fsBlinkState = !fsBlinkState;
      digitalWrite(LED_FAILSAFE, fsBlinkState);
      lastFsBlink = millis();
    }
  } else {
    digitalWrite(LED_FAILSAFE, LOW);
  }

  static unsigned long lastLbBlink = 0;
  if (lowBatCutoff) {
    digitalWrite(LED_LOWBAT, HIGH);
  } else if (lowBatWarning) {
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

  Serial.printf("Arm:%d Hold:%d FSafe:%d | R:%.1f P:%.1f H:%.2f Thr:%.0f SP:%.2f VBat:%.2f | M1:%u M2:%u M3:%u M4:%u | dt:%.1f ms Loop rate: %.1f Hz\n",
                armed, altitudeHoldEnabled, receiverFailsafeActive,
                roll, pitch, height,
                altitudeHoldEnabled
                  ? (float)(altOutput + 1500)
                  : constrain((rcThrottle / 1000.0f) * 800.0f + 1000.0f, 1000.0f, 1800.0f),
                altSetpoint, vbat, m1, m2, m3, m4, global_dt * 1000.0f, 1.0f / global_dt);

}

// ────────────────────────────────────────────────────────────────
// HTTP TELEMETRY SERVER
// ────────────────────────────────────────────────────────────────
void handleData() {
  static float prevRollRate = 0, prevPitchRate = 0, prevYawRate_s = 0;
  static float rotAccRoll = 0, rotAccPitch = 0, rotAccYaw = 0;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  float l_roll = roll;
  float l_pitch = pitch;
  float l_yawRate = yawRate;
  float l_height = height;
  float l_velocity = velocity;
  float l_accelX = accelX;
  float l_accelY = accelY;
  float l_accelZ = accelZ;
  float l_gyroRoll = gyroRollRate;
  float l_gyroPitch = gyroPitchRate;
  float l_vbat = vbat;
  float l_dt = global_dt;
  bool  l_armed = armed;
  bool  l_altHold = altitudeHoldEnabled;
  uint16_t l_pwmRoll = pwmRoll;
  uint16_t l_pwmPitch = pwmPitch;
  uint16_t l_pwmThrottle = pwmThrottle;
  uint16_t l_pwmYaw = pwmYaw;
  uint16_t l_pwmArm = pwmArm;
  uint16_t l_pwmAux = pwmAux;
  float l_temperature = temperature; // safe snapshot — written on Core 1, float is atomic
  xSemaphoreGive(dataMutex);

  if (l_dt > 0.001f) {
    rotAccRoll  = 0.7f * rotAccRoll  + 0.3f * ((l_gyroRoll  - prevRollRate)  / l_dt) * GYRO_SCALE;
    rotAccPitch = 0.7f * rotAccPitch + 0.3f * ((l_gyroPitch - prevPitchRate) / l_dt) * GYRO_SCALE;
    rotAccYaw   = 0.7f * rotAccYaw   + 0.3f * ((l_yawRate   - prevYawRate_s) / l_dt) * GYRO_SCALE;
  }
  prevRollRate  = l_gyroRoll;
  prevPitchRate = l_gyroPitch;
  prevYawRate_s = l_yawRate;

  char json[1024];
  int written = snprintf(json, sizeof(json),
    "{"
    "\"timestamp\":%lu,"
    "\"rcRoll\":%u,\"rcPitch\":%u,\"rcThrottle\":%u,\"rcYaw\":%u,\"rcArm\":%u,\"rcAux\":%u,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"yawRate\":%.2f,"
    "\"height\":%.2f,\"velocity\":%.2f,"
    "\"accelX\":%.2f,\"accelY\":%.2f,\"accelZ\":%.2f,"
    "\"rotVelRoll\":%.2f,\"rotVelPitch\":%.2f,\"rotVelYaw\":%.2f,"
    "\"rotAccRoll\":%.2f,\"rotAccPitch\":%.2f,\"rotAccYaw\":%.2f,"
    "\"temperature\":%.2f,"
    "\"vbat\":%.2f,"
    "\"armed\":%s,\"altitudeHoldEnabled\":%s"
    "}",
    millis(),
    l_pwmRoll, l_pwmPitch, l_pwmThrottle, l_pwmYaw, l_pwmArm, l_pwmAux,
    l_roll, l_pitch, l_yawRate,
    l_height, l_velocity,
    l_accelX, l_accelY, l_accelZ,
    l_gyroRoll  * GYRO_SCALE,
    l_gyroPitch * GYRO_SCALE,
    l_yawRate   * GYRO_SCALE,
    rotAccRoll, rotAccPitch, rotAccYaw,
    l_temperature,
    l_vbat,
    l_armed   ? "true" : "false",
    l_altHold ? "true" : "false"
  );

  if (written >= (int)sizeof(json))
    Serial.println("WARNING: JSON buffer too small, output truncated!");

  server.send(200, "application/json", json);
}

void handleRoot() {
  static const char html[] = R"rawliteral(
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
