#include <math.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <PID_v1_bc.h>
#include <WiFi.h>
#include <WebServer.h>

// ────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ────────────────────────────────────────────────────────────────
#define ESC1 25 // Front-left motor (X configuration)
#define ESC2 26 // Front-right motor
#define ESC3 27 // Rear-right motor
#define ESC4 14 // Rear-left motor

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
//  CONSTANTS
// ────────────────────────────────────────────────────────────────
#define VDIV_M 3.672f
#define VDIV_C 1.974f
#define LOW_VOLT_CUTOFF 9.3f
#define VOLT_WARNING 10.2f
#define BAT_CHECK_MS 1000

#define FAILSAFE_TIMEOUT_MS 500
#define FAILSAFE_MIN_PULSE 800

const float alpha = 0.98f;
const float GYRO_SCALE = (180.0f / M_PI);  // rad/s → deg/s

// WiFi AP settings
const char* apSSID = "QuadTelemetry";
const char* apPassword = "flysafe123";

// ────────────────────────────────────────────────────────────────
//  GLOBAL STATE VARIABLES
// ────────────────────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
WebServer server(80);

double rollSetpoint = 0, rollInput, rollOutput;
double pitchSetpoint = 0, pitchInput, pitchOutput;
double yawRateSetpoint = 0, yawRateInput, yawRateOutput;
double altSetpoint = 0.5;
double altInput, altOutput;

PID pidRoll(&rollInput, &rollOutput, &rollSetpoint, 4.0, 0.05, 1.0, DIRECT);
PID pidPitch(&pitchInput, &pitchOutput, &pitchSetpoint, 4.0, 0.05, 1.0, DIRECT);
PID pidYawRate(&yawRateInput, &yawRateOutput, &yawRateSetpoint, 5.0, 0.03, 0.2, DIRECT);
PID pidAlt(&altInput, &altOutput, &altSetpoint, 2.5, 0.2, 1.2, DIRECT);

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float baroOffset = 0.0f;

float roll = 0, pitch = 0, yawRate = 0;
float height = 0, velocity = 0;
float gyroRollRate = 0, gyroPitchRate = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float baroAlt = 0;
float vbat = 0.0f;

unsigned long prevTime = 0;
unsigned long lastValidSignalTime = 0;

bool armed = false;
bool altitudeHoldEnabled = false;
bool receiverFailsafeActive = false;

float rcRoll = 0, rcPitch = 0, rcYawRate = 0, rcThrottle = 0;
int currentPwmAux = 0;

float m1, m2, m3, m4;

// ────────────────────────────────────────────────────────────────
//  SETUP – One-time initialization
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
  updateLEDs();
  server.handleClient(); // Process HTTP requests

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

void calibrateAllSensors() {
  Serial.println("Calibrating sensors...");
  calibrateMPU650(); // Gyro + Accel
  calibrateBarometer(); // Baro
  Serial.println("Calibration complete.");
  Serial.printf("Gyro offsets: X=%.3f Y=%.3f Z=%.3f deg/s\n", gyroOffsetX, gyroOffsetY, gyroOffsetZ);
  Serial.printf("Accel offsets: X=%.3f Y=%.3f Z=%.3f m/s²\n", accelOffsetX, accelOffsetY, accelOffsetZ);
  Serial.printf("Baro offset: %.2f m\n", baroOffset);
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
}

// ────────────────────────────────────────────────────────────────
// CALIBRATION FUNCTIONS
// ────────────────────────────────────────────────────────────────
void calibrateMPU650() {
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
// SENSOR & INPUT FUNCTIONS
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
  bool newHoldMode = (currentPwmAux > 1500);

  if (newHoldMode != altitudeHoldEnabled) {
    double savedKi = pidAlt.GetKi();
    pidAlt.SetTunings(pidAlt.GetKp(), 0.0, pidAlt.GetKd());
    pidAlt.Compute();
    pidAlt.SetTunings(pidAlt.GetKp(), savedKi, pidAlt.GetKd());

    if (newHoldMode) altSetpoint = height;
    altitudeHoldEnabled = newHoldMode;
  }

  if (currentPwmAux > 1800 && rcThrottle < 1100) {
    armed = true;
    if (altitudeHoldEnabled) altSetpoint = height;
    noTone(BUZZER_PIN);
  }

  if (currentPwmAux < 1200 || rcThrottle < 1050) {
    armed = false;
    altitudeHoldEnabled = false;
  }
}

void fuseAttitude(float dt) {
  float accelPitch = atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ)) * GYRO_SCALE;
  float accelRoll  = atan2(-accelY, accelZ) * GYRO_SCALE;

  pitch = alpha * (pitch + gyroPitchRate * dt) + (1.0f - alpha) * accelPitch;
  roll  = alpha * (roll  + gyroRollRate  * dt) + (1.0f - alpha) * accelRoll;
}

void fuseAltitude(float dt) {
  float cr = cos(roll * DEG_TO_RAD);
  float cp = cos(pitch * DEG_TO_RAD);
  float sr = sin(roll * DEG_TO_RAD);
  float sp = sin(pitch * DEG_TO_RAD);

  float zAccel = (accelZ * cr * cp - accelY * sr - accelX * sp) - 9.81f;

  velocity = 0.92f * (velocity + zAccel * dt) + 0.08f * ((baroAlt - height) / dt);
  height   = 0.92f * (height + velocity * dt)   + 0.08f * baroAlt;
}

void runPIDs() {
  rollInput     = roll;
  pitchInput    = pitch;
  yawRateInput  = yawRate;
  altInput      = height;

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

void handleData() {
  String json = "{";
  json += "\"roll\":" + String(roll, 2) + ",";
  json += "\"pitch\":" + String(pitch, 2) + ",";
  json += "\"yawRate\":" + String(yawRate, 2) + ",";
  json += "\"height\":" + String(height, 2) + ",";
  json += "\"vbat\":" + String(vbat, 2) + ",";
  json += "\"armed\":" + String(armed ? "true" : "false") + ",";
  json += "\"altitudeHoldEnabled\":" + String(altitudeHoldEnabled ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}
