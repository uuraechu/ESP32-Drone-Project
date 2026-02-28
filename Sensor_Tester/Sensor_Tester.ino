#include <math.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

#define GRAVITY_CON 9.81f

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;

const float GYRO_SCALE = (180.0f / M_PI); // rad/s → deg/s

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float baroOffset = 0.0f;

float gyroRollRate = 0, gyroPitchRate = 0, yawRate = 0;
float accelPitch = 0, accelRoll  = 0;
float accelX = 0, accelY = 0, accelZ = 0;
float baroAlt = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  initSensors();
  calibrateMPU6050();
  calibrateBarometer();
}

void loop() {
  readSensors();
  delay(50);
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
  accelOffsetZ = (sumAz / numSamples) - GRAVITY_CON;
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
}

void readSensors() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Standard drone body-frame (your description):
  // X = left-right (positive right, pitch rotation axis)
  // Y = front-back (positive forward, roll rotation axis)
  // Z = down-up (positive up, yaw rotation axis)

  gyroPitchRate = ((g.gyro.x * GYRO_SCALE) - gyroOffsetX) * DEG_TO_RAD; // gyro.x = left-right → pitch rate (rad/s)
  gyroRollRate = ((g.gyro.y * GYRO_SCALE) - gyroOffsetY) * DEG_TO_RAD; // gyro.y = front-back → roll rate (rad/s)
  yawRate = -((g.gyro.z * GYRO_SCALE) - gyroOffsetZ) * DEG_TO_RAD; // gyro.z = up → yaw (negative for CW) (rad/s)

  accelX = a.acceleration.x - accelOffsetX; // left-right
  accelY = a.acceleration.y - accelOffsetY; // front-back
  accelZ = a.acceleration.z - accelOffsetZ; // up

  // ── Y-axis offset compensation (IMU forward of center) ──
  // gyroOffsetY_meters = positive forward offset
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
  accelPitch = atan2(correctedAccelX, sqrt(correctedAccelY * correctedAccelY + correctedAccelZ * correctedAccelZ)) * GYRO_SCALE;
  accelRoll = atan2(-correctedAccelY, -correctedAccelZ) * GYRO_SCALE;

  // Overwrite raw values for consistency in fusion/telemetry
  accelX = correctedAccelX;
  accelY = correctedAccelY;
  accelZ = correctedAccelZ;

  static float seaLevel = 1013.25f;
  baroAlt = bmp.readAltitude(seaLevel) - baroOffset;

  Serial.print("AccelX:");
  Serial.print(accelX);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(accelY);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(accelZ);
  Serial.print(",");
  Serial.print("PitchRate:");
  Serial.print(gyroPitchRate);
  Serial.print(",");
  Serial.print("RollRate:");
  Serial.print(gyroRollRate);
  Serial.print(",");
  Serial.print("YawRate:");
  Serial.print(yawRate);
  Serial.print(",");
  Serial.print("Altitude:");
  Serial.println(baroAlt);
}
