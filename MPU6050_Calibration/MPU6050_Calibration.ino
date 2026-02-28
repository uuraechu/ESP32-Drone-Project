// ────────────────────────────────────────────────────────────────
// MPU6050 Hard Offset Calibration Tool
// Flash this to the ESP32 ONCE on a known-flat stable platform.
// Copy the printed constants into the main flight controller code.
// Do NOT use this sketch for flight — it has no flight code.
//
// Instructions:
//   1. Mount ESP32 + MPU6050 in the frame exactly as they will fly
//   2. Place frame on a flat surface (verify with spirit level)
//   3. Flash this sketch
//   4. Open Serial Monitor at 115200 baud
//   5. Keep completely still — takes ~25 seconds for 3 runs
//   6. Copy the output block at the end into the flight code
// ────────────────────────────────────────────────────────────────

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define GRAVITY_CON  9.81f
#define BUZZER_PIN   13
#define NUM_SAMPLES  500    // per run — higher = more accurate average
#define NUM_RUNS     3      // runs to average — catches thermal drift
#define SAMPLE_DELAY 5      // ms between samples

Adafruit_MPU6050 mpu;

// ── Per-run results ──────────────────────────────────────────────
struct OffsetResult {
  float gx, gy, gz;
  float ax, ay, az;
};

// ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println("==============================================");
  Serial.println("  MPU6050 Hard Offset Calibration Tool");
  Serial.println("==============================================");
  Serial.println();
  Serial.println("Requirements:");
  Serial.println("  - Frame mounted exactly as it will fly");
  Serial.println("  - Sitting on a spirit-level verified flat surface");
  Serial.println("  - Completely still for the entire measurement");
  Serial.println();

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found. Check wiring.");
    while (1) {
      tone(BUZZER_PIN, 400, 500);
      delay(600);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("MPU6050 found.");
  Serial.println();

  // Warm up — let sensor settle
  Serial.print("Warming up sensor (3 seconds)");
  for (int i = 0; i < 30; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    delay(100);
    if (i % 5 == 0) Serial.print(".");
  }
  Serial.println(" done.");
  Serial.println();

  tone(BUZZER_PIN, 1200, 200);
  delay(300);

  // ── Run measurements ─────────────────────────────────────────
  OffsetResult runs[NUM_RUNS];

  for (int run = 0; run < NUM_RUNS; run++) {
    Serial.printf("Run %d of %d — keep still...\n", run + 1, NUM_RUNS);

    float sumGx = 0, sumGy = 0, sumGz = 0;
    float sumAx = 0, sumAy = 0, sumAz = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      sumGx += g.gyro.x;
      sumGy += g.gyro.y;
      sumGz += g.gyro.z;
      sumAx += a.acceleration.x;
      sumAy += a.acceleration.y;
      sumAz += a.acceleration.z;

      delay(SAMPLE_DELAY);

      // Progress dot every 50 samples
      if (i % 50 == 0) Serial.print(".");
    }
    Serial.println();

    runs[run].gx = sumGx / NUM_SAMPLES;
    runs[run].gy = sumGy / NUM_SAMPLES;
    runs[run].gz = sumGz / NUM_SAMPLES;
    runs[run].ax = sumAx / NUM_SAMPLES;
    runs[run].ay = sumAy / NUM_SAMPLES;
    runs[run].az = sumAz / NUM_SAMPLES;

    Serial.printf("  Gyro  X: %+.6f  Y: %+.6f  Z: %+.6f rad/s\n",
                  runs[run].gx, runs[run].gy, runs[run].gz);
    Serial.printf("  Accel X: %+.6f  Y: %+.6f  Z: %+.6f m/s²\n",
                  runs[run].ax, runs[run].ay, runs[run].az);
    Serial.printf("  Accel Z - gravity: %+.6f m/s²\n",
                  runs[run].az - GRAVITY_CON);
    Serial.println();

    // Beep between runs
    if (run < NUM_RUNS - 1) {
      tone(BUZZER_PIN, 1500, 150);
      delay(2000);  // pause between runs
    }
  }

  // ── Average across runs ──────────────────────────────────────
  float avgGx = 0, avgGy = 0, avgGz = 0;
  float avgAx = 0, avgAy = 0, avgAz = 0;

  for (int i = 0; i < NUM_RUNS; i++) {
    avgGx += runs[i].gx;
    avgGy += runs[i].gy;
    avgGz += runs[i].gz;
    avgAx += runs[i].ax;
    avgAy += runs[i].ay;
    avgAz += runs[i].az;
  }

  avgGx /= NUM_RUNS;
  avgGy /= NUM_RUNS;
  avgGz /= NUM_RUNS;
  avgAx /= NUM_RUNS;
  avgAy /= NUM_RUNS;
  avgAz /= NUM_RUNS;

  // ── Consistency check across runs ───────────────────────────
  Serial.println("==============================================");
  Serial.println("  Consistency check (max deviation across runs)");
  Serial.println("==============================================");

  float maxGyroDeviation  = 0;
  float maxAccelDeviation = 0;

  for (int i = 0; i < NUM_RUNS; i++) {
    maxGyroDeviation = max(maxGyroDeviation,
      max(max(fabsf(runs[i].gx - avgGx),
              fabsf(runs[i].gy - avgGy)),
              fabsf(runs[i].gz - avgGz)));

    maxAccelDeviation = max(maxAccelDeviation,
      max(max(fabsf(runs[i].ax - avgAx),
              fabsf(runs[i].ay - avgAy)),
              fabsf(runs[i].az - avgAz)));
  }

  bool gyroOk  = maxGyroDeviation  < 0.002f;  // < 2 mrad/s across runs
  bool accelOk = maxAccelDeviation < 0.05f;   // < 50 mm/s² across runs

  Serial.printf("  Gyro  max deviation: %.6f rad/s  %s\n",
                maxGyroDeviation,  gyroOk  ? "[PASS]" : "[WARN — drone may have moved]");
  Serial.printf("  Accel max deviation: %.6f m/s²   %s\n",
                maxAccelDeviation, accelOk ? "[PASS]" : "[WARN — drone may have moved]");
  Serial.println();

  if (!gyroOk || !accelOk) {
    Serial.println("WARNING: High deviation between runs.");
    Serial.println("Drone may have moved or surface is not stable.");
    Serial.println("Results are still shown but consider remeasuring.");
    Serial.println();
    // Four warning beeps
    for (int i = 0; i < 4; i++) {
      tone(BUZZER_PIN, 800, 200);
      delay(300);
    }
  } else {
    Serial.println("All runs consistent — offsets are reliable.");
    Serial.println();
    tone(BUZZER_PIN, 2000, 150);
    delay(200);
    tone(BUZZER_PIN, 2000, 150);
  }

  // ── Print copy-paste block ───────────────────────────────────
  Serial.println("==============================================");
  Serial.println("  Copy this block into your flight code");
  Serial.println("  Replace the existing hard offset defines");
  Serial.println("==============================================");
  Serial.println();
  Serial.println("// ── HARD IMU OFFSETS ──────────────────────────────────");
  Serial.println("// Generated by calibration tool");
  Serial.printf ("// %d runs x %d samples  |  Gyro dev: %.6f  Accel dev: %.6f\n",
                 NUM_RUNS, NUM_SAMPLES, maxGyroDeviation, maxAccelDeviation);
  Serial.println("// Remeasure if IMU is remounted, replaced, or readings");
  Serial.println("// show consistent non-zero pitch/roll at rest.");
  Serial.println("// Gyro units: rad/s    Accel units: m/s²");
  Serial.printf ("#define GYRO_OFFSET_X   %+.6ff\n", avgGx);
  Serial.printf ("#define GYRO_OFFSET_Y   %+.6ff\n", avgGy);
  Serial.printf ("#define GYRO_OFFSET_Z   %+.6ff\n", avgGz);
  Serial.println();
  Serial.printf ("#define ACCEL_OFFSET_X  %+.6ff\n", avgAx);
  Serial.printf ("#define ACCEL_OFFSET_Y  %+.6ff\n", avgAy);
  Serial.printf ("#define ACCEL_OFFSET_Z  %+.6ff   // raw mean - gravity\n",
                 avgAz - GRAVITY_CON);
  Serial.println("// ─────────────────────────────────────────────────────");
  Serial.println();
  Serial.println("Calibration complete. Reflash flight code with these values.");

  // Done — long beep
  tone(BUZZER_PIN, 1800, 500);
}

void loop() {
  // Nothing — all work done in setup()
  delay(1000);
}
