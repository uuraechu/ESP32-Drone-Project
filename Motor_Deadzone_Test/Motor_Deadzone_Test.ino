// ESP32 - ESC Dead Zone / Startup Threshold Tester with Manual Selection
// Type 1–4 to test single ESC, 'a' for all four, 'q' to quit

#include <Arduino.h>

// ── Pins ───────────────────────────────────────────────────────────
#define ESC1 12 // Front-left   (CW)
#define ESC2 19 // Front-right  (CCW)
#define ESC3 27 // Rear-right   (CW)
#define ESC4 14 // Rear-left    (CCW)

// ── PWM constants ─────────────────────────────────────────────────
const uint32_t PWM_FREQ = 50; // Standard 50 Hz servo PWM
const uint8_t  PWM_RES = 16; // 16-bit resolution

// Test settings
const int START_PWM = 1000;
const int END_PWM = 1400;
const int STEP_SIZE = 10;
const int STEP_DELAY = 3000; // ms pause at each PWM value

// ── Globals ────────────────────────────────────────────────────────
int selectedESC = 0; // 0 = idle, 1–4 = single ESC, 5 = all
int currentPWM = START_PWM;
unsigned long lastStepTime = 0;
int currentTestMotor = 1; // For "all" mode: 1–4

// ────────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== ESC Dead Zone / Startup Threshold Tester ===\n");
  Serial.println("WARNING: REMOVE ALL PROPS BEFORE STARTING!");
  Serial.println("Commands:");
  Serial.println("  1  → test ESC1 only (front-left)");
  Serial.println("  2  → test ESC2 only (front-right)");
  Serial.println("  3  → test ESC3 only (rear-right)");
  Serial.println("  4  → test ESC4 only (rear-left)");
  Serial.println("  a  → test ALL four ESCs in sequence");
  Serial.println("  q  → quit / stop current test");
  Serial.println("----------------------------------------------------\n");

  // Configure all ESC outputs
  ledcAttach(ESC1, PWM_FREQ, PWM_RES);
  ledcAttach(ESC2, PWM_FREQ, PWM_RES);
  ledcAttach(ESC3, PWM_FREQ, PWM_RES);
  ledcAttach(ESC4, PWM_FREQ, PWM_RES);

  // Initial safe state
  stopAllESCs();

  delay(2000);
  Serial.println("Ready. Type a command (1-4, a, q) and press Enter.\n");
}

// ────────────────────────────────────────────────────────────────
// MAIN LOOP
// ────────────────────────────────────────────────────────────────
void loop() {
  // Handle Serial commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 1) {
      char cmd = input.charAt(0);

      if (cmd >= '1' && cmd <= '4') {
        selectedESC = cmd - '0';
        currentPWM = START_PWM;
        lastStepTime = millis();
        currentTestMotor = selectedESC;  // For display
        Serial.printf("\n=== Testing ESC%d (Motor %d) ===\n", selectedESC, selectedESC);
        Serial.printf("Ramping from 1000 to 1400 µs (10 µs steps, %d sec pause)", STEP_DELAY/1000);
        Serial.println("Observe when motor starts spinning smoothly.");
        Serial.println("Type 'q' to stop.\n");
      }
      else if (cmd == 'a' || cmd == 'A') {
        selectedESC = 5; // All mode
        currentTestMotor = 1;
        currentPWM = START_PWM;
        lastStepTime = millis();
        Serial.println("\n=== Testing ALL ESCs in sequence ===\n");
        Serial.println("Order: ESC1 → ESC2 → ESC3 → ESC4");
        Serial.println("Type 'q' to stop.\n");
      }
      else if (cmd == 'q' || cmd == 'Q') {
        if (selectedESC != 0) {
          Serial.printf("Test stopped (ESC%d).\n", selectedESC);
          selectedESC = 0;
          stopAllESCs();
        }
      }
    }
  }

  // Run active test
  if (selectedESC >= 1) {
    runDeadZoneTest();
  }

  delay(10);
}

// ────────────────────────────────────────────────────────────────
// Run one step of the dead zone ramp test
// ────────────────────────────────────────────────────────────────
void runDeadZoneTest() {
  if (millis() - lastStepTime < STEP_DELAY) return;
  lastStepTime = millis();

  int pin = 0;
  if (selectedESC == 5) {
    // All mode – cycle through 1–4
    pin = (currentTestMotor == 1) ? ESC1 :
          (currentTestMotor == 2) ? ESC2 :
          (currentTestMotor == 3) ? ESC3 : ESC4;
  } else {
    // Single mode
    pin = (selectedESC == 1) ? ESC1 :
          (selectedESC == 2) ? ESC2 :
          (selectedESC == 3) ? ESC3 : ESC4;
  }

  if (currentPWM <= END_PWM) {
    writeESC(pin, currentPWM);
    Serial.printf("ESC%d (Motor %d) at %d µs\n", (selectedESC == 5 ? currentTestMotor : selectedESC), (selectedESC == 5 ? currentTestMotor : selectedESC), currentPWM);
    currentPWM += STEP_SIZE;
  } else {
    // Test complete for this motor
    Serial.printf("Test complete for ESC%d (Motor %d). Reliable start around %d µs.\n\n",
                  (selectedESC == 5 ? currentTestMotor : selectedESC),
                  (selectedESC == 5 ? currentTestMotor : selectedESC),
                  currentPWM - STEP_SIZE);

    // If in "all" mode, move to next motor
    if (selectedESC == 5) {
      currentTestMotor++;
      if (currentTestMotor > 4) {
        Serial.println("All ESCs tested. Restarting sequence in 5 seconds...\n");
        currentTestMotor = 1;
        delay(5000);
      }
      currentPWM = START_PWM;
      lastStepTime = millis();
    } else {
      // Single mode – stop after one
      selectedESC = 0;
      currentPWM = START_PWM;
      stopAllESCs();
      Serial.println("Test finished. Type 1-4 or 'a' to start again.\n");
    }
  }
}

// ────────────────────────────────────────────────────────────────
// Write PWM value to ESC (constrained)
// ────────────────────────────────────────────────────────────────
void writeESC(int pin, float us) {
  us = constrain(us, 800.0f, 2200.0f);
  uint32_t duty = (uint32_t)((us * 65535UL) / 20000UL);
  ledcWrite(pin, duty);
}

// ────────────────────────────────────────────────────────────────
// Stop all ESCs (set to 1000 µs)
// ────────────────────────────────────────────────────────────────
void stopAllESCs() {
  writeESC(ESC1, 1000);
  writeESC(ESC2, 1000);
  writeESC(ESC3, 1000);
  writeESC(ESC4, 1000);
}
