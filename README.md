# ESP32 Quadcopter Flight Controller

Custom lightweight flight controller built on ESP32 Dev Module for a 5-inch quadcopter.  
Uses MPU6050 + BMP280 sensor fusion, Flysky FS-iA6 receiver, PID control, altitude hold, failsafe, low-voltage cutoff, priority buzzer alerts, and optional status LEDs.

Includes live telemetry over WiFi (HTTP server — view in browser on phone or laptop).

---

## ⚠️ Safety First

- **Always remove props during bench testing.**
- **Always arm with throttle at minimum.**
- **Never fly over people or animals.**
- Hard voltage cutoff at 3.1 V/cell (9.3 V for 3S pack) — land before warning threshold (10.2 V).
- Change the default WiFi credentials before flying in public (see [Configuration](#configuration)).

---

## Features

- Complementary filtering for roll/pitch (gyro + accelerometer)
- Complementary filtering for altitude & velocity (accel Z + barometer)
- Altitude hold mode (toggled via CH6)
- PID control for roll, pitch, yaw rate, and altitude
- Flysky 6-channel PWM input (FS-iA6 receiver)
- Battery monitoring with latched cutoff, hysteresis recovery, and low-voltage warning
- Prioritised buzzer alert system (battery > failsafe > status)
- Receiver failsafe with debounced trigger and recovery (auto-disarm on signal loss)
- One-time power-on calibration (gyro, accel, barometer)
- Optional status LEDs (armed, low battery, altitude hold, failsafe)
- Live telemetry via WiFi AP (browser dashboard at 192.168.4.1)
- Dual-core safe: flight loop on Core 1, WiFi telemetry on Core 0 with mutex protection

---

## Hardware Overview

| Component | Spec |
|---|---|
| Frame | 5-inch, ≤200 g target (PETG HF) |
| Motors | 2300 KV (2205–2306 size) |
| Props | 5-inch 2-blade (e.g. 5×4.5×2) |
| Battery | 3S 2200 mAh 50C |
| Power | 5V buck converter (≥2 A recommended) |
| Receiver | Flysky FS-iA6 (PWM, 6-channel) |
| IMU | MPU6050 (gyro + accelerometer) |
| Barometer | BMP280 |
| Controller | ESP32 Dev Module |

Full component list → [COMPONENTS.md](COMPONENTS.md)  
Detailed wiring → [WIRING.md](WIRING.md)  
Troubleshooting → [TROUBLESHOOTING.md](TROUBLESHOOTING.md)  
Change history → [CHANGELOG.md](CHANGELOG.md)

---

## Configuration

Before flashing, update these values at the top of `Drone_Control.ino`:

```cpp
// ── USER CONFIGURATION ────────────────────────────────────────
#define WIFI_SSID     "QuadTelemetry"   // ← change before public use
#define WIFI_PASSWORD "flysafe123"       // ← change before public use
```

Also calibrate your voltage divider constants using a multimeter (see [Voltage Divider Calibration](#voltage-divider-calibration)).

---

## Software Requirements

- Arduino IDE + ESP32 Arduino core 3.x
- Libraries (install via Library Manager):
  - Adafruit MPU6050
  - Adafruit BMP280
  - Adafruit Unified Sensor
  - PID_v1_bc (or PID_v1)
  - WiFi.h and WebServer.h (built-in with ESP32 core)

---

## Setup & Usage

### First-Time Setup

1. Wire components per [WIRING.md](WIRING.md)
2. Calibrate voltage divider (see below)
3. Update WiFi credentials in source
4. Flash firmware via Arduino IDE
5. Open Serial Monitor at 115200 baud

### Power-On Sequence

1. Place quad **flat and still** on a table
2. Power on — calibration runs automatically (~4 seconds)
3. Two beep sequence confirms calibration complete
4. Connect to WiFi AP: **QuadTelemetry** / **flysafe123** (or your custom credentials)
5. Open browser at `http://192.168.4.1` for live telemetry

### Arming Sequence

```
1. CH5 switch → LOW (disarmed position)
2. Power on → wait for calibration beeps
3. CH6 → LOW (altitude hold off)
4. Throttle stick → minimum (bottom)
5. CH5 switch → HIGH → single arm beep → ARMED
6. CH6 → HIGH to enable altitude hold
```

> **Throttle must be below 20% to arm.** This is a safety interlock — the drone will not arm with throttle raised.

### Disarming

Flip CH5 switch LOW at any time. Motors stop immediately.  
The drone also auto-disarms on:
- Receiver signal loss (500 ms timeout)
- Low voltage cutoff (9.3 V)

---

## Channel Mapping (Flysky FS-i6)

| Channel | Function | Notes |
|---|---|---|
| CH1 | Roll | ±45° setpoint |
| CH2 | Pitch | ±45° setpoint |
| CH3 | Throttle | 0–100% |
| CH4 | Yaw | ±220°/s rate |
| CH5 | Arm/Disarm | 2-position switch |
| CH6 | Altitude Hold | 2-position switch |

---

## Buzzer Alerts

Alerts use a priority system — higher priority alerts always interrupt lower ones:

| Priority | Alert | Pattern |
|---|---|---|
| 2 (highest) | Low voltage cutoff | Continuous beeping — land immediately |
| 2 | Low battery warning | 2 beeps |
| 1 | Receiver failsafe | 4 beeps |
| 0 | Armed | 1 beep |
| 0 | Disarmed | 1 low beep |
| 0 | Altitude hold enabled | 1 beep |

---

## LED Status

| LED | Colour | State |
|---|---|---|
| LED_ARMED | Green | Solid = armed |
| LED_ALTHOLD | Blue | Solid = altitude hold active |
| LED_FAILSAFE | Yellow | Blinking = failsafe active |
| LED_LOWBAT | Red | Blinking = low battery warning / Solid = cutoff |

---

## Voltage Divider Calibration

The battery voltage reading uses a linear calibration: `vbat = (adc / 4095.0 * 3.3 * VDIV_M) + VDIV_C`

To calibrate your specific resistor divider:

1. Measure actual pack voltage with a multimeter — note as `V_actual`
2. Open Serial Monitor and read the raw ADC value printed at startup — note as `adc_raw`
3. Take a second measurement at a different voltage level
4. Solve for `VDIV_M` and `VDIV_C` using the two data points:

```
VDIV_M = (V1 - V2) / ((adc1 / 4095.0 * 3.3) - (adc2 / 4095.0 * 3.3))
VDIV_C = V1 - (adc1 / 4095.0 * 3.3 * VDIV_M)
```

Update the constants in the source:
```cpp
#define VDIV_M  3.672f   // ← your calculated value
#define VDIV_C  1.974f   // ← your calculated value
```

---

## PID Tuning

### Starting Gains (3S)

| Axis | Kp | Ki | Kd |
|---|---|---|---|
| Roll | 2.5 | 0.02 | 0.8 |
| Pitch | 2.5 | 0.02 | 0.8 |
| Yaw Rate | 3.5 | 0.02 | 0.1 |
| Altitude | 2.0 | 0.15 | 1.0 |

For 4S, reduce all gains by ~15–20% (more power per unit throttle).

### Tuning Order

1. **Zero Ki and Kd** on roll/pitch. Increase Kp until fast oscillations appear, then back off 30%
2. **Add Kd** starting at 0.1. Increase until oscillations smooth out
3. **Add Ki last** starting at 0.005. Increase slowly — watch for slow-growing oscillations (windup)
4. **Tune yaw** separately — it is naturally more stable, needs less gain
5. **Tune altitude hold** only after attitude PIDs are solid

### Warning Signs

| Symptom | Cause |
|---|---|
| Fast oscillations | Kp too high |
| Slow growing oscillations | Ki too high (integral windup) |
| Sluggish response | Kp too low |
| High-frequency buzz/vibration | Kd too high, or unbalanced props |
| Steady drift in one axis | Ki too low |

> **Balance your props before tuning.** Unbalanced props corrupt gyro data and make Kd tuning unreliable.

### Motor Minimum Values

Each motor has an individual minimum PWM value to account for manufacturing differences:

```cpp
const float motorMin[4] = {1000, 1040, 1050, 1070};
```

Adjust these so all four motors spin up at the same throttle level from idle. Test without props.

---

## Debug Output

Serial Monitor (115200 baud) prints every 250 ms when armed:

```
Arm:1 Hold:0 FSafe:0 | R:0.3 P:-0.1 H:1.24 Thr:1240 SP:1.22 VBat:11.84 | M1:1045 M2:1067 M3:1058 M4:1071 | dt:5.1 ms Loop: 196.1 Hz
```

| Field | Description |
|---|---|
| Arm | 1 = armed |
| Hold | 1 = altitude hold active |
| FSafe | 1 = failsafe triggered |
| R / P | Roll and pitch angles (degrees) |
| H | Estimated height (metres) |
| Thr | Effective throttle (µs) |
| SP | Altitude setpoint (metres) |
| VBat | Battery voltage |
| M1–M4 | Individual motor PWM outputs (µs) |
| dt | Loop time (ms) |
| Loop | Loop rate (Hz) |

---

## License

MIT License — free to use, modify, and share. See [LICENSE.md](LICENSE.md).