# ESP32 Quadcopter Flight Controller

Custom lightweight flight controller built on ESP32 Dev Module for a 5-inch quadcopter.  
Uses MPU6050 + BMP280 sensor fusion, Flysky FS-iA6 receiver, PID control, altitude hold, failsafe, low-voltage cutoff, buzzer alerts, and optional status LEDs.

Includes live telemetry over WiFi (HTTP server – view in browser on phone/laptop).

## Features

- Complementary filtering for roll/pitch (gyro + accel)
- Complementary filtering for altitude & velocity (accel Z + baro)
- Altitude hold mode (toggled via CH6)
- PID for roll, pitch, yaw rate, altitude
- Flysky 6-channel PWM input
- Battery monitoring + cutoff/warning (linear calibration)
- Receiver failsafe (auto-disarm on signal loss)
- Buzzer alerts (low battery, failsafe, cutoff)
- One-time power-on calibration (gyro, accel, baro)
- Optional status LEDs (armed, low battery, alt hold, failsafe)
- Live telemetry via WiFi AP (browser dashboard)

## Hardware Overview

- Frame: 5-inch (target ≤ 200 g printed in PETG HF)
- Motors: 2300 KV (2205–2306 size)
- Props: 5-inch 2-blade (e.g. 5×4.5×2)
- Battery: 3S 2200 mAh 50C
- Power: 5V buck converter (≥2 A recommended)
- Receiver: Flysky FS-iA6 (PWM)

Full component list → [COMPONENTS.md](COMPONENTS.md)  
Detailed wiring → [WIRING.md](WIRING.md)  
Troubleshooting → [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

## Setup & Usage

1. Place quad flat and still on table.
2. Power on → wait ~4 seconds for auto calibration.
3. Bind FS-i6 transmitter.
4. Connect to WiFi AP: **QuadTelemetry** / password **flysafe123**
5. Open browser at IP shown in Serial Monitor (usually 192.168.4.1) for live telemetry.
6. Arm (CH5 high + throttle low) → test hover/altitude hold (CH6 toggle).

**Important**: Always test **without props** first!

## Software Requirements

- Arduino IDE + ESP32 core 3.x
- Libraries (via Library Manager):
  - Adafruit MPU6050
  - Adafruit BMP280
  - Adafruit Unified Sensor
  - PID_v1_bc (or PID_v1)
  - WiFi.h & WebServer.h (built-in)

## Safety & Tuning

- Start with very low PID gains — increase slowly while tethered.
- Calibrate voltage divider (VDIV_M & VDIV_C) using multimeter.
- Use 2-blade props for smooth, longer flights.
- Remove props during bench testing.
- Hard cutoff at 3.1 V/cell (9.3 V pack).

## License

MIT License — feel free to use, modify, share.