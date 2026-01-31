# ESP32 Quadcopter Flight Controller

A custom, lightweight flight controller for a 5-inch (or similar) quadcopter using an **ESP32 Dev Module**, MPU6050 IMU, BMP280 barometer, Flysky FS-iA6 receiver, and complementary filtering + PID control for attitude and altitude hold.

Built for hobbyist learning and experimentation — includes sensor calibration, receiver failsafe, battery monitoring with low-voltage cutoff, and buzzer alerts.

## Features

- Complementary filtering for roll/pitch (gyro + accel fusion)
- Complementary filtering for altitude & vertical velocity (accel Z + barometer)
- Altitude hold mode toggled via CH6 switch
- PID control for roll, pitch, yaw rate, and altitude
- Flysky FS-iA6 6-channel PWM receiver input
- Battery voltage monitoring + low-voltage cutoff & warning
- Receiver signal loss failsafe (auto-disarm)
- Buzzer alerts for low battery, failsafe, and cutoff
- One-time power-on calibration (gyro, accel, barometer offset)
- 5V buck converter power from 3S LiPo
- Arduino-ESP32 core 3.x compatible (LEDC PWM)

## Hardware Overview

- **Frame / Propulsion**: 5-inch wheelbase (210–220 mm) recommended  
  - 4 × 2300KV brushless motors (2205–2306 size)  
  - 4 × 5-inch **2-blade propellers** (e.g., 5×4.5×2 or 5×5×2 – chosen for smoother power, longer flight time, and beginner-friendly control)  
  - 4 × 30A continuous / 40A peak ESCs

- **Power**: 3S LiPo (e.g., 2200mAh 50C) + 5V buck converter

Full component list → [COMPONENTS.md](COMPONENTS.md)  
Detailed wiring diagram → [WIRING.md](WIRING.md)  
Troubleshooting guide → [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

## Setup & Usage

1. Place quad **flat and still** on a table.
2. Power on → wait ~4 seconds for automatic calibration (gyro, accel, baro offset).
3. Bind Flysky transmitter to receiver.
4. Connect via USB → open Serial Monitor (115200 baud) to see calibration results.
5. Arm (CH5 high + throttle low) → test hover/altitude hold (CH6 toggle).

**Important**: Test **without props** first!

## Software Requirements

- **Arduino IDE** with ESP32 core 3.x
- Libraries (install via Library Manager):
  - Adafruit MPU6050
  - Adafruit BMP280
  - Adafruit Unified Sensor
  - PID_v1 (or PID_v1_bc for anti-windup)

## Safety & Tuning

- Start with very low PID gains — increase slowly while tethered.
- Calibrate voltage divider ratio (`VDIV_RATIO`) using a multimeter.
- Use 2-blade props for smoother power delivery & longer flights (3-blade optional for more punch).
- Always remove props during bench testing.
- Monitor battery voltage — 3.1 V/cell cutoff is hard-enforced in code.

## License

MIT License — feel free to use, modify, and share.