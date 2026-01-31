# ESP32 Quadcopter – Components List

### Core Electronics
- **ESP32 Dev Module** (or ESP32-WROOM-32 DevKit)  
  Main flight controller board

- **MPU6050** (6-axis IMU – accel + gyro)  
  Attitude estimation (roll/pitch/yaw rate)

- **BMP280** (barometric pressure/temperature sensor)  
  Altitude estimation

- **Flysky FS-iA6 Receiver** (6-channel PWM output)  
  Receives signals from FS-i6 transmitter

- **Flysky FS-i6 Transmitter** (6-channel 2.4 GHz, Mode 2)  
  Pilot input controller

### Power System
- **3S LiPo Battery** (11.1 V nominal, e.g., 2200mAh 50C)  
  Main power source

- **5V Buck Converter** (step-down from 3S, ≥1–2 A output)  
  Powers ESP32 VIN and receiver

- **Voltage Divider** (2 resistors: 330 kΩ top + 100 kΩ bottom recommended)  
  Scales battery voltage for ESP32 ADC

### Motor & Propulsion
- **4 × ESCs** (30A continuous / 40A peak recommended)  
  Control brushless motors

- **4 × Brushless Motors** (2300KV, 2205–2306 size recommended)  
  Propulsion (5-inch prop compatible)

- **4 × 2-Blade Propellers** (5-inch recommended)  
  e.g., 5×4.5×2, 5×5×2, HQProp Durable, Gemfan 5045 or 5140 2-blade  
  → Chosen for smoother power delivery, longer flight time, and beginner-friendly control

### Alerts
- **Buzzer** (active or passive piezo)  
  Alerts for low battery, failsafe, cutoff

### Power Stability (Capacitors)
- **0.1 µF ceramic capacitors** (16 V or higher)  
  Quantity: 4–6 pcs  
  For sensors, voltage divider, and buck output noise filtering

- **470 µF electrolytic capacitors** (16–25 V)  
  Quantity: 1–2 pcs  
  Bulk smoothing on 5V buck output

- **100–220 µF electrolytic capacitor** (16 V or higher) – optional  
  Quantity: 1 pc  
  Near receiver VCC–GND for PWM stability

### Optional / Future Add-ons
- Extra capacitors for noise filtering
- LED indicators (arm status, low battery)
- Current sensor
- GPS module
- SD card for logging

This is everything currently used or required by the project code and wiring.

For full project details → [README.md](README.md)  
For wiring → [WIRING.md](WIRING.md)  
For troubleshooting → [TROUBLESHOOTING.md](TROUBLESHOOTING.md)