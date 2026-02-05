# ESP32 Quadcopter – Components List

### Core Electronics
- **ESP32 Dev Module** (or ESP32-WROOM-32 DevKit)  
  Main flight controller board (runs code, WiFi AP, HTTP telemetry server)

- **MPU6050** (6-axis IMU – accelerometer + gyroscope)  
  Attitude estimation (roll/pitch/yaw rate)

- **BMP280** (barometric pressure/temperature sensor)  
  Altitude estimation

- **Flysky FS-iA6 Receiver** (6-channel PWM output)  
  Receives signals from FS-i6 transmitter

- **Flysky FS-i6 Transmitter** (6-channel 2.4 GHz, Mode 2)  
  Pilot input controller

### Power System
- **3S LiPo Battery** (11.1 V nominal, e.g., 2200 mAh 50C)  
  Main power source

- **5V Buck Converter** (step-down from 3S, ≥2 A continuous recommended)  
  Powers ESP32 VIN and receiver VCC

- **Voltage Divider** (2 resistors: e.g. 330 kΩ top + 100 kΩ bottom)  
  Scales battery voltage for ESP32 ADC (GPIO 36)

### Motor & Propulsion
- **4 × ESCs** (30A continuous / 40A peak recommended)  
  Control brushless motors

- **4 × Brushless Motors** (2300 KV, 2205–2306 size recommended)  
  Propulsion (5-inch prop compatible)

- **4 × 2-Blade Propellers** (5-inch recommended)  
  e.g., 5×4.5×2, 5×5×2, HQProp Durable, Gemfan 5045 or 5140 2-blade  
  → Chosen for smoother power delivery, longer flight time, beginner-friendly control

### Alerts & Feedback
- **Buzzer** (active or passive piezo)  
  Alerts for low battery, failsafe, cutoff

- **4 × LEDs** (optional visual indicators)
  - Green — Armed status (steady ON when CH5 high or armed)
  - Red — Low battery warning (blink) or cutoff (steady ON)
  - Blue — Altitude hold active (steady ON when CH6 high or enabled)
  - Yellow — Receiver failsafe (blink when signal lost)

- **4 × 220–330 Ω resistors** (1/4 W)  
  Current limiting for LEDs

### Power Stability (Capacitors)
- **0.1 µF ceramic capacitors** (16 V or higher)  
  Quantity: 4–6 pcs  
  For sensors (MPU6050 & BMP280), voltage divider output (near GPIO 36), optional at ESP32 VIN and receiver VCC–GND

- **One large electrolytic capacitor** (16–25 V, e.g., 1000 µF)  
  Quantity: 1 pc  
  Bulk smoothing across 5V buck converter output (main rail capacitor)

- **100–220 µF electrolytic capacitor** (16 V or higher) – optional  
  Quantity: 1 pc  
  Near receiver VCC–GND for PWM stability

### Optional / Future Add-ons
- Extra capacitors for additional noise filtering
- FPV camera module (e.g., ESP32-CAM with OV2640 or OV5640 upgrade) — for custom video streaming
- External antenna for ESP32-CAM (for better WiFi range)
- Current sensor — for power monitoring
- GPS module — for position hold
- SD card module — for blackbox logging

This is everything currently used or planned in the project code and wiring.

For full project overview → [README.md](README.md)  
For detailed wiring → [WIRING.md](WIRING.md)  
For troubleshooting → [TROUBLESHOOTING.md](TROUBLESHOOTING.md)