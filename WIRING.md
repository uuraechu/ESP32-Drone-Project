# ESP32 Quadcopter – Wiring Guide

All sensors use 3.3V logic. The 5V buck converter powers ESP32 VIN and receiver VCC. ESP32's onboard regulator supplies 3.3V to sensors.

### Wiring Table

| Component                  | Component Pin / Wire       | ESP32 Pin / Connection       | Voltage / Notes                                                                 | Capacitor Recommendation (Placement) |
|----------------------------|----------------------------|------------------------------|---------------------------------------------------------------------------------|--------------------------------------|
| **5V Buck Converter**      | Input (+)                  | 3S LiPo positive (+)         | 9–12.6 V input from battery                                                     | —                                    |
| 5V Buck Converter          | Input (–)                  | 3S LiPo negative (–)         | Common ground                                                                   | —                                    |
| 5V Buck Converter          | Output (+)                 | ESP32 **VIN** pin            | 5 V main power input to ESP32                                                   | **One large electrolytic (e.g., 1000 µF, 16–25 V)** + optional 0.1 µF ceramic (parallel, close to buck output terminals) |
| 5V Buck Converter          | Output (+)                 | Flysky receiver **VCC**      | 5 V to receiver (preferred range)                                               | Optional 100–220 µF electrolytic (across VCC–GND near receiver pins) |
| 5V Buck Converter          | Output (–)                 | ESP32 **GND** (any)          | Common ground for entire system                                                 | —                                    |
| **ESP32 Onboard 3.3V**     | 3V3 pin                    | MPU6050 VCC + BMP280 VCC     | Clean 3.3 V from ESP32 regulator to sensors                                     | **0.1 µF ceramic** (across 3V3–GND near each sensor) |
| **MPU6050 (IMU)**          | VCC                        | ESP32 **3V3**                | 3.3 V power                                                                     | **0.1 µF ceramic** (right next to VCC–GND pins on breakout) |
| MPU6050                    | GND                        | ESP32 **GND**                | Ground                                                                          | —                                    |
| MPU6050                    | SDA                        | ESP32 **GPIO 21**            | I²C Data (shared bus)                                                           | —                                    |
| MPU6050                    | SCL                        | ESP32 **GPIO 22**            | I²C Clock (shared bus)                                                          | —                                    |
| **BMP280 (Barometer)**     | VCC                        | ESP32 **3V3**                | 3.3 V power                                                                     | **0.1 µF ceramic** (right next to VCC–GND pins on breakout) |
| BMP280                     | GND                        | ESP32 **GND**                | Ground                                                                          | —                                    |
| BMP280                     | SDA                        | ESP32 **GPIO 21**            | Same I²C bus as MPU6050                                                         | —                                    |
| BMP280                     | SCL                        | ESP32 **GPIO 22**            | Same I²C bus as MPU6050                                                         | —                                    |
| **Flysky FS-iA6 Receiver** | VCC                        | 5V Buck Converter output     | 5 V power                                                                       | Optional 100–220 µF electrolytic (across VCC–GND near receiver) |
| Receiver                   | GND                        | ESP32 **GND**                | Common ground                                                                   | —                                    |
| Receiver CH1 (Roll)        | Signal                     | ESP32 **GPIO 32**            | PWM input – Roll channel                                                        | —                                    |
| Receiver CH2 (Pitch)       | Signal                     | ESP32 **GPIO 33**            | PWM input – Pitch channel                                                       | —                                    |
| Receiver CH3 (Throttle)    | Signal                     | ESP32 **GPIO 34**            | PWM input – Throttle channel                                                    | —                                    |
| Receiver CH4 (Yaw)         | Signal                     | ESP32 **GPIO 35**            | PWM input – Yaw channel                                                         | —                                    |
| Receiver CH5 (Arm)         | Signal                     | ESP32 **GPIO 25**            | PWM input – Arm/disarm switch (controls Green LED)                              | —                                    |
| Receiver CH6 (Aux)         | Signal                     | ESP32 **GPIO 26**            | PWM input – Altitude hold toggle (controls Blue LED)                            | —                                    |
| **ESC1** (front-left)      | Signal                     | GPIO 12                      | PWM output                                                                      | —                                    |
| **ESC2** (front-right)     | Signal                     | GPIO 19                      | PWM output                                                                      | —                                    |
| **ESC3** (rear-right)      | Signal                     | GPIO 27                      | PWM output                                                                      | —                                    |
| **ESC4** (rear-left)       | Signal                     | GPIO 14                      | PWM output                                                                      | —                                    |
| Each ESC                   | GND (signal)               | ESP32 **GND**                | Signal ground                                                                   | —                                    |
| Each ESC                   | Power (+)                  | Direct 3S LiPo (+)           | ESC power from battery (not through buck)                                       | —                                    |
| **Battery Voltage Divider**| Input (+)                  | 3S LiPo positive (+)         | Raw battery voltage                                                             | **0.1 µF ceramic** (across output junction–GND, near GPIO 36) |
| Voltage Divider            | Output (junction)          | ESP32 **GPIO 36**            | Scaled voltage (0–3.3 V max)                                                    | —                                    |
| Voltage Divider            | GND side                   | ESP32 **GND**                | Ground                                                                          | —                                    |
| **Buzzer**                 | Positive                   | ESP32 **GPIO 13**            | Driven by code for alerts                                                       | —                                    |
| Buzzer                     | Negative                   | ESP32 **GND**                | Ground                                                                          | —                                    |

### Optional LED Indicators (Visual Status Feedback)

| LED Color  | Purpose                     | LED + Leg (Anode) → ESP32 Pin | LED – Leg (Cathode) → Resistor → GND | Resistor Value | Notes / Behavior |
|------------|-----------------------------|-------------------------------|---------------------------------------|----------------|------------------|
| Green      | Armed status                | GPIO 15                       | 220–330 Ω → GND                      | 220–330 Ω      | Steady ON when armed (or CH5 high) |
| Red        | Low battery warning         | GPIO 2                        | 220–330 Ω → GND                      | 220–330 Ω      | Fast blink when vbat < VOLT_WARNING; steady ON on cutoff |
| Blue       | Altitude hold active        | GPIO 4                        | 220–330 Ω → GND                      | 220–330 Ω      | Steady ON when altitudeHoldEnabled (or CH6 high) |
| Yellow     | Receiver failsafe           | GPIO 16                       | 220–330 Ω → GND                      | 220–330 Ω      | Fast blink when failsafe active |

**Notes**:
- All sensor signals (MPU6050 & BMP280) are 3.3 V logic → powered from ESP32 **3V3** pin.
- Shared I²C bus: Both sensors connect to **GPIO 21 (SDA)** and **GPIO 22 (SCL)**.
- Ground all GND connections together (LiPo –, buck GND, ESP32 GND, sensors, receiver, ESCs).
- Place capacitors **as close as possible** to the pins they protect.
- One large electrolytic (e.g., 1000 µF) on the 5V buck output is sufficient for bulk filtering; add 0.1 µF ceramics near loads for high-frequency noise.

For full project details → [README.md](README.md)  
For complete component list → [COMPONENTS.md](COMPONENTS.md)  
For troubleshooting → [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
