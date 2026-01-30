# ESP32-Drone-Project
This ESP32 code for a quadcopter drone (X shape)

The list of components used are:
1. ESP32 Dev Module
2. MPU6050 IMU
3. BMP280 barometer
4. Flysky FS-iA6 receiver (with FS-i6 transmitter)
5. 3S LiPo battery
6. 5V buck converter
7. Voltage divider (2 resistors)
8. ESC (x4)
9. Brushless motors (with props) (x4)
10. Active/passive buzzer
11. LEDs
12. Drone Frame


### ESP32 Quadcopter Wiring Table (5V Buck Converter Setup)

| Component                  | Component Pin / Wire       | ESP32 Pin / Connection       | Voltage / Notes                                                                 | Capacitor Recommendation (Placement) |
|----------------------------|----------------------------|------------------------------|---------------------------------------------------------------------------------|--------------------------------------|
| **5V Buck Converter**      | Input (+)                  | 3S LiPo positive (+)         | 9–12.6 V input from battery                                                     | —                                    |
| 5V Buck Converter          | Input (–)                  | 3S LiPo negative (–)         | Common ground                                                                   | —                                    |
| 5V Buck Converter          | Output (+)                 | ESP32 **VIN** pin            | 5 V main power input to ESP32                                                   | **470 µF electrolytic + 0.1 µF ceramic** (parallel, close to buck output terminals) |
| 5V Buck Converter          | Output (+)                 | Flysky receiver **VCC**      | 5 V to receiver (preferred range)                                               | **100–220 µF electrolytic** (across VCC–GND near receiver pins) |
| 5V Buck Converter          | Output (–)                 | ESP32 **GND** (any)          | Common ground for entire system                                                 | —                                    |
| **ESP32 Onboard 3.3V**     | 3V3 pin                    | MPU6050 VCC + BMP280 VCC     | Clean 3.3 V from ESP32 regulator to sensors                                     | **0.1 µF ceramic** (across 3V3–GND near each sensor) |
| **MPU6050 (IMU)**          | VCC                        | ESP32 **3V3**                | 3.3 V power                                                                     | **0.1 µF ceramic** (right next to VCC–GND pins on MPU6050 breakout) |
| MPU6050                    | GND                        | ESP32 **GND**                | Ground                                                                          | —                                    |
| MPU6050                    | SDA                        | ESP32 **GPIO 21**            | I²C Data (shared bus)                                                           | —                                    |
| MPU6050                    | SCL                        | ESP32 **GPIO 22**            | I²C Clock (shared bus)                                                          | —                                    |
| **BMP280 (Barometer)**     | VCC                        | ESP32 **3V3**                | 3.3 V power                                                                     | **0.1 µF ceramic** (right next to VCC–GND pins on BMP280 breakout) |
| BMP280                     | GND                        | ESP32 **GND**                | Ground                                                                          | —                                    |
| BMP280                     | SDA                        | ESP32 **GPIO 21**            | Same I²C bus as MPU6050                                                         | —                                    |
| BMP280                     | SCL                        | ESP32 **GPIO 22**            | Same I²C bus as MPU6050                                                         | —                                    |
| **Flysky FS-iA6 Receiver** | VCC                        | 5V Buck Converter output     | 5 V power                                                                       | **100–220 µF electrolytic** (across VCC–GND near receiver) |
| Receiver                   | GND                        | ESP32 **GND**                | Common ground                                                                   | —                                    |
| Receiver CH1 (Roll)        | Signal                     | ESP32 **GPIO 32**            | PWM input – Roll channel                                                        | —                                    |
| Receiver CH2 (Pitch)       | Signal                     | ESP32 **GPIO 33**            | PWM input – Pitch channel                                                       | —                                    |
| Receiver CH3 (Throttle)    | Signal                     | ESP32 **GPIO 34**            | PWM input – Throttle channel                                                    | —                                    |
| Receiver CH4 (Yaw)         | Signal                     | ESP32 **GPIO 35**            | PWM input – Yaw channel                                                         | —                                    |
| Receiver CH5 (Arm)         | Signal                     | ESP32 **GPIO 25**            | PWM input – Arm/disarm switch                                                   | —                                    |
| Receiver CH6 (Aux)         | Signal                     | ESP32 **GPIO 26**            | PWM input – Altitude hold toggle                                                | —                                    |
| **ESCs (4×)**              | Signal                     | GPIO 25, 26, 27, 14          | PWM outputs to ESCs                                                             | —                                    |
| Each ESC                   | GND (signal)               | ESP32 **GND**                | Signal ground                                                                   | —                                    |
| Each ESC                   | Power (+)                  | Direct 3S LiPo (+)           | ESC power from battery (not through buck)                                       | —                                    |
| **Battery Voltage Divider**| Input (+)                  | 3S LiPo positive (+)         | Raw battery voltage                                                             | **0.1 µF ceramic** (across output junction–GND, near GPIO 34) |
| Voltage Divider            | Output (junction)          | ESP32 **GPIO 34**            | Scaled voltage (0–3.3 V max)                                                    | —                                    |
| Voltage Divider            | GND side                   | ESP32 **GND**                | Ground                                                                          | —                                    |
| **Buzzer**                 | Positive                   | ESP32 **GPIO 13**            | Driven by code for alerts                                                       | —                                    |
| Buzzer                     | Negative                   | ESP32 **GND**                | Ground                                                                          | —                                    |

### Notes
- All sensor signals (MPU6050 & BMP280) are 3.3 V logic → powered from ESP32 **3V3** pin.
- Shared I²C bus: Both sensors connect to **GPIO 21 (SDA)** and **GPIO 22 (SCL)**.
- Capacitors are critical for stability — place them as close as possible to the pins they protect.
- Ground all GND connections together (LiPo –, buck GND, ESP32 GND, sensors, receiver, ESCs).