# ESP32 Quadcopter – Wiring Guide

All sensors use 3.3V logic. 5V buck powers ESP32 VIN and receiver. ESP32 3V3 powers sensors.

### Wiring Table

| Component                  | Pin / Wire                 | ESP32 Pin / Connection       | Voltage / Notes                                                                 | Capacitor Recommendation |
|----------------------------|----------------------------|------------------------------|---------------------------------------------------------------------------------|--------------------------|
| **5V Buck Converter**      | Input (+)                  | 3S LiPo (+)                  | 9–12.6 V input                                                                  | —                        |
| 5V Buck Converter          | Input (–)                  | 3S LiPo (–)                  | Common ground                                                                   | —                        |
| 5V Buck Converter          | Output (+)                 | ESP32 **VIN**                | 5 V main power                                                                  | **One large electrolytic** (e.g. 1000 µF, 16–25 V) + optional 0.1 µF ceramic (parallel, close to buck output) |
| 5V Buck Converter          | Output (+)                 | Flysky receiver **VCC**      | 5 V to receiver (preferred)                                                     | Optional 100–220 µF electrolytic near receiver VCC–GND |
| 5V Buck Converter          | Output (–)                 | ESP32 **GND**                | Common ground                                                                   | —                        |
| **ESP32 Onboard 3.3V**     | 3V3 pin                    | MPU6050 VCC + BMP280 VCC     | 3.3 V to sensors                                                                | **0.1 µF ceramic** near each sensor VCC–GND |
| **MPU6050**                | VCC                        | ESP32 **3V3**                | 3.3 V power                                                                     | 0.1 µF ceramic near pins |
| MPU6050                    | GND                        | ESP32 **GND**                | Ground                                                                          | —                        |
| MPU6050                    | SDA                        | ESP32 **GPIO 21**            | I²C Data                                                                        | —                        |
| MPU6050                    | SCL                        | ESP32 **GPIO 22**            | I²C Clock                                                                       | —                        |
| **BMP280**                 | VCC                        | ESP32 **3V3**                | 3.3 V power                                                                     | 0.1 µF ceramic near pins |
| BMP280                     | GND                        | ESP32 **GND**                | Ground                                                                          | —                        |
| BMP280                     | SDA                        | ESP32 **GPIO 21**            | Shared I²C                                                                      | —                        |
| BMP280                     | SCL                        | ESP32 **GPIO 22**            | Shared I²C                                                                      | —                        |
| **Flysky FS-iA6 Receiver** | VCC                        | 5V Buck output               | 5 V power                                                                       | Optional 100–220 µF electrolytic near VCC–GND |
| Receiver                   | GND                        | ESP32 **GND**                | Common ground                                                                   | —                        |
| Receiver CH1 (Roll)        | Signal                     | GPIO 32                      | PWM input                                                                       | —                        |
| Receiver CH2 (Pitch)       | Signal                     | GPIO 33                      | PWM input                                                                       | —                        |
| Receiver CH3 (Throttle)    | Signal                     | GPIO 34                      | PWM input                                                                       | —                        |
| Receiver CH4 (Yaw)         | Signal                     | GPIO 35                      | PWM input                                                                       | —                        |
| Receiver CH5 (Arm)         | Signal                     | GPIO 25                      | PWM input – controls Green LED                                                  | —                        |
| Receiver CH6 (Aux)         | Signal                     | GPIO 26                      | PWM input – controls Blue LED                                                   | —                        |
| **ESCs (4×)**              | Signal                     | GPIO 25,26,27,14             | PWM outputs                                                                     | —                        |
| Each ESC                   | GND (signal)               | ESP32 **GND**                | Signal ground                                                                   | —                        |
| Each ESC                   | Power (+)                  | Direct 3S LiPo (+)           | ESC power from battery                                                          | —                        |
| **Battery Voltage Divider**| Input (+)                  | 3S LiPo (+)                  | Raw battery voltage                                                             | **0.1 µF ceramic** across output–GND near GPIO 36 |
| Voltage Divider            | Output (junction)          | ESP32 **GPIO 36**            | Scaled voltage (0–3.3 V)                                                        | —                        |
| Voltage Divider            | GND side                   | ESP32 **GND**                | Ground                                                                          | —                        |
| **Buzzer**                 | Positive                   | ESP32 **GPIO 13**            | Alerts                                                                          | —                        |
| Buzzer                     | Negative                   | ESP32 **GND**                | Ground                                                                          | —                        |

### Optional LED Indicators

| LED Color  | Purpose                     | ESP32 Pin | Resistor (220–330 Ω) | Behavior |
|------------|-----------------------------|-----------|----------------------|----------|
| Green      | Armed status                | GPIO 15   | Yes → GND            | Steady ON when armed (or CH5 high) |
| Red        | Low battery / cutoff        | GPIO 2    | Yes → GND            | Blink on warning, steady on cutoff |
| Blue       | Altitude hold active        | GPIO 4    | Yes → GND            | Steady ON when active (or CH6 high) |
| Yellow     | Receiver failsafe           | GPIO 16   | Yes → GND            | Fast blink on failsafe |

**Notes**:
- Shared I²C bus: MPU6050 & BMP280 on GPIO 21 (SDA) + 22 (SCL)
- Ground all GND together
- One large electrolytic (e.g. 1000 µF) on 5V buck output is sufficient for bulk filtering
- Add 0.1 µF ceramics near loads (ESP32 VIN–GND, receiver VCC–GND, sensors) for high-frequency noise
- Place capacitors as close as possible to pins

For full project → [README.md](README.md)  
For components → [COMPONENTS.md](COMPONENTS.md)  
For troubleshooting → [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
