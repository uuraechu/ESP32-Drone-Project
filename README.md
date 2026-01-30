# ESP32-Drone-Project
This ESP32 code for a quadcopter drone (X shape)

### Current Components List – ESP32 Quadcopter Project

#### 1. Core Microcontroller
- **ESP32 Dev Module** (or ESP32-WROOM-32 DevKit)  
  Main flight controller board that runs the code, processes sensors, PID, PWM, etc.

#### 2. Sensors
- **MPU6050** (6-axis IMU – 3-axis accelerometer + 3-axis gyroscope)  
  Provides raw acceleration and angular velocity data for attitude estimation (roll, pitch, yaw rate)

- **BMP280** (barometric pressure and temperature sensor)  
  Used for altitude estimation via atmospheric pressure changes

#### 3. Radio Control System
- **Flysky FS-i6 Transmitter** (Mode 2, 6-channel 2.4 GHz)  
  Pilot controller (sticks + switches)

- **Flysky FS-iA6 Receiver** (6-channel PWM output)  
  Receives signals from transmitter; outputs PWM for roll, pitch, throttle, yaw, arm, and altitude hold toggle

#### 4. Power System
- **3S LiPo Battery** (11.1 V nominal, 12.6 V full charge)  
  Primary power source for motors/ESCs and overall system (e.g., 1300–2200 mAh, 50–75C)

- **5V Buck Converter** (step-down regulator)  
  Converts 9–12.6 V from 3S LiPo to stable 5 V  
  Powers ESP32 VIN pin and Flysky receiver VCC

- **Voltage Divider** (2 resistors)  
  Scales battery voltage for safe ADC reading  
  Recommended: 330 kΩ (top) + 100 kΩ (bottom) → ratio ≈ 0.233

#### 5. Motor & Propulsion
- **4 × ESCs** (Electronic Speed Controllers, 30–45 A)  
  Control brushless motors; accept PWM from ESP32

- **4 × Brushless Motors** (with propellers)  
  Propulsion units (recommended: 2205–2306 size, 2300–2600 KV for 5-inch props on 3S)

- **Propellers** (4 × 5-inch tri-blade or similar)  
  e.g., HQProp 5x4.8x3, Gemfan 51466, etc.

#### 6. Alerts & Feedback
- **Buzzer** (active or passive piezo buzzer)  
  Connected to GPIO 13; used for low battery, failsafe, and cutoff alerts

#### 7. Power Stability Components (Capacitors)
- **0.1 µF ceramic capacitors** (16 V or higher)  
  Quantity: 4–6 pcs  
  Placement: 1 near MPU6050 VCC–GND, 1 near BMP280 VCC–GND, 1 across voltage divider output–GND, extras on 5V rail

- **470 µF electrolytic capacitors** (16 V or 25 V)  
  Quantity: 1–2 pcs  
  Placement: Across 5V buck converter output terminals (parallel with 0.1 µF ceramic)

- **100–220 µF electrolytic capacitor** (16 V or higher) – optional  
  Quantity: 1 pc  
  Placement: Near Flysky receiver VCC–GND

### Total Estimated Component Count (Core Build)
- 1 × ESP32 Dev Module
- 1 × MPU6050
- 1 × BMP280
- 1 × Flysky FS-iA6 Receiver
- 1 × Flysky FS-i6 Transmitter
- 1 × 3S LiPo Battery
- 1 × 5V Buck Converter
- 2 × Resistors (for voltage divider)
- 4 × ESCs
- 4 × Brushless Motors + 4 × Propellers
- 1 × Buzzer
- 5–10 × 0.1 µF ceramic capacitors
- 1–2 × 470 µF electrolytic capacitors
- 1 × 100–220 µF electrolytic capacitor (optional)

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