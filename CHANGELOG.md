# Changelog

All notable changes to this project are documented here.  
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased] — Current

### Added
- `CHANGELOG.md` — this file
- Firmware version constant (`FIRMWARE_VERSION`) in source
- WiFi credential `#define` block in user configuration section

---

## [0.9.0] — Optimisation Pass

### Changed
- `pow(x, 2)` replaced with direct multiply `x * x` in `readSensors()` — removes general power function call
- `sqrt()` replaced with `sqrtf()` in `readSensors()` — avoids float→double→float round-trip
- `cos()` / `sin()` replaced with `cosf()` / `sinf()` in `fuseAttitude()` — uses float trig, no double conversion
- Motor scale factors hoisted to `static const` array in `mixAndWriteMotors()` — eliminates 4 float divides per loop
- ADC-to-voltage conversion collapsed to single precomputed constant `ADC_TO_VBAT` in `checkBatteryVoltage()`
- `GYRO_SCALE * dt` precomputed once per `fuseAttitude()` call instead of twice
- `millis()` cached to single call in `updateLEDs()` — removed 3 redundant syscalls

### Fixed
- `calibrateBarometer()` was using hardcoded `1013.25f` instead of `seaLevel` constant — calibration and flight readings could silently diverge if constant was changed

---

## [0.8.0] — LED & Failsafe Hardening

### Fixed
- `updateLEDs()` low battery blink used `digitalRead()` to track toggle state — replaced with `static bool lbBlinkState` matching the existing failsafe pattern
- `fsBlinkState` now reset to `false` when failsafe clears — prevents stale LED state on recovery
- `lbBlinkState` now reset to `false` when battery recovers — same fix for low battery LED
- `checkReceiverFailsafe()` `timeoutCounter` grew unbounded after failsafe activated — clamped to `TIMEOUT_CONFIRM_LOOPS`
- `goodSignalCounter` similarly clamped to `GOOD_CONFIRM_LOOPS`

---

## [0.7.0] — Buzzer Priority System

### Added
- `priority` field to `BuzzerState` struct (`uint8_t`, 0–2)
- Priority-aware `buzzerAlert()` — new alert only starts if its priority ≥ currently running alert
- Priority-aware `buzzerAlertContinuous()` — same logic
- `buzzerStop()` resets priority to 0 on stop so next alert of any level can start
- `noTone()` called immediately on priority override to cleanly stop current tone

### Changed
- Battery cutoff alert: priority 2 (highest) — guaranteed to override any running alert
- Battery warning alert: priority 2
- Receiver failsafe alert: priority 1
- Arm, disarm, altitude hold toggle alerts: priority 0 — cannot interrupt battery or failsafe alerts
- `updateArmingAndHoldMode()` arm/disarm/hold beeps converted from direct `tone()` calls to `buzzerAlert()` at priority 0 — direct `tone()` calls were bypassing the priority system entirely and could silence active battery warnings

### Fixed
- `buzzerAlertContinuous()` during battery cutoff could be silently skipped if another alert was active — `buzzerStop()` now called first in cutoff path before `buzzerAlertContinuous()`

---

## [0.6.0] — Battery Monitoring Improvements

### Added
- `lowBatCutoff` boolean — latched state, prevents re-arm until voltage recovers
- `lowBatWarning` boolean — separate flag for warning threshold, prevents repeated warning beeps
- Hysteresis on voltage cutoff recovery: cutoff clears at 9.6 V, triggers at 9.3 V
- `buzzerAlertContinuous()` function for continuous beeping on cutoff
- `buzzerStop()` function to force-clear any running buzzer sequence

### Changed
- `canArm` now checks `!lowBatCutoff` — drone cannot be armed while battery is in cutoff state
- `forceDisarm` now triggers on `lowBatCutoff` in addition to arm switch low
- `updateLEDs()` low battery LED: solid on cutoff, blinking on warning, off otherwise

### Removed
- Old `checkBatteryVoltage()` blocking `delay()` loops — replaced with non-blocking state machine in previous version

---

## [0.5.0] — Failsafe Debouncing

### Added
- `timeoutCounter` — failsafe requires 5 consecutive loop iterations of signal loss before triggering (~25 ms at 200 Hz)
- `goodSignalCounter` — signal recovery requires 10 consecutive good loops before clearing failsafe (~50 ms)
- Prevents single-frame glitches from triggering full failsafe disarm

### Changed
- Failsafe buzzer now uses `buzzerAlert()` state machine instead of blocking beep sequence

---

## [0.4.0] — WiFi Core Pinning & Mutex

### Added
- `dataMutex` — FreeRTOS mutex protecting all shared variables accessed from both cores
- WiFi task pinned to Core 0 via `xTaskCreatePinnedToCore()` with 8192-byte stack
- Flight loop remains on Core 1

### Fixed
- `handleData()` was calling `readSensors()` mid-loop — caused double sensor reads and I2C bus contention with Core 1
- `temperature` now read exclusively on Core 1 in `readSensors()` — eliminates I2C bus conflict
- `global_dt` moved inside mutex — was written on Core 1 outside mutex, read inside mutex on Core 0 (data race)
- PID setpoints (`rollSetpoint`, `pitchSetpoint`, `yawRateSetpoint`, `altSetpoint`) moved inside mutex — consistent with PID compute
- Altitude tracking `altSetpoint` update moved inside mutex

### Changed
- `handleData()` now snapshots all shared variables inside a single mutex lock then releases before JSON formatting
- `snprintf()` with 1024-byte stack buffer replaces `String` concatenation in `handleData()` — eliminates heap fragmentation
- Buffer overflow check added: warns to Serial if JSON output is truncated

---

## [0.3.0] — Timing & Sensor Fixes

### Fixed
- `global_dt` was measured before `readSensors()` but used after — sensor I2C blocking time (~800 µs) was excluded from dt, corrupting PID derivative term. `global_dt` now recaptured after `readSensors()` using `now2`
- `prevTime` updated to `now2` (post-sensor timestamp) for consistency — `prevTime = now` was leaving a systematic timing error
- `prevTime` set to `now` when disarmed — prevents massive dt spike on first armed loop iteration after disarm period
- Altitude feedforward was using wrong throttle centre (1500) instead of 500 — corrected

### Changed
- I2C clock set to 400 kHz (`Wire.setClock(400000)`) — reduces MPU6050 + BMP280 read time from ~3200 µs to ~800 µs
- BMP280 standby time corrected to `STANDBY_MS_63` — was `STANDBY_MS_62_5` which is not a valid enum value
- MPU6050 filter bandwidth increased from 21 Hz to 44 Hz
- PID sample time explicitly set: `SetSampleTime(LOOP_INTERVAL_MS)` on all four PIDs
- `LOOP_INTERVAL_MS` global constant added — single source of truth for 5 ms loop target
- `setup()` uses single `millis()` capture for both `lastValidSignalTime` and `prevTime` — removes race between two separate calls

---

## [0.2.0] — Non-Blocking Refactor

### Added
- `BuzzerState` struct — non-blocking buzzer state machine
- `updateBuzzer()` — called every loop, drives beep sequences without blocking
- `buzzerAlert()` — triggers a beep sequence and returns immediately
- Non-blocking battery sampling — 16-sample ADC average spread across loop iterations, one sample per 2 ms
- Hardware ADC oversampling: 4 samples averaged per iteration

### Fixed
- `buzzerAlert()` was blocking up to 550 ms during flight via `delay()` loops
- `checkBatteryVoltage()` was blocking 32 ms via delay loop between ADC samples

### Changed
- `stopMotors()` no longer calls `noTone()` — buzzer state machine manages tone independently

---

## [0.1.0] — Initial Release

### Added
- ESP32 flight controller core
- MPU6050 gyro + accelerometer reading with calibration
- BMP280 barometer reading with calibration
- Complementary filter attitude fusion (roll, pitch)
- Complementary filter altitude + velocity fusion
- Four PID controllers: roll, pitch, yaw rate, altitude
- X-configuration motor mixing
- Per-motor minimum PWM calibration array
- Altitude hold mode with integral zeroing on enable, feedforward on enable
- Flysky FS-iA6 PWM receiver input (6 channels)
- Battery voltage monitoring via ADC with linear calibration
- Receiver failsafe with auto-disarm
- Arming logic: CH5 switch + throttle safety interlock
- Buzzer alerts for arm, disarm, calibration complete, low battery, failsafe
- Status LEDs: armed, altitude hold, failsafe, low battery
- WiFi AP telemetry server (HTTP, port 80)
- Live JSON data endpoint at `/data`
- Browser dashboard at `/`
- Serial debug output every 250 ms
- Centripetal acceleration correction for IMU offset from centre of rotation