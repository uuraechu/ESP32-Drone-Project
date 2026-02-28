# Changelog

All notable changes to this project are documented here.  
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased] — Current

### Fixed
- **Roll mixer sign inversion** — `rollOutput` signs on M1/M4 and M2/M3 were swapped,
  causing the drone to amplify roll errors instead of correcting them. Drone flipped
  immediately on throttle-up. Corrected:
  `M1 FL: throttle - rollOutput`, `M2 FR: throttle + rollOutput`,
  `M3 RR: throttle + rollOutput`, `M4 RL: throttle - rollOutput`

### Added
- `reEstimateAttitude()` — called at every disarm event (normal disarm, failsafe,
  battery cutoff). Takes 20-sample accelerometer average and re-seeds the complementary
  filter with measured pitch and roll. Skipped if tilt exceeds 60° to avoid seeding a
  crash angle into the filter on the next arm.

---

## [1.2.0] — Orientation Check, Hard Offsets, and Trim

### Added
- `checkBootOrientation()` — reads 100 accelerometer samples at boot and classifies
  orientation into four states: LEVEL (≤15°), TILTED (15–60°), ON_SIDE (>60°),
  INVERTED (accelZ negative). Distinct buzzer pattern for each state.
- Filter seeding at boot — `checkBootOrientation()` output angles assigned directly to
  `pitch` and `roll`, eliminating the ~1.25s complementary filter convergence delay.
- Runtime orientation re-check in `updateArmingAndHoldMode()` — re-checks every 100ms
  until cleared. Allows correction after an off-level boot without requiring reboot.
  Re-seeds filter and emits two high beeps when orientation clears. Arming blocked until
  `orientationClearToFly = true`.
- `PITCH_TRIM` / `ROLL_TRIM` `#define` constants — added to setpoints in `loop()` to
  compensate for fixed mechanical offsets (e.g. forward battery position). Set to 1.8°
  pitch to match measured resting offset.
- Separate calibration sketch (`Calibration/`) — standalone Arduino program that runs
  3 × 500-sample measurement passes, validates consistency across runs (gyro deviation
  < 0.002 rad/s, accel < 0.05 m/s²), and prints a ready-to-paste `#define` block.
- `BootOrientation` enum — LEVEL, TILTED, ON_SIDE, INVERTED.
- `volatile bool orientationClearToFly` global flag.

### Changed
- Hard IMU offsets replace runtime calibration — `calibrateAllSensors()` now applies
  fixed `#define` constants (GYRO_OFFSET_X/Y/Z, ACCEL_OFFSET_X/Y/Z) instead of
  sampling the IMU at every boot. Eliminates silent calibration corruption if the drone
  moves during startup. Boot time reduced by ~2 seconds.
- Barometer calibration retained as runtime — atmospheric pressure varies with weather
  and location so `calibrateBarometer()` still runs on every boot.
- Orientation re-check reduced to 5 samples (was 20) — 20 samples at 400kHz I2C
  (~6.3ms) exceeded the 5ms loop period. 5 samples (~1.6ms) acceptable for a
  rate-limited check.
- Forward declarations block updated — `calibrateBarometer()`, `checkBootOrientation()`,
  and `reEstimateAttitude()` added.

### Removed
- `calibrateMPU6050()` — dead code after switch to hard offsets, never called.

### Fixed
- `goto` over variable declarations in `updateArmingAndHoldMode()` — `goto` jumping over
  `const int numSamples` and `float sum*` initializations is illegal in C++ and causes a
  compile error. Replaced with a nested `if (now_oc - lastOrientCheck >= 100)` block.

---

## [1.1.0] — Data Race Fixes and RC Spike Rejection

### Added
- RC spike rejection in `RCTask` — `pulseIn` on ESP32 is vulnerable to WiFi hardware
  ISR corruption producing occasional wildly incorrect pulse timings. Stick channels
  (roll, pitch, yaw, throttle) reject any reading that changes by more than 300µs per
  frame. Switch channels (ARM, AUX) use range-check only — they legitimately flip the
  full range (~500–1000µs) in one frame and would be permanently locked by a threshold.
- `volatile` on race-prone globals — variables written on Core 1 outside `dataMutex`
  but read inside `handleData()` on Core 0 declared `volatile` to prevent compiler
  reordering across the mutex acquire barrier: `vbat`, `temperature`, `armed`,
  `altitudeHoldEnabled`, `receiverFailsafeActive`, `lowBatCutoff`, `lowBatWarning`,
  `m1`–`m4`.
- `failsafe`, `lowBatCutoff`, `lowBatWarning` fields in telemetry JSON — previously
  missing. Added to `dataMutex` snapshot in `handleData()` and `snprintf` format string.

### Fixed
- Dashboard failsafe LED always off — JavaScript condition `d.armed === false && !d.armed`
  was redundant and never correctly reflected failsafe state. Fixed to use `d.failsafe`,
  `d.lowBatWarning`, `d.lowBatCutoff` directly from JSON.
- `yawRate` in JSON was raw rad/s — dashboard yaw slider showed ~1.7% deflection at
  full stick (±3.84 rad/s displayed against ±220 range). Now multiplied by `GYRO_SCALE`
  before output, consistent with all other rotational fields.
- ARM/AUX channels locked by spike filter — 300µs threshold applied to switch channels
  prevented them from ever registering a flip. Separated into stick-only threshold;
  switch channels use range-check only.

### Changed
- `handleData()` RC snapshot reads from `sharedPwm*` under `rcMutex` — previously read
  local `pwm*` variables which are written by `readReceiver()` on Core 1 outside any
  mutex, creating a data race visible to the HTTP server on Core 0.

---

## [1.0.0] — Dual-Core Architecture and HTTP Telemetry Dashboard

### Added
- `RCTask` on Core 0 (priority 2) — `pulseIn` for all 6 RC channels moved off Core 1
  flight loop. Throttle read first so `sharedLastValidSignal` is updated immediately,
  reducing failsafe detection latency from ~26ms to ~1.5ms typical.
- `WiFiTask` on Core 0 (priority 1) — `server.handleClient()` runs during the
  `vTaskDelay(1)` windows between channel reads (~23% CPU share).
- `rcMutex` — FreeRTOS mutex protecting `sharedPwm*` variables written by `RCTask` and
  read by flight loop and `handleData()`.
- HTTP browser dashboard — dark-theme single-page app with: RC input sliders (bipolar
  for sticks, unipolar for switches), attitude bars, motor bars with colour-coded load,
  status LEDs (armed, alt hold, failsafe, low battery, cutoff), battery/altitude
  setpoint/temperature gauges, raw JSON viewer.
- `/debug/on` and `/debug/off` HTTP endpoints — runtime toggle for 250ms serial debug
  output without reflashing. Controlled by `volatile bool debugOutputEnabled`.
- `rcMutex` snapshot in `handleData()` — separate from `dataMutex` snapshot, prevents
  race on RC channel data served to dashboard.
- JSON buffer overflow detection — `snprintf` return value checked; warning printed to
  Serial if output truncated.

### Changed
- `prevTime = now` moved to loop guard — was set after sensor reads, causing `elapsed`
  to undercount by ~1.4ms (I2C read time) each iteration, corrupting `global_dt`.
- `global_dt = elapsed / 1000.0f` computed inside `dataMutex` — consistent with PID
  compute which also runs inside mutex.
- Angle limit tightened to 25° in altitude hold mode — preserves thrust headroom and
  altitude estimate reliability (`cos(25°) = 0.906` vs `cos(45°) = 0.707`).

### Fixed
- 180ms `delay()` in RC channel reading blocked flight loop entirely between reads —
  eliminated by moving `pulseIn` to dedicated `RCTask`.

---

## [0.9.0] — Optimisation Pass

### Changed
- `pow(x, 2)` replaced with direct multiply `x * x` in `readSensors()` — removes
  general power function call
- `sqrt()` replaced with `sqrtf()` in `readSensors()` — avoids float→double→float
  round-trip
- `cos()` / `sin()` replaced with `cosf()` / `sinf()` in `fuseAttitude()` — uses float
  trig, no double conversion
- Motor scale factors hoisted to `static const` array in `mixAndWriteMotors()` —
  eliminates 4 float divides per loop
- ADC-to-voltage conversion collapsed to single precomputed constant `ADC_TO_VBAT` in
  `checkBatteryVoltage()`
- `GYRO_SCALE * dt` precomputed once per `fuseAttitude()` call instead of twice
- `millis()` cached to single call in `updateLEDs()` — removed 3 redundant syscalls

### Fixed
- `calibrateBarometer()` was using hardcoded `1013.25f` instead of `seaLevel` constant
  — calibration and flight readings could silently diverge if constant was changed

---

## [0.8.0] — LED & Failsafe Hardening

### Fixed
- `updateLEDs()` low battery blink used `digitalRead()` to track toggle state —
  replaced with `static bool lbBlinkState` matching the existing failsafe pattern
- `fsBlinkState` now reset to `false` when failsafe clears — prevents stale LED state
  on recovery
- `lbBlinkState` now reset to `false` when battery recovers — same fix for low battery
  LED
- `checkReceiverFailsafe()` `timeoutCounter` grew unbounded after failsafe activated —
  clamped to `TIMEOUT_CONFIRM_LOOPS`
- `goodSignalCounter` similarly clamped to `GOOD_CONFIRM_LOOPS`

---

## [0.7.0] — Buzzer Priority System

### Added
- `priority` field to `BuzzerState` struct (`uint8_t`, 0–2)
- Priority-aware `buzzerAlert()` — new alert only starts if its priority ≥ currently
  running alert
- Priority-aware `buzzerAlertContinuous()` — same logic
- `buzzerStop()` resets priority to 0 on stop so next alert of any level can start
- `noTone()` called immediately on priority override to cleanly stop current tone

### Changed
- Battery cutoff alert: priority 2 (highest) — guaranteed to override any running alert
- Battery warning alert: priority 2
- Receiver failsafe alert: priority 1
- Arm, disarm, altitude hold toggle alerts: priority 0 — cannot interrupt battery or
  failsafe alerts
- `updateArmingAndHoldMode()` arm/disarm/hold beeps converted from direct `tone()` calls
  to `buzzerAlert()` at priority 0 — direct `tone()` calls were bypassing the priority
  system entirely and could silence active battery warnings

### Fixed
- `buzzerAlertContinuous()` during battery cutoff could be silently skipped if another
  alert was active — `buzzerStop()` now called first in cutoff path before
  `buzzerAlertContinuous()`

---

## [0.6.0] — Battery Monitoring Improvements

### Added
- `lowBatCutoff` boolean — latched state, prevents re-arm until voltage recovers
- `lowBatWarning` boolean — separate flag for warning threshold, prevents repeated
  warning beeps
- Hysteresis on voltage cutoff recovery: cutoff clears at 9.6V, triggers at 9.3V
- `buzzerAlertContinuous()` function for continuous beeping on cutoff
- `buzzerStop()` function to force-clear any running buzzer sequence

### Changed
- `canArm` now checks `!lowBatCutoff` — drone cannot be armed while battery is in
  cutoff state
- `forceDisarm` now triggers on `lowBatCutoff` in addition to arm switch low
- `updateLEDs()` low battery LED: solid on cutoff, blinking on warning, off otherwise

### Removed
- Old `checkBatteryVoltage()` blocking `delay()` loops — replaced with non-blocking
  state machine in previous version

---

## [0.5.0] — Failsafe Debouncing

### Added
- `timeoutCounter` — failsafe requires 5 consecutive loop iterations of signal loss
  before triggering (~25ms at 200Hz)
- `goodSignalCounter` — signal recovery requires 10 consecutive good loops before
  clearing failsafe (~50ms)
- Prevents single-frame glitches from triggering full failsafe disarm

### Changed
- Failsafe buzzer now uses `buzzerAlert()` state machine instead of blocking beep
  sequence

---

## [0.4.0] — WiFi Core Pinning & Mutex

### Added
- `dataMutex` — FreeRTOS mutex protecting all shared variables accessed from both cores
- WiFi task pinned to Core 0 via `xTaskCreatePinnedToCore()` with 8192-byte stack
- Flight loop remains on Core 1

### Fixed
- `handleData()` was calling `readSensors()` mid-loop — caused double sensor reads and
  I2C bus contention with Core 1
- `temperature` now read exclusively on Core 1 in `readSensors()` — eliminates I2C bus
  conflict
- `global_dt` moved inside mutex — was written on Core 1 outside mutex, read inside
  mutex on Core 0 (data race)
- PID setpoints (`rollSetpoint`, `pitchSetpoint`, `yawRateSetpoint`, `altSetpoint`)
  moved inside mutex — consistent with PID compute
- Altitude tracking `altSetpoint` update moved inside mutex

### Changed
- `handleData()` now snapshots all shared variables inside a single mutex lock then
  releases before JSON formatting
- `snprintf()` with 1024-byte stack buffer replaces `String` concatenation in
  `handleData()` — eliminates heap fragmentation
- Buffer overflow check added: warns to Serial if JSON output is truncated

---

## [0.3.0] — Timing & Sensor Fixes

### Fixed
- `global_dt` was measured before `readSensors()` but used after — sensor I2C blocking
  time (~800µs) was excluded from dt, corrupting PID derivative term. `global_dt` now
  recaptured after `readSensors()` using `now2`
- `prevTime` updated to `now2` (post-sensor timestamp) for consistency — `prevTime = now`
  was leaving a systematic timing error
- `prevTime` set to `now` when disarmed — prevents massive dt spike on first armed loop
  iteration after disarm period
- Altitude feedforward was using wrong throttle centre (1500) instead of 500 —
  corrected

### Changed
- I2C clock set to 400kHz (`Wire.setClock(400000)`) — reduces MPU6050 + BMP280 read
  time from ~3200µs to ~800µs
- BMP280 standby time corrected to `STANDBY_MS_63` — was `STANDBY_MS_62_5` which is
  not a valid enum value
- MPU6050 filter bandwidth increased from 21Hz to 44Hz
- PID sample time explicitly set: `SetSampleTime(LOOP_INTERVAL_MS)` on all four PIDs
- `LOOP_INTERVAL_MS` global constant added — single source of truth for 5ms loop target
- `setup()` uses single `millis()` capture for both `lastValidSignalTime` and `prevTime`
  — removes race between two separate calls

---

## [0.2.0] — Non-Blocking Refactor

### Added
- `BuzzerState` struct — non-blocking buzzer state machine
- `updateBuzzer()` — called every loop, drives beep sequences without blocking
- `buzzerAlert()` — triggers a beep sequence and returns immediately
- Non-blocking battery sampling — 16-sample ADC average spread across loop iterations,
  one sample per 2ms
- Hardware ADC oversampling: 4 samples averaged per iteration

### Fixed
- `buzzerAlert()` was blocking up to 550ms during flight via `delay()` loops
- `checkBatteryVoltage()` was blocking 32ms via delay loop between ADC samples

### Changed
- `stopMotors()` no longer calls `noTone()` — buzzer state machine manages tone
  independently

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
- Serial debug output every 250ms
- Centripetal acceleration correction for IMU offset from centre of rotation