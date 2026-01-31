# ESP32 Quadcopter – Troubleshooting Guide

This document lists common issues, symptoms, causes, and step-by-step fixes for building, calibrating, and flying the quadcopter.

## 1. Sensors Not Detected ("Failed to find MPU6050" or "BMP280")
**Symptoms**: Serial Monitor shows failure messages during `setup()`.

**Common Causes & Fixes**:
- **Wiring swapped**: Swap SDA (GPIO 21) and SCL (GPIO 22) wires — very common mistake.
- **Power wrong**: Sensors must be on **3V3** pin (not 5V from buck). Check voltage with multimeter.
- **No pull-ups**: Add 4.7 kΩ resistors from SDA/SCL to 3.3V if your breakout boards lack them.
- **Address conflict**: MPU6050 AD0 pin should be GND (0x68). BMP280 SDO pin should be GND (0x76).
- **I²C noise**: Add 0.1 µF ceramic capacitor across VCC–GND on each sensor.
- **Fix**: Run an I²C scanner sketch to confirm addresses appear on bus.

## 2. Noisy / Jumping Altitude Readings
**Symptoms**: `height` value fluctuates wildly even when quad is still.

**Common Causes & Fixes**:
- **Vibration**: Mount BMP280 with foam tape or soft mounting to isolate from motor vibes.
- **Power noise**: Add 0.1 µF ceramic cap right next to BMP280 VCC–GND pins.
- **Filter tuning**: Reduce `alpha` in `fuseAltitude()` (e.g., from 0.98 → 0.92–0.95) to trust baro more.
- **Bad baro offset**: Power cycle while flat/still — recalibrate `baroOffset`.
- **Fix**: Print raw `baroAlt` vs fused `height` in debug to isolate the issue.

## 3. Random Disarms or ESP32 Resets
**Symptoms**: Quad disarms mid-test or ESP32 reboots unexpectedly.

**Common Causes & Fixes**:
- **Brownout (voltage dip)**: Add 470 µF electrolytic + 0.1 µF ceramic at 5V buck output.
- **Receiver signal loss**: Check antenna placement; increase `FAILSAFE_TIMEOUT_MS` if false triggers.
- **Low battery false positive**: Recalibrate `VDIV_RATIO` with multimeter at full charge.
- **Overcurrent**: Confirm motors/ESCs not drawing >30A continuous per motor.
- **Fix**: Watch Serial for "LOW VOLTAGE CUTOFF" or "FAILSAFE" messages.

## 4. Motors Not Spinning After Arming
**Symptoms**: Armed (no failsafe), but motors stay at 1000 µs.

**Common Causes & Fixes**:
- **ESC not calibrated**: Power ESCs with throttle high → wait for beeps → lower throttle to arm.
- **PWM signal wrong**: Confirm 50 Hz in code matches ESC protocol (most accept it).
- **Throttle too low**: Ensure `rcThrottle` >1100 µs when arming.
- **Fix**: Print `throttle` value in `mixAndWriteMotors()` to verify.

## 5. Altitude Hold Unstable (climbs/descends or oscillates)
**Symptoms**: Drone drifts up/down or bounces in hold mode.

**Common Causes & Fixes**:
- **PID gains wrong**: Reduce `pidAlt` Kp/Kd first (start low), then tune Ki slowly.
- **Baro noise/vibration**: Add capacitor + soft mount BMP280.
- **Accel Z not compensated**: Check `accelOffsetZ` from calibration (should be near -9.81).
- **Fix**: Test manual throttle first — ensure `height` is stable near 0 after calibration.

## 6. Receiver Input Not Working
**Symptoms**: `rcRoll`, `rcPitch`, etc. stuck at 0 or wrong values.

**Common Causes & Fixes**:
- **Wrong pins**: Confirm GPIO 32–35,25,26 match CH1–CH6 on receiver.
- **Not bound**: Hold bind button on receiver, power on transmitter with bind switch.
- **PulseIn timeout**: Increase timeout value (30000 µs) or use interrupt-based reading.
- **Fix**: Print raw PWM values in `readReceiver()` to debug.

## 7. Buzzer Not Working or Constant Tone
**Symptoms**: No sound or tone never stops.

**Common Causes & Fixes**:
- **Wrong pin**: Confirm buzzer + is on GPIO 13.
- **Passive vs active**: Active buzzers just need HIGH/LOW; passive need `tone()`.
- **Tone stuck**: `noTone(BUZZER_PIN)` called on re-arm or disarm.
- **Fix**: Test buzzer with simple `tone(BUZZER_PIN, 1000, 500);` in `setup()`.

## General Tips
- **Breadboard noise**: Use short wires, solid ground rail, and add ceramics ASAP.
- **No props first**: Always bench-test arming, sensors, and buzzer without props.
- **Serial Monitor**: Keep it open (115200 baud) — most errors print there.
- **Still stuck?** Paste your Serial output here or in issues for help.

Most problems are wiring, noise, calibration, or ESC protocol related.