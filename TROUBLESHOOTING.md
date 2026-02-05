# ESP32 Quadcopter – Troubleshooting Guide

Common issues, symptoms, causes and fixes for this project.

## 1. No WiFi AP ("QuadTelemetry") appears
**Symptoms**: No network to connect to after power-on.

**Causes & Fixes**:
- WiFi init failed → Check `initHTTPTelemetry()` ran (Serial prints IP)
- ESP32 in deep sleep or brownout → Add 1000 µF electrolytic on 5V rail
- Code crash before WiFi.start → Look for earlier errors in Serial
- Fix: Re-upload sketch, watch Serial for "WiFi AP started" message

## 2. Browser can't connect to 192.168.4.1
**Symptoms**: Page doesn't load or "connection refused"

**Causes & Fixes**:
- Not connected to QuadTelemetry WiFi → Connect phone/laptop to it first
- Wrong IP → Check Serial Monitor for correct IP (usually 192.168.4.1)
- Firewall or VPN blocking → Disable VPN, try different device
- Server not running → Ensure `server.handleClient()` is called in loop()

## 3. Telemetry data not updating / stuck
**Symptoms**: JSON shows old values or "Loading..." forever

**Causes & Fixes**:
- ESP32 overloaded → Reduce `delay()` in loop or increase update interval
- Browser cache → Hard refresh (Ctrl+F5)
- WiFi interference → Move to open area or change AP channel in code
- Fix: Open `/data` directly in browser → should return JSON instantly

## 4. Receiver channels not responding
**Symptoms**: PWM values stuck at 0 or random

**Causes & Fixes**:
- Not bound → Hold bind button on receiver, power on transmitter in bind mode
- Wrong pins → Confirm CH1–CH6 on GPIO 32,33,34,35,25,26
- Power issue → Receiver VCC must be 4.8–6 V (use 5V buck, not 3V3 if unstable)
- Noise → Add 0.1 µF ceramic across receiver VCC–GND

## 5. Voltage reading inaccurate / noisy
**Symptoms**: VBat doesn't match multimeter or jumps around

**Causes & Fixes**:
- Wrong calibration → Use multimeter at 2–3 points (full, half, low) → recalculate VDIV_M & VDIV_C
- High source impedance → Lower resistors (e.g. 33 kΩ + 10 kΩ) or add op-amp buffer
- ADC noise → Add 0.1 µF ceramic from GPIO 36 to GND
- Fix: Average more samples (e.g. 64 instead of 16)

## 6. LEDs not responding to switches
**Symptoms**: Green/Blue LEDs don't change with CH5/CH6

**Causes & Fixes**:
- Wrong pin → Confirm Green = 15, Blue = 4
- PWM values wrong → Check Serial for pwmArm/pwmAux (should be ~1000–2000 µs)
- Threshold wrong → Adjust >1500 logic if your transmitter mid-point is not 1500 µs
- Fix: Print pwmArm and pwmAux in loop() to verify

## 7. Buzzer not sounding
**Symptoms**: No beeps on startup or confirmation

**Causes & Fixes**:
- Wrong pin → Confirm BUZZER_PIN = 13
- Passive vs active buzzer → Active buzzers need only HIGH/LOW; passive need tone()
- Polarity reversed → Swap buzzer legs
- Fix: Test with `tone(BUZZER_PIN, 1000, 500);` in setup()

## General Tips
- Always test **without props** first
- Keep Serial Monitor open — most errors print there
- Breadboard noise → Short wires, solid ground rail, add ceramics
- Still stuck? Paste Serial output here for help

For full project details → [README.md](README.md)  
For wiring → [WIRING.md](WIRING.md)  
For components → [COMPONENTS.md](COMPONENTS.md)