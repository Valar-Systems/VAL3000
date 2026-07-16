# VAL3000 ESPHome Firmware

This YAML configuration integrates the VAL3000 with [Home Assistant](https://www.home-assistant.io/) via [ESPHome](https://esphome.io/). It exposes the motor as a **cover** entity (open/close/position) and lets you tune speed, acceleration, current (IRUN), TCOOLTHRS, and SGTHRS directly from the Home Assistant UI — no reflashing needed.

It uses the [slimcdk TMC2209 external components](https://github.com/slimcdk/esphome-custom-components) for full UART control of the driver, including StallGuard-based homing (the cover homes itself by driving until it stalls).

## Setup

1. Copy `VAL3000-esphome.yml` into your ESPHome dashboard (or use the ESPHome CLI).
2. Set your secrets — the config expects `wifi_ssid`, `wifi_password`, and `api_key` in your ESPHome `secrets.yaml`.
3. Change the `name` and `friendly_name` under `esphome:` to match your device.
4. Adjust the `substitutions` for your mechanics:
   - `gear_distance_cm` — distance traveled per motor revolution (default is for an MK7 drive gear)
   - `steps_per_revolution` — steps per revolution at the configured microstepping (default 1600 = 200 full steps × 8 microsteps)
5. Flash over USB the first time (hold BOOT, tap RESET, release BOOT — see the [wiki](https://github.com/Valar-Systems/VAL3000/wiki)). After that, updates work over the air.

## What you get in Home Assistant

- **Cover entity** with position control, homed via StallGuard
- **Buttons on the board**: Button 1 closes, Button 2 opens; holding Button 1 for 3–10 seconds re-homes to position 0
- **Config entities**: distance (cm), speed, acceleration, IRUN current, TCOOLTHRS, SGTHRS, and motor direction
- **Diagnostic sensors**: TSTEP and SG_RESULT — useful for [tuning StallGuard](/firmware/README.md#stallguard)
