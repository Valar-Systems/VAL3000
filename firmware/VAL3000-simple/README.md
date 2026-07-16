# VAL3000 Simple Firmware

This is the best place to start. It's a minimal Arduino sketch that spins the motor with the on-board buttons and shows how the TMC2209 works: UART configuration, the internal pulse generator (VACTUAL), position tracking via the INDEX pin, and StallGuard stall detection via the DIAG pin.

The code is written for the ESP32-C3 on the VAL3000, but can easily be modified to work with any single-core microcontroller.

## Required Libraries

- [TMCStepper](https://github.com/teemuatlut/TMCStepper)
- [ESP32_Button](https://github.com/esp-arduino-libs/ESP32_Button)

## Usage

1. Open `VAL3000-simple.ino` in the Arduino IDE and install the libraries above.
2. Upload to the board (first flash requires manually entering download mode — see the [wiki](https://github.com/Valar-Systems/VAL3000/wiki)).
3. Press Button 1 or Button 2 to spin the motor in either direction. The motor stops automatically when StallGuard detects a stall.

To tune the current, speed, and stall sensitivity, see the [StallGuard guide](/firmware/README.md#stallguard).
