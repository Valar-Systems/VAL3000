# VAL3000 Matter Firmware (ESP-IDF)

This is the **recommended** Matter firmware for the VAL3000. It is built on [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/) and [esp-matter](https://docs.espressif.com/projects/esp-matter/en/latest/esp32/), and is more stable than the Arduino Matter version.

Once commissioned, the VAL3000 appears as a window covering device in Apple Home, Amazon Alexa, Google Home, or any other Matter-compatible ecosystem.

## Requirements

> **WARNING:** This project can ONLY be built on **Linux or macOS**. Windows will NOT work. If you have Windows, you MUST use WSL and open this folder from within WSL (navigate to this folder in WSL and run `code .` to open it in VS Code).

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html) installed and exported in your shell
- The target is the **ESP32-C3** (already configured in `sdkconfig.defaults`)

## Building and Flashing

```bash
idf.py set-target esp32c3
idf.py build
idf.py -p <PORT> flash monitor
```

If this is the board's first flash, manually enter download mode first: **press and hold** BOOT, **press and release** RESET, then **release** BOOT. See the [wiki](https://github.com/Valar-Systems/VAL3000/wiki) for details.

## Commissioning

After flashing, open the serial monitor. The device will print a QR code URL and a manual pairing code. Use your Matter hub's app (Apple Home, Google Home, Alexa, etc.) to add the device by scanning the QR code or entering the pairing code.

To factory reset / decommission the device, erase the flash:

```bash
idf.py -p <PORT> erase-flash
```

## Project Structure

- `main/app_main.cpp` — Matter node setup and window covering endpoint
- `main/app_driver.cpp` — TMC2209 motor control driver
- `sdkconfig.defaults` — default configuration (ESP32-C3 target)
