# VAL3000 HTTP Server Firmware

This firmware turns the VAL3000 Wi-Fi stepper board into a standalone web server. It hosts its own web UI for setting position and motor parameters, plus an HTTP API you can call from your own automations.

## Step 1 — Connect Power & Motor

1. Connect a 12V 2A+ power cord like [this](https://amzn.to/30qsbh7)
2. Connect a stepper motor like [this](https://amzn.to/3kUWkP4)

If you need to move something heavy (over 50 lbs), consider using a geared motor [like this](https://amzn.to/3c9xmI0).

## Step 2 — Download and Open the Arduino Code

Using Arduino, open the `VAL3000-api.ino` file.

You will need to install the ESP32 core, as well as several libraries.

Install the ESP32 core by following the instructions [here](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/).

Install the following libraries inside Arduino:

1. [TMC Stepper](https://www.arduino.cc/reference/en/libraries/tmcstepper/)
2. [ESPUI](https://github.com/s00500/ESPUI)
3. [ArduinoJson](https://www.arduino.cc/reference/en/libraries/arduinojson/)

Install the following libraries outside Arduino. These cannot be installed using the Arduino library manager:

1. [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
2. [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)

a. Installing the ESPAsyncWebServer library:

- [Click here](https://github.com/me-no-dev/ESPAsyncWebServer/archive/refs/heads/master.zip) to download the ESPAsyncWebServer library. You should have a .zip folder in your Downloads folder
- Unzip the .zip folder and you should get an ESPAsyncWebServer-master folder
- Rename your folder from ESPAsyncWebServer-master to ESPAsyncWebServer
- Move the ESPAsyncWebServer folder to your Arduino IDE installation libraries folder

b. Installing the AsyncTCP library:

- The ESPAsyncWebServer library requires the AsyncTCP library to work. Follow the next steps to install that library:
- [Click here](https://github.com/me-no-dev/AsyncTCP/archive/refs/heads/master.zip) to download the AsyncTCP library. You should have a .zip folder in your Downloads folder
- Unzip the .zip folder and you should get an AsyncTCP-master folder
- Rename your folder from AsyncTCP-master to AsyncTCP
- Move the AsyncTCP folder to your Arduino IDE installation libraries folder

Finally, re-open your Arduino IDE. Compile the sketch and everything should compile.

## Step 3 — Upload the Sketch

You do not need to change anything in the code.

Upload the firmware to your board. (If this is the first upload, you'll need to manually enter download mode — see the [wiki](https://github.com/Valar-Systems/VAL3000/wiki) for the BOOT/RESET button sequence.)

## Step 4 — Connect to the Board as an Access Point

Using your smartphone or laptop (or any device with Wi-Fi), look for the Wi-Fi network named `VALAR-AP-XXXXXXXX` and connect to it.

This is an access point, which means you will connect directly to the device and will lose your internet connection.

When prompted, enter the Wi-Fi password `password`. If you want to change this, modify the `ap_password` variable in the `api_settings.h` file.

In your browser, go to http://192.168.4.1/

You are now connected to the device. In the top box, enter a value from 0–100 and the motor should spin.

## Step 5 — Add the Board to Your Network

It's best to add this device to your network so you do not need to connect to it directly.

To do so, go to http://192.168.4.1/wifi

Enter the name and password of your Wi-Fi router. Your device will now connect to your router.

Change your Wi-Fi network back to your router, because you will get disconnected from the VALAR-AP network.

You now need to find the IP address that your router assigned to the VAL3000. To do this, use a network scanning tool like [Angry IP Scanner](https://angryip.org/).

The hostname should be `esp32-arduino`. Find the IP address of this device and enter it into your browser.

You should now be connected to the device.

## Step 6 — Control Your Motor

Under the "Position" section, enter a value of 50 and click "Set Position". The motor should begin spinning to the 50 percent position.

The position value is a percentage from 0–100. If you enter 100, it will move 100 percent of the way. If you enter 0, it will move to the home position. The percentage is based on the `max_steps` value described below.

Set motor parameters by modifying the following values, then press the "Set Parameters" button:

### max_steps

This value sets the number of steps that corresponds to the 100 percent position. If you have a curtain or window, set the number of steps required to move it all the way open/closed.

There are 200 full steps per revolution, so use some math to figure out how many steps you need. For example, to move 20 inches (508 mm):

1. Measure the diameter of the pulley you are using. For the S1 curtain opener it is 12 mm.
2. Convert diameter to circumference: 12 mm × π ≈ 37.7 mm (1.484 inches) of travel per revolution.
3. Divide total travel by circumference: 20 in ÷ 1.484 in ≈ 13.47 revolutions.
4. Multiply by steps per revolution: 13.47 × 200 ≈ 2,695 steps.

### current

This value (400–2000) sets the amount of current in milliamps that will run through the motor. The higher the current, the stronger the motor will be. However, it will also run hotter, and it makes setting the stall value more difficult. It is recommended to set the current to the minimum required to move your motor: start at a small value and move up until the motor moves how you want. The maximum value is 2000 mA.

### stall

This value (0–255) sets the StallGuard sensitivity of the motor. StallGuard is fairly complex — see the [StallGuard guide](/firmware/README.md#stallguard) or take my [Udemy course](https://www.udemy.com/course/trinamic/?referralCode=F21BCEB4F4C3C664D13A) to learn more. Set this value to 0 to disable it if you do not understand how it works.

### accel

This value sets the acceleration of the motor. It is recommended you keep this the same as `max_speed` when using StallGuard.

### max_speed

This value (0–500) sets the speed of the motor. The driver will use the acceleration value to speed up to this value.

### Set Zero

Pressing this button will set the current position of the device back to 0.

## Step 7 — Control via API

You may want to set up automations to control your device. To trigger the motor to move, simply send an HTTP command via your browser or preferred method.

Go to the API tab in the web UI to find the exact address to send commands to. Replace the `0` with a value of 0–100 to set the position.
