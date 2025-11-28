/* TO DO
*
* long press main buttons to override positions. Move until button pressed 
* Press both buttons same time to set max position
*
* short press btn3 to change direction
* long press btn3 to reset wifi
*/


#include <Arduino.h>
//#include "button_settings.h"
#include <Button.h>

#define BUTTON_1_PIN GPIO_NUM_3
#define BUTTON_2_PIN GPIO_NUM_4
#define WIFI_RESET_PIN GPIO_NUM_7

#include "memory.h"
#include "motor_control.h"
#include "api_settings.h"
#include "espui_settings.h"

#include <ArduinoOTA.h>  // For enabling over-the-air updates

static void onButtonSingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button single click");
}
static void onButtonDoubleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Button double click");
}
static void onButtonLongPressStartCb(void *button_handle, void *usr_data) {
  Serial.println("Button long press click");
}

void setup() {

  Serial.begin(115200);
  Serial.println("BEGIN SETUP");

  Button btn1 = Button(BUTTON_1_PIN, false);    //BUTTON_1_PIN
  Button btn2 = Button(BUTTON_2_PIN, false);    //BUTTON_1_PIN
  Button btn3 = Button(WIFI_RESET_PIN, false);  //BUTTON_1_PIN

  btn1.attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn1.attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn1.attachLongPressStartEventCb(&onButtonLongPressStartCb, NULL);

  btn2.attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn2.attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn2.attachLongPressStartEventCb(&onButtonLongPressStartCb, NULL);

  btn3.attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn3.attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn3.attachLongPressStartEventCb(&onButtonLongPressStartCb, NULL);

  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 can use any pins to Serial

  delay(100);

  preferences.begin("local", false);
  load_preferences();
  setup_motors();
  setup_wifi();
  ESPUIsetup();

  ArduinoOTA.begin();

  Serial.println("COMPLETE SETUP");
}

// // Variables will change:
// int lastStateBtn1 = LOW;  // the previous state from the input pin
// int currentStateBtn1;     // the current reading from the input pin

// int lastStateBtn2 = LOW;  // the previous state from the input pin
// int currentStateBtn2;     // the current reading from the input pin

// int lastStateBtn3 = LOW;  // the previous state from the input pin
// int currentStateBtn3;     // the current reading from the input pin

void loop() {
  dnsServer.processNextRequest();  // Process request for ESPUI
  ArduinoOTA.handle();             // Handles a code update request
  //wifiResetButton();

  // // read the state of the switch/button:
  // currentStateBtn1 = digitalRead(BUTTON_1_PIN);
  // currentStateBtn2 = digitalRead(BUTTON_2_PIN);
  // currentStateBtn3 = digitalRead(WIFI_RESET_PIN);

  // if (lastStateBtn1 == HIGH && currentStateBtn1 == LOW) {
  //   Serial.println("Btn1 is pressed");
  //   if (is_moving) {
  //     stop_flag = true;
  //   } else {
  //     move_to_percent100ths(0);
  //   }
  // }

  // if (lastStateBtn2 == HIGH && currentStateBtn2 == LOW) {
  //   Serial.println("Btn2 is pressed");
  //   if (is_moving) {
  //     stop_flag = true;
  //   } else {
  //     move_to_percent100ths(100);
  //   }
  // }

  // if (lastStateBtn3 == HIGH && currentStateBtn3 == LOW) {
  //   Serial.println("Btn3 is pressed");
  //   if (is_moving) {
  //     // Do nothing
  //   } else {
  //     setCloseCall();
  //     ESPUI.updateSlider(positionSlider, target_percent);
  //   }
  // }

  // // save the the last state
  // lastStateBtn1 = currentStateBtn1;
  // lastStateBtn2 = currentStateBtn2;
  // lastStateBtn3 = currentStateBtn3;
  delay(10);
}