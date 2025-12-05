// https://github.com/esp-arduino-libs/ESP32_Button

#include <Arduino.h>
#include <Button.h>

#include <ESPUI.h>

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
  // put your setup code here, to run once:
  Serial.begin(115200);

  // initializing a button
  Button btn1 = Button(GPIO_NUM_3, false);
  Button btn2 = Button(GPIO_NUM_4, false);
  Button btn3 = Button(GPIO_NUM_7, false);

  btn1.attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn1.attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn1.attachLongPressStartEventCb(&onButtonLongPressStartCb, NULL);

  btn2.attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn2.attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn2.attachLongPressStartEventCb(&onButtonLongPressStartCb, NULL);

  btn3.attachSingleClickEventCb(&onButtonSingleClickCb, NULL);
  btn3.attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
  btn3.attachLongPressStartEventCb(&onButtonLongPressStartCb, NULL);
}

void loop() {
  delay(10);
}