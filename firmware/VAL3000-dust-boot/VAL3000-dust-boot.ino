/****LIBRARIES REQUIRED******
* TMCStepper: https://github.com/teemuatlut/TMCStepper
* ESP32_Button: https://github.com/esp-arduino-libs/ESP32_Button
*/

// ACTIONS
/*
On boot up do nothing
On button press 1 or input from the CNC controller spin the motor until it hits a limit switch, I assume I can use the 2nd io on the board for this. Hold the motor at the limit switch. 
On 2 being pressed or the input from the controller going off spin the motor in the opposite direction for the same amount of time it took to reach the limit switch. 
*/

#include <TMCStepper.h>
#include <Button.h>
#include <Preferences.h>
#include "memory.h"

#define STEP_PIN 10
#define ENABLE_PIN 8
#define RX_PIN 5
#define TX_PIN 6
#define STALLGUARD_PIN 1
#define INDEX_PIN 0
#define BUTTON_1_PIN GPIO_NUM_3
#define BUTTON_2_PIN GPIO_NUM_4
#define BUTTON_WIFI_PIN GPIO_NUM_7

#define R_SENSE 0.11f     // R_SENSE for current calc.
#define DRIVER_ADDRESS 0  // TMC2209 Driver address according to MS1 and MS2

// ## Speed
// Sets the speed in microsteps per second.
// If for example the the motor_microsteps is set to 16 and your stepper motor has 200 full steps per revolution (Most common type. The motor angle will be 1.8 degrees on the datasheet)
// It means 200 x 16 = 3200 steps per revolution. To set the speed to rotate one revolution per seconds, we would set the value below to 3200.
// µstep velocity v[Hz] = VACTUAL[2209] * 0.715Hz
//Change these values to get different speeds
#define CLOSE_VELOCITY 1000
#define OPEN_VELOCITY -1000
#define STOP_MOTOR_VELOCITY 0

bool stalled_motor = false;
bool motor_moving = false;
bool is_moving = false;
bool is_lowering = false;
bool move_to_max = false;
bool set_distance = false;

uint32_t target_position;
int32_t motor_position;
uint32_t maximum_motor_position;
uint8_t target_percent;

// FreeRTOS task handle for position monitoring
TaskHandle_t position_watcher_task_handler = NULL;

// We communicate with the TMC2209 over UART
// But the Arduino UNO only has one Serial port which is connected to the Serial Monitor
// We can use software serial on the UNO, and hardware serial on the ESP32 or Mega 2560

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

// Interrupt
// This interrupt will fire when a HIGH rising signal is detected on the DIAG pin. This indicates a stall.

void IRAM_ATTR stalled_position() {
  stalled_motor = true;
}

// ## Track Steps
// When using the pulse generator, the TMC2209 will tell us each time a step has been taken by pulsing the INDEX pin. We can create a tracker to add or subract steps from the tracker
// The index output gives one pulse per electrical rotation, i.e., one pulse per each four fullsteps. I
// The is_lowering variable keeps track of which direction the motor is spinning, this way we know whether to add or subract from the position.

void IRAM_ATTR index_interrupt(void) {

  if (is_lowering == true) {
    motor_position++;
  } else {
    motor_position--;
  }
}

// Lower the boot to the maximum_motor_position
static void btn1SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Boot Down");

  set_distance = false;
  motor_position = 0;  //Resets the motor position to 0
  target_position = maximum_motor_position;
  vTaskResume(position_watcher_task_handler);
  delay(100);
  enable_driver();
  driver.VACTUAL(OPEN_VELOCITY);
}

// Lower the boot until it stalls
static void btn1LongPressCb(void *button_handle, void *usr_data) {
  Serial.println("Boot Down");

  set_distance = true;

  motor_position = 0;       //Resets the motor position to 0
  target_position = 20000;  // Random high number
  vTaskResume(position_watcher_task_handler);
  delay(100);
  enable_driver();
  driver.VACTUAL(OPEN_VELOCITY);
}

// Turns the motor in a different direction
static void btn2SingleClickCb(void *button_handle, void *usr_data) {
  Serial.println("Boot Up");

  target_position = 0;

  vTaskResume(position_watcher_task_handler);
  delay(100);
  enable_driver();
  driver.VACTUAL(CLOSE_VELOCITY);
}


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 can use any pins to Serial

  Serial.println("START setup");
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  pinMode(STALLGUARD_PIN, INPUT);
  pinMode(INDEX_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(STALLGUARD_PIN), stalled_position, RISING);
  attachInterrupt(digitalPinToInterrupt(INDEX_PIN), index_interrupt, RISING);

  Button btn1 = Button(BUTTON_1_PIN, false);
  Button btn2 = Button(BUTTON_2_PIN, false);

  btn1.attachSingleClickEventCb(&btn1SingleClickCb, NULL);   // Attaches button function btn1SingleClickCb
  btn1.attachLongPressStartEventCb(&btn1LongPressCb, NULL);  // Attaches button function btn1LongPressCb
  btn2.attachSingleClickEventCb(&btn2SingleClickCb, NULL);   // Attaches button function btn2SingleClickCb


  driver.begin();  // Start all the UART communications functions behind the scenes

  /* General Registers */
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);
  driver.en_spreadCycle(false);
  driver.index_otpw(false);
  driver.index_step(true);
  driver.pdn_disable(true);
  driver.mstep_reg_select(true);
  driver.multistep_filt(true);

  // NODECONF Registers
  driver.senddelay(6);

  // Factory Registers
  driver.ottrim(0);

  // Velocity Dependent Control
  driver.irun(31);  // Max current. Based on 0.11 Rsense resistors
  driver.ihold(0);
  driver.iholddelay(1);  // Set I_HOLD_DELAY to 1 to 15 for smooth standstill current decay
  driver.TPOWERDOWN(20);
  driver.TPWMTHRS(0);
  driver.VACTUAL(0);

  // CHOPCONF – Chopper Configuration
  driver.diss2vs(0);
  driver.diss2g(0);
  driver.dedge(0);
  driver.intpol(1);

  driver.mres(8);  // 8 = FULLSTEP mode. 200 pulses per revolution.
  driver.vsense(0);
  driver.tbl(2);
  driver.hend(0);
  driver.hstrt(4);
  driver.toff(5);

  // PWMCONF – Voltage PWM Mode StealthChop
  driver.pwm_lim(12);
  driver.pwm_reg(8);    // Try 2
  driver.freewheel(1);  // 1= Freewheel mode. 3 = Coil Short HS. Only short the coil when motor is NOT moving. Use for physical security
  driver.pwm_autograd(1);
  driver.pwm_autoscale(1);
  driver.pwm_freq(1);
  //driver.pwm_grad(PWM_grad);  // Test different initial values. Use scope.
  driver.pwm_ofs(36);
}

void loop() {

  // Button actions are in the button functions
}

int getMotorPosition() {

  return motor_position;
}

/* Enables power stage of TMC */
void enable_driver() {
  digitalWrite(ENABLE_PIN, 0);
}

/* Disabled power stage of TMC */
void disable_driver() {
  digitalWrite(ENABLE_PIN, 1);
}

/* Stops motor */
void stop() {
  disable_driver();
  driver.VACTUAL(STOP_MOTOR_VELOCITY);
#ifdef LOGGING_ENABLED
  printf("stop(): Motor stopped\n");
#endif
}

// ========================================
// POSITION WATCHER TASK
// ========================================

/**
 * @brief FreeRTOS task that monitors motor position and stops at target
 *
 * This task runs continuously, suspended when not needed. It monitors the motor
 * position and stops movement when target is reached or stop conditions occur.
 */
void position_watcher_task(void *parameter) {

  Serial.println("position_watcher_task CREATED");

  while (true) {
    vTaskSuspend(NULL);

    Serial.print("is_moving: ");
    Serial.println(is_moving);

    while (is_moving) {
      loop_counter++;

      // Check if stop button was pressed
      if (stop_flag) {
        stop();
        stop_flag = false;

        printf("position_watcher: Stop requested\n");

        delay(1000);
        goto notify_and_suspend;
      }

      // Check for stall condition
      if (is_lowering) {
        if (stall_flag) {
          stop();
          stall_flag = false;
          if (set_distance) {
            maximum_motor_position = motor_position;
            //Need to save to preferences
          }

          printf("position_watcher: Stall detected\n");

          goto notify_and_suspend;
        }
      }

      // Check if target position reached
      if (is_lowering) {
        if (motor_position >= target_position) {

          printf("position_watcher: Target reached (closing) - pos: %u, target: %u\n",
                 (unsigned int)motor_position, (unsigned int)target_position);

          stop();
          goto notify_and_suspend;
        }
      } else {
        if (motor_position <= target_position) {

          printf("position_watcher: Target reached (opening) - pos: %u, target: %u\n",
                 (unsigned int)motor_position, (unsigned int)target_position);

          stop();
          goto notify_and_suspend;
        }
      }

      // Periodic position logging
      Serial.println(motor_position);
    }

    delay(20);
  }

notify_and_suspend:

  Serial.println("Movement complete - updating state");

  is_moving = false;

  // Calculate current lift percentage (inverted for Matter standard: 100% = closed, 0% = open)
  int currentLiftPercent = (((float)motor_position / (float)maximum_motor_position) * 100.0);

  printf("Final state - Position: %lu, Lift%%: %lu, Target: %lu, Max: %lu\n",
         motor_position, currentLiftPercent, target_position, maximum_motor_position);

  // Save state to preferences
  preferences.putUChar(PREF_LIFT_PERCENT, currentLiftPercent);
  preferences.putInt(PREF_MOTOR_POS, motor_position);
}
}
