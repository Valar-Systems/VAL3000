/**
 * @file curtain_motor.h
 * @brief Hardware driver for the VAL3000 curtain/blind motor controller.
 *
 * Encapsulates TMC2209 stepper motor control over UART, GPIO-based step
 * counting (INDEX pin), StallGuard obstacle detection, and non-volatile
 * storage of motor state. Designed for ESP32-C3 running ESP-IDF.
 *
 * @note Position is tracked by counting INDEX pin pulses (100 per revolution).
 *       The TMC2209 is driven in StealthChop mode with CoolStep current scaling.
 */

#pragma once

#include <TMCStepper.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/** @defgroup uart_config UART Configuration
 *  @{ */
#define MOTOR_UART_NUM UART_NUM_1  /**< UART peripheral used for TMC2209 communication. */
/** @} */

/** @defgroup pin_defs Hardware Pin Definitions (ESP32-C3)
 *  @{ */
#define ENABLE_PIN     GPIO_NUM_8  /**< Motor driver enable (active LOW). */
#define MOTOR_RX_PIN   GPIO_NUM_5  /**< UART RX for TMC2209. */
#define MOTOR_TX_PIN   GPIO_NUM_6  /**< UART TX for TMC2209. */
#define STALLGUARD_PIN GPIO_NUM_1  /**< StallGuard interrupt input. */
#define INDEX_PIN      GPIO_NUM_0  /**< INDEX/step pulse interrupt input. */
/** @} */

/** @defgroup button_pins Button Pin Definitions
 *  @{ */
#define BUTTON_CLOSE_PIN  GPIO_NUM_4  /**< Button 1 - Close control. */
#define BUTTON_OPEN_PIN   GPIO_NUM_3  /**< Button 2 - Open control. */
#define BUTTON_FUNC_PIN   GPIO_NUM_7  /**< Button 3 - Direction / Decommission. */
/** @} */

/** @defgroup velocity Motor Velocity Constants (VACTUAL units)
 *  @{ */
#define OPEN_VELOCITY    400   /**< Opening velocity (positive VACTUAL). */
#define CLOSE_VELOCITY  -400   /**< Closing velocity (negative VACTUAL). */
#define CALIBRATION_OPEN_VELOCITY    300   /**< Opening velocity (positive VACTUAL). */
#define CALIBRATION_CLOSE_VELOCITY  -300   /**< Closing velocity (negative VACTUAL). */
#define STOP_VELOCITY    0     /**< Zero velocity (motor stopped). */
/** @} */

/** @defgroup tmc_config TMC2209 Configuration
 *  @{ */
#define DRIVER_ADDRESS  0b00       /**< TMC2209 UART address (AD0/AD1 = LOW). */
#define R_SENSE         0.11f      /**< Sense resistor value in ohms. */
#define MOTOR_BAUD_RATE 115200     /**< UART baud rate for TMC2209. */
/** @} */

/**
 * @brief Callback type invoked when the motor finishes a movement.
 * @param percent100ths Final lift position in hundredths of a percent (0-10000).
 */
typedef void (*motor_stop_callback_t)(uint16_t percent100ths);

/**
 * @class curtain_motor
 * @brief Controls a TMC2209-driven stepper motor for curtain/blind positioning.
 *
 * Provides percentage-based positioning (0-10000 in percent100ths), calibration
 * via set-distance mode, direction toggling, and NVS persistence of all motor state.
 * Movement is monitored by a dedicated FreeRTOS task that tracks INDEX pulses
 * and invokes application-layer callbacks on completion or stall.
 */
class curtain_motor
{
private:
    /**
     * @brief FreeRTOS task that monitors motor position during movement.
     *
     * Suspends itself when idle. Resumes when a move command starts.
     * Checks stop_flag, stall_flag, and target_position every 20 ms.
     * On completion, saves state to NVS and invokes on_motor_stop.
     *
     * @param[in] param Pointer to the owning curtain_motor instance.
     */
    static void position_watcher_task(void *param);

    /**
     * @brief ISR for the INDEX pin (step pulse counter).
     *
     * Increments motor_position when closing, decrements when opening.
     * Runs in IRAM; uses a portMUX spinlock for atomicity.
     *
     * @param[in] arg Pointer to the owning curtain_motor instance.
     */
    static void IRAM_ATTR index_interrupt(void *arg);

    /**
     * @brief ISR for the StallGuard pin.
     *
     * Sets the stall_flag for the position_watcher_task to handle.
     * Runs in IRAM.
     *
     * @param[in] arg Pointer to the owning curtain_motor instance.
     */
    static void IRAM_ATTR stall_interrupt(void *arg);

    TMC2209Stepper driver;                        /**< TMCStepper library driver instance. */
    TaskHandle_t position_watcher_task_handler;    /**< Handle of the position watcher FreeRTOS task. */

    /* Motor state */
    enum SystemState { OPENING, CLOSING, IDLE }; /**< Motor operational state: OPENING, CLOSING, IDLE. */
    enum SystemState operational_state = IDLE;
    volatile int32_t motor_position;               /**< Current motor position in INDEX pulses. */
    int32_t target_position;                       /**< Target motor position in INDEX pulses. */
    int32_t maximum_motor_position;                /**< Maximum travel distance in INDEX pulses (calibrated). */

    volatile bool is_closing;                      /**< True when the motor is moving in the close direction. */ // REPLACE WITH OPERATIONAL_STATE
    volatile bool is_moving;                       /**< True when the motor is actively moving. */ // Also replace with operantional_state?
    volatile bool stop_flag;                       /**< Set to request a graceful stop from the watcher task. */
    volatile bool stall_flag;                      /**< Set by the StallGuard ISR on obstacle detection. */
    

    /* Configuration */
    uint8_t opening_direction;                     /**< Motor shaft direction: 0 = normal, 1 = reversed. */

    /* NVS keys */
    static constexpr const char *NVS_NAMESPACE       = "curtain_motor";  /**< NVS namespace for motor data. */
    static constexpr const char *NVS_MOTOR_POS_KEY    = "motor_pos";     /**< NVS key: current motor position. */
    static constexpr const char *NVS_MAX_POS_KEY      = "max_motor_pos"; /**< NVS key: maximum motor position. */
    static constexpr const char *NVS_OPEN_DIR_KEY     = "open_dir";      /**< NVS key: opening direction. */
    static constexpr const char *NVS_LIFT_PERCENT_KEY = "lift_percent";  /**< NVS key: last lift percentage. */
    static constexpr const char *NVS_MAX_LIFT_KEY     = "max_lift";      /**< NVS key: max lift in centimeters. */

    /* Callbacks */
    motor_stop_callback_t on_motor_stop;           /**< Called when motor movement completes. */

    /** @brief Immediately stop the motor (disable driver, set VACTUAL to 0). */
    void stop_motor(void);

    /** @brief Enable the TMC2209 driver (pull ENABLE_PIN LOW). */
    void enable_driver(void);

    /** @brief Disable the TMC2209 driver (pull ENABLE_PIN HIGH). */
    void disable_driver(void);

    /** @brief Save the current motor_position to NVS. */
    void save_motor_position(void);

    /**
     * @brief Write the full TMC2209 register configuration.
     *
     * Sets StealthChop, StallGuard threshold, CoolStep parameters,
     * chopper configuration, and PWM settings. Must be called after
     * UART is initialized.
     */
    void configure_tmc2209(void);

public:
    /**
     * @brief Construct a new curtain_motor instance.
     *
     * @param[in] on_motor_stop Callback invoked when motor movement ends.
     */
    curtain_motor(motor_stop_callback_t on_motor_stop);

    /**
     * @brief Initialize all hardware: UART, GPIOs, ISRs, TMC2209 registers, and watcher task.
     *
     * Also loads saved preferences from NVS. Must be called once before any
     * movement commands.
     *
     * @return ESP_OK on success, or an error code on failure.
     */
    esp_err_t initialize(void);

    /**
     * @brief Move the motor to a target lift position.
     *
     * Converts the percentage to a step-based target_position and starts
     * the motor in the appropriate direction. The position_watcher_task
     * monitors progress and stops automatically.
     *
     * @param[in] percent100ths Target position in hundredths of a percent (0 = fully open, 10000 = fully closed).
     * @return ESP_OK on success, or an error code.
     */
    esp_err_t move_to_percent100ths(uint16_t percent100ths);

    /**
     * @brief Request a graceful motor stop.
     *
     * Sets the stop_flag, which the position_watcher_task checks on its
     * next 20 ms polling cycle.
     */
    void smooth_stop(void);

    /**
     * @brief Enter set-distance calibration mode in the opening direction.
     *
     * Resets motor_position to 0 and starts the motor opening. The user
     * presses a button to stop and call finish_set_distance_open().
     */
    void start_set_distance_open(void);

    /**
     * @brief Enter set-distance calibration mode in the closing direction.
     *
     * Starts the motor closing. The user presses a button to stop and
     * call finish_set_distance_close().
     */
    void start_set_distance_close(void);

    /**
     * @brief Finish set-distance calibration (open direction).
     *
     * Stops the motor, calculates maximum_motor_position from the distance
     * traveled, computes max lift in centimeters, resets position to 0
     * (fully open), and persists everything to NVS.
     */
    void finish_set_distance_open(void);

    /**
     * @brief Finish set-distance calibration (close direction).
     *
     * Stops the motor and resets motor_position to 0 (fully closed reference).
     * Saves the new position to NVS.
     */
    void finish_set_distance_close(void);

    /**
     * @brief Toggle the motor shaft direction and save to NVS.
     *
     * Swaps between normal (0) and reversed (1) opening direction.
     * Takes effect immediately via TMC2209 shaft() register.
     */
    void toggle_direction(void);

    /**
     * @brief Force motor_position to maximum_motor_position and save to NVS.
     *
     * Used by the long-press button callback to mark the current physical
     * position as fully closed (100%) without actually moving the motor.
     */
    void set_position_to_max(void);

    /**
     * @brief Get the current lift position in hundredths of a percent.
     * @return Position in the range 0 (fully open) to 10000 (fully closed).
     */
    uint16_t get_percent100ths(void);

    /**
     * @brief Get the target lift position in hundredths of a percent.
     * @return Target position in the range 0 (fully open) to 10000 (fully closed).
     */
    uint16_t get_target_percent100ths(void);

    /**
     * @brief Get the current lift position as an integer percentage.
     * @return Position in the range 0 (fully open) to 100 (fully closed).
     */
    uint8_t  get_lift_percentage(void);

    /**
     * @brief Check whether the motor is currently moving.
     * @return true if motor is in motion, false otherwise.
     */
    bool     get_is_moving(void)   { return is_moving; }

    /**
     * @brief Check whether the motor is moving in the close direction.
     * @return true if closing, false if opening or stopped.
     */
    bool     get_is_closing(void)  { return is_closing; }

    /**
     * @brief Get the maximum travel distance in INDEX pulses.
     * @return Maximum motor position (set during calibration).
     */
    int32_t  get_max_motor_position(void) { return maximum_motor_position; }

    /**
     * @brief Get the maximum lift distance in centimeters.
     *
     * Calculated from maximum_motor_position assuming 100 pulses/revolution
     * and a spool circumference of ~3.77 cm.
     *
     * @return Maximum lift distance in centimeters.
     */
    uint16_t get_max_lift_cm(void);

    /**
     * @brief Erase all motor-related NVS data (factory reset).
     * @return ESP_OK on success, or an NVS error code.
     */
    esp_err_t clean_nvs_values(void);

    /**
     * @brief Load motor position, max position, and direction from NVS.
     * @return ESP_OK on success, or an NVS error code (e.g., first boot).
     */
    esp_err_t load_preferences(void);

    /**
     * @brief Save the current lift percentage to NVS.
     * @param[in] percent Lift percentage (0-100).
     */
    void save_lift_percent(uint8_t percent);

    /**
     * @brief Save the maximum lift distance (in cm) to NVS.
     * @param[in] max_lift_cm Maximum lift distance in centimeters.
     */
    void save_max_lift(uint16_t max_lift_cm);
};
