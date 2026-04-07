/**
 * @file curtain_motor.cpp
 * @brief Implementation of the curtain_motor hardware driver.
 *
 * Manages the TMC2209 stepper motor driver over UART, GPIO-based step
 * counting via the INDEX pin, StallGuard obstacle detection, position
 * monitoring in a FreeRTOS task, and NVS persistence of all motor state.
 *
 * @see curtain_motor.h for the class interface and pin definitions.
 */

#include "curtain_motor.h"

#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <cmath>
#include <cstring>

static const char *TAG = "curtain_motor";

/** @brief Spinlock for protecting motor_position accessed from ISR context. */
static portMUX_TYPE motor_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* ========================================
 * CONSTRUCTOR
 * ======================================== */

curtain_motor::curtain_motor(motor_stop_callback_t on_motor_stop_cb)
    : driver(MOTOR_UART_NUM, R_SENSE, DRIVER_ADDRESS), position_watcher_task_handler(NULL), motor_position(0), target_position(0), maximum_motor_position(500), is_closing(false), is_moving(false), stop_flag(false), stall_flag(false), opening_direction(0), on_motor_stop(on_motor_stop_cb)
{
}

/* ========================================
 * INITIALIZATION
 * ======================================== */

esp_err_t curtain_motor::initialize(void)
{
    ESP_LOGI(TAG, "Initializing curtain motor system");

    /* Load saved preferences from NVS */
    load_preferences();

    /* Configure UART for TMC2209 communication */
    uart_config_t uart_config = {
        .baud_rate = MOTOR_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(MOTOR_UART_NUM, &uart_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(MOTOR_UART_NUM, MOTOR_TX_PIN, MOTOR_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_driver_install(MOTOR_UART_NUM, 256, 256, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Configure GPIO: Enable pin */
    gpio_config_t enable_conf = {
        .pin_bit_mask = (1ULL << ENABLE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&enable_conf);
    disable_driver(); /* Start with driver disabled */

    /* Configure GPIO: StallGuard pin (input, rising edge interrupt) */
    gpio_config_t stallguard_conf = {
        .pin_bit_mask = (1ULL << STALLGUARD_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&stallguard_conf);

    /* Configure GPIO: Index pin (input, rising edge interrupt) */
    gpio_config_t index_conf = {
        .pin_bit_mask = (1ULL << INDEX_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&index_conf);

    /* Install GPIO ISR service and attach interrupt handlers */
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(STALLGUARD_PIN, stall_interrupt, this);
    gpio_isr_handler_add(INDEX_PIN, index_interrupt, this);

    /* Configure TMC2209 driver */
    configure_tmc2209();

    /* Create position watcher task */
    BaseType_t result = xTaskCreate(
        position_watcher_task,
        "pos_watcher",
        4192,
        this,
        1,
        &position_watcher_task_handler);

    if (result != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create position watcher task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Motor system initialized. Position: %ld, Max: %ld, Direction: %d",
             (long)motor_position, (long)maximum_motor_position, opening_direction);

    return ESP_OK;
}

/* ========================================
 * TMC2209 CONFIGURATION
 * ======================================== */

/**
 * @brief Write the complete TMC2209 register configuration.
 *
 * Configures StealthChop (silent operation), INDEX pin as step output,
 * StallGuard threshold, CoolStep current scaling, chopper parameters,
 * and PWM settings. Register values match the original Arduino firmware.
 */
void curtain_motor::configure_tmc2209(void)
{
    ESP_LOGI(TAG, "Configuring TMC2209 driver");

    /* Set motor shaft direction */
    driver.shaft(opening_direction == 1);

    /* General Registers */
    driver.I_scale_analog(false);
    driver.internal_Rsense(false);
    driver.en_spreadCycle(false); /* Use StealthChop (silent mode) */
    driver.index_otpw(false);
    driver.index_step(true); /* INDEX pin outputs step pulses */
    driver.pdn_disable(true);
    driver.mstep_reg_select(true);
    driver.multistep_filt(true);

    /* NODECONF Registers */
    driver.senddelay(6);

    /* Factory Registers */
    driver.ottrim(0);

    /* Velocity Dependent Control */
    driver.ihold(0); /* No hold current at standstill */
    driver.iholddelay(1);
    driver.TPOWERDOWN(20);
    driver.TPWMTHRS(0);
    driver.VACTUAL(0);

    driver.irun(24); /* Max run current */
    driver.TCOOLTHRS(80);

    /* StallGuard and CoolStep configuration */
    driver.SGTHRS(120); /* StallGuard threshold */
    driver.semin(6);    /* CoolStep minimum current */
    driver.seup(0);
    driver.semax(0);
    driver.sedn(0);
    driver.seimin(1);

    /* CHOPCONF - Chopper Configuration */
    driver.diss2vs(0);
    driver.diss2g(0);
    driver.dedge(0);
    driver.intpol(1); /* Interpolate microsteps to 256 */
    driver.mres(8);   /* Microstep resolution: 8 = 100 index pulses/revolution */
    driver.vsense(0);
    driver.tbl(0); /* Blank time */
    driver.hend(0);
    driver.hstrt(5);
    driver.toff(3);

    /* PWMCONF - Voltage PWM Mode StealthChop */
    driver.pwm_lim(12);
    driver.pwm_reg(1);
    driver.freewheel(1);
    driver.pwm_autograd(1);
    driver.pwm_autoscale(1);
    driver.pwm_freq(1);
    driver.pwm_grad(0); /* PWM gradient */
    driver.pwm_ofs(36); /* PWM offset */

    ESP_LOGI(TAG, "TMC2209 configuration complete");
}

/* ========================================
 * INTERRUPTS (IRAM)
 * ======================================== */

/**
 * @brief INDEX pin ISR - counts motor steps.
 *
 * Increments motor_position when closing, decrements when opening.
 * Protected by a critical section (portMUX spinlock).
 *
 * @param[in] arg Pointer to the owning curtain_motor instance.
 */
void IRAM_ATTR curtain_motor::index_interrupt(void *arg)
{
    curtain_motor *self = static_cast<curtain_motor *>(arg);
    portENTER_CRITICAL_ISR(&motor_spinlock);
    if (self->operational_state == CLOSING)
    {
        self->motor_position++;
    }
    else if (self->operational_state == OPENING)
    {
        self->motor_position--;
    }
    portEXIT_CRITICAL_ISR(&motor_spinlock);
}

/**
 * @brief StallGuard pin ISR - flags obstacle detection.
 *
 * Sets stall_flag for the position_watcher_task to process on
 * its next polling cycle.
 *
 * @param[in] arg Pointer to the owning curtain_motor instance.
 */
void IRAM_ATTR curtain_motor::stall_interrupt(void *arg)
{
    curtain_motor *self = static_cast<curtain_motor *>(arg);
    self->stall_flag = true;
}

/* ========================================
 * DRIVER ENABLE/DISABLE
 * ======================================== */

void curtain_motor::enable_driver(void)
{
    gpio_set_level(ENABLE_PIN, 0);
}

void curtain_motor::disable_driver(void)
{
    gpio_set_level(ENABLE_PIN, 1);
}

/* ========================================
 * MOTOR STOP
 * ======================================== */

/**
 * @brief Immediately stop the motor by disabling the driver and zeroing VACTUAL.
 */
void curtain_motor::stop_motor(void)
{   
    operational_state = IDLE;
    disable_driver();
    driver.VACTUAL(STOP_VELOCITY);
    ESP_LOGI(TAG, "Motor stopped");
}

void curtain_motor::smooth_stop(void)
{
    stop_flag = true;
}

/* ========================================
 * MOVEMENT COMMANDS
 * ======================================== */

esp_err_t curtain_motor::move_to_percent100ths(uint16_t percent100ths)
{
    uint8_t lift_percent = percent100ths / 100;

    /* Calculate target position from percentage */
    target_position = ((float)lift_percent / 100.0f) * maximum_motor_position;

    /* Clamp motor position to valid range */
    if (motor_position > maximum_motor_position)
    {
        motor_position = maximum_motor_position;
    }
    if (motor_position < 0)
    {
        motor_position = 0;
    }

    ESP_LOGI(TAG, "Move to %d%% (target_pos=%ld, current_pos=%ld, max=%ld)",
             lift_percent, (long)target_position, (long)motor_position, (long)maximum_motor_position);

    if (target_position == motor_position)
    {
        ESP_LOGI(TAG, "Already at desired position");
        return ESP_OK;
    }

    if (target_position > motor_position)
    {
        /* Need to close */
        ESP_LOGI(TAG, "Moving: CLOSING");
        stop_flag = false;
        stall_flag = false;
        operational_state = CLOSING;

        is_closing = true;
        is_moving = true;

        vTaskResume(position_watcher_task_handler);
        vTaskDelay(pdMS_TO_TICKS(100));
        enable_driver();

        // Acceleration
        for (int i = 0; i >= CLOSE_VELOCITY; i = i - 1)
        {
            driver.VACTUAL(i);
            vTaskDelay(pdMS_TO_TICKS(10));
            // Need to stop the acceleration if stop flag is set
            // Buttons don't work while in the loop
            if (operational_state == IDLE)
            {
                driver.VACTUAL(STOP_VELOCITY);
                return ESP_OK;
            }
        }
        driver.VACTUAL(CLOSE_VELOCITY);
    }
    else if (target_position < motor_position)
    {
        /* Need to open */
        ESP_LOGI(TAG, "Moving: OPENING");
        stop_flag = false;
        stall_flag = false;
        operational_state = OPENING;
        is_closing = false;
        is_moving = true;

        vTaskResume(position_watcher_task_handler);
        vTaskDelay(pdMS_TO_TICKS(100));
        enable_driver();

        // Acceleration

        // Buttons don't work while in the loop
        
        for (int i = 0; i <= OPEN_VELOCITY; i = i + 1)
        {
            driver.VACTUAL(i);
            vTaskDelay(pdMS_TO_TICKS(10));
            
            // Need to stop the acceleration if stop flag is set
            if (operational_state == IDLE)
            {
                driver.VACTUAL(STOP_VELOCITY);
                return ESP_OK;
            }
        }
        driver.VACTUAL(OPEN_VELOCITY);
    }

    return ESP_OK;
}

/* ========================================
 * POSITION WATCHER TASK
 * ======================================== */

/**
 * @brief FreeRTOS task that monitors motor position during movement.
 *
 * Lifecycle:
 * 1. Suspends itself immediately on entry (and after each movement completes).
 * 2. Resumed by move_to_percent100ths() when a new movement starts.
 * 3. Polls every 20 ms checking: stop_flag, stall_flag, and target reached.
 * 4. On any stop condition: disables driver, waits 1000 ms for deceleration,
 *    saves state to NVS, and invokes the on_motor_stop callback.
 *
 * @param[in] param Pointer to the owning curtain_motor instance.
 */
void curtain_motor::position_watcher_task(void *param)
{
    curtain_motor *self = static_cast<curtain_motor *>(param);
    int loop_counter = 0;

    ESP_LOGI(TAG, "Position watcher task created");

    while (true)
    {
        vTaskSuspend(NULL);

        ESP_LOGI(TAG, "Position watcher resumed, state=%d", self->operational_state);

        while (self->operational_state != IDLE)
        {
            loop_counter++;

            /* Check if stop was requested */
            if (self->stop_flag)
            {
                self->stop_motor();
                self->stop_flag = false;
                ESP_LOGI(TAG, "Position watcher: Stop requested");
                vTaskDelay(pdMS_TO_TICKS(1000));
                goto notify_and_suspend;
            }

            /* Check for stall condition */
            if (self->stall_flag)
            {
                self->stop_motor();
                self->stall_flag = false;
                ESP_LOGW(TAG, "Position watcher: Stall detected!");
                goto notify_and_suspend;
            }

            /* Check if target position reached */
            if (self->operational_state == CLOSING)
            {
                if (self->motor_position >= self->target_position)
                {
                    ESP_LOGI(TAG, "Target reached (closing) - pos: %ld, target: %ld",
                             (long)self->motor_position, (long)self->target_position);
                    self->stop_motor();
                    goto notify_and_suspend;
                }
            }
            else if (self->operational_state == OPENING)
            {
                if (self->motor_position <= self->target_position)
                {
                    ESP_LOGI(TAG, "Target reached (opening) - pos: %ld, target: %ld",
                             (long)self->motor_position, (long)self->target_position);
                    self->stop_motor();
                    goto notify_and_suspend;
                }
            }

            /* Periodic position logging */
            if (loop_counter >= 50)
            {
                ESP_LOGD(TAG, "Motor position: %ld", (long)self->motor_position);
                loop_counter = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }

    notify_and_suspend:
        self->is_moving = false;
        self->operational_state = IDLE;

        /* Calculate current lift percentage */
        uint16_t current_percent100ths = self->get_percent100ths();
        uint8_t current_percent = current_percent100ths / 100;

        ESP_LOGI(TAG, "Movement complete - Position: %ld, Lift: %d%%, Target: %ld, Max: %ld",
                 (long)self->motor_position, current_percent,
                 (long)self->target_position, (long)self->maximum_motor_position);

        /* Save state to NVS */
        self->save_motor_position();
        self->save_lift_percent(current_percent);

        /* Notify application layer */
        if (self->on_motor_stop)
        {
            self->on_motor_stop(current_percent100ths);
        }

        loop_counter = 0;
    }
}

/* ========================================
 * CALIBRATION / SET DISTANCE MODE
 * ======================================== */

void curtain_motor::start_set_distance_open(void)
{
    ESP_LOGI(TAG, "Starting set_distance mode: OPENING");
    operational_state = OPENING;
    is_closing = false;
    is_moving = true;
    motor_position = 0; /* Reset to count from zero */

    enable_driver();
    driver.VACTUAL(CALIBRATION_OPEN_VELOCITY);
}

void curtain_motor::start_set_distance_close(void)
{
    ESP_LOGI(TAG, "Starting set_distance mode: CLOSING");
    operational_state = CLOSING;
    is_closing = true;
    is_moving = true;

    enable_driver();
    driver.VACTUAL(CALIBRATION_CLOSE_VELOCITY);
}

/**
 * @brief Finish set-distance calibration (open direction).
 *
 * Stops the motor and calculates:
 * - maximum_motor_position from the absolute distance traveled
 * - max_lift_cm from revolutions * spool circumference (~3.77 cm)
 *
 * Resets motor_position to 0 (fully open) and saves all values to NVS.
 */
void curtain_motor::finish_set_distance_open(void)
{
    disable_driver();
    driver.VACTUAL(STOP_VELOCITY);
    operational_state = IDLE;
    is_moving = false;

    /* The motor_position now holds the travel distance */
    maximum_motor_position = abs(motor_position);

    /* Reset position to 0 (fully open) */
    motor_position = 0;

    /* Calculate max lift in centimeters (100 pulses/rev, ~3.77 cm/rev spool circumference) */
    float revolutions = (float)maximum_motor_position / 100.0f;
    uint16_t max_lift_cm = (uint16_t)(revolutions * 3.7699f);

    /* Save to NVS */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_i32(nvs, NVS_MAX_POS_KEY, maximum_motor_position);
        nvs_set_i32(nvs, NVS_MOTOR_POS_KEY, motor_position);
        nvs_set_i32(nvs, NVS_MAX_LIFT_KEY, max_lift_cm);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "Set distance OPEN complete: max_pos=%ld, max_lift=%d cm",
             (long)maximum_motor_position, max_lift_cm);
}

/**
 * @brief Finish set-distance calibration (close direction).
 *
 * Stops the motor and resets motor_position to 0 (fully closed reference point).
 * Saves the new position to NVS.
 */
void curtain_motor::finish_set_distance_close(void)
{
    disable_driver();
    driver.VACTUAL(STOP_VELOCITY);
    operational_state = IDLE;
    is_moving = false;

    /* Reset motor position to maximum motor position (fully closed reference) */
    motor_position = maximum_motor_position;

    /* Save to NVS */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_i32(nvs, NVS_MOTOR_POS_KEY, motor_position);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "Set distance CLOSE complete: motor_position reset to maximum_motor_position: (%ld)", (long)motor_position);
}

/* ========================================
 * POSITION OVERRIDE
 * ======================================== */

void curtain_motor::set_position_to_max(void)
{
    motor_position = maximum_motor_position;
    save_motor_position();
    ESP_LOGI(TAG, "Motor position forced to max: %ld", (long)motor_position);
}

/* ========================================
 * DIRECTION CONTROL
 * ======================================== */

void curtain_motor::toggle_direction(void)
{
    if (opening_direction == 0)
    {
        opening_direction = 1;
        driver.shaft(true);
    }
    else
    {
        opening_direction = 0;
        driver.shaft(false);
    }

    /* Save to NVS */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_i32(nvs, NVS_OPEN_DIR_KEY, opening_direction);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "Direction toggled to %d", opening_direction);
}

/* ========================================
 * GETTERS
 * ======================================== */

uint16_t curtain_motor::get_percent100ths(void)
{
    if (maximum_motor_position <= 0)
    {
        return 10000; /* Default to fully closed */
    }

    float pct = ((float)motor_position / (float)maximum_motor_position) * 100.0f;

    /* Clamp to 0-100 */
    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;

    return (uint16_t)(pct * 100.0f); /* Return in percent100ths (0-10000) */
}

uint16_t curtain_motor::get_target_percent100ths(void)
{
    if (maximum_motor_position <= 0)
    {
        return 10000; /* Default to fully closed */
    }

    float pct = ((float)target_position / (float)maximum_motor_position) * 100.0f;

    /* Clamp to 0-100 */
    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;

    return (uint16_t)(pct * 100.0f); /* Return in percent100ths (0-10000) */
}

uint8_t curtain_motor::get_lift_percentage(void)
{
    return get_percent100ths() / 100;
}

/**
 * @brief Calculate max lift distance in centimeters from maximum_motor_position.
 *
 * Assumes 100 INDEX pulses per revolution and a spool circumference
 * of approximately 3.7699 cm (1.2 cm diameter spool).
 *
 * @return Maximum lift distance in centimeters.
 */
uint16_t curtain_motor::get_max_lift_cm(void)
{
    float revolutions = (float)maximum_motor_position / 100.0f;
    return (uint16_t)(revolutions * 3.7699f);
}

/* ========================================
 * NVS PERSISTENCE
 * ======================================== */

esp_err_t curtain_motor::load_preferences(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS open failed (first boot?): %s", esp_err_to_name(err));
        return err;
    }

    int32_t val;

    if (nvs_get_i32(nvs, NVS_MAX_POS_KEY, &val) == ESP_OK)
    {
        maximum_motor_position = val;
    }
    if (nvs_get_i32(nvs, NVS_MOTOR_POS_KEY, &val) == ESP_OK)
    {
        motor_position = val;
    }
    if (nvs_get_i32(nvs, NVS_OPEN_DIR_KEY, &val) == ESP_OK)
    {
        opening_direction = (uint8_t)val;
    }

    nvs_close(nvs);

    ESP_LOGI(TAG, "Preferences loaded: pos=%ld, max=%ld, dir=%d",
             (long)motor_position, (long)maximum_motor_position, opening_direction);

    return ESP_OK;
}

void curtain_motor::save_motor_position(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_i32(nvs, NVS_MOTOR_POS_KEY, motor_position);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

void curtain_motor::save_lift_percent(uint8_t percent)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_i32(nvs, NVS_LIFT_PERCENT_KEY, percent);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

void curtain_motor::save_max_lift(uint16_t max_lift_cm)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK)
    {
        nvs_set_i32(nvs, NVS_MAX_LIFT_KEY, max_lift_cm);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

esp_err_t curtain_motor::clean_nvs_values(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK)
        return err;

    nvs_erase_all(nvs);
    nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "NVS values cleared");
    return ESP_OK;
}
