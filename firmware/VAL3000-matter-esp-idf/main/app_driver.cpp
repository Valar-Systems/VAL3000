/**
 * @file app_driver.cpp
 * @brief Application driver: Matter callbacks, button handling, and attribute reporting.
 *
 * Implements the bridge between the Matter application layer and the curtain_motor
 * hardware driver. Contains:
 * - A FreeRTOS queue-based Matter attribute reporting system
 * - Motor event callbacks (stop, stall)
 * - Matter attribute PRE_UPDATE and READ handlers
 * - Button callbacks for 3-button control (close, open, function/reset)
 * - Commissioning window management
 *
 * Converted from the Arduino VAL3000-matter-arduino project.
 */

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <esp_matter.h>
#include <iot_button.h>

#include <app_priv.h>
#include <app/clusters/window-covering-server/window-covering-server.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <app-common/zap-generated/attributes/Accessors.h>

static const char *TAG = "app_driver";

using chip::app::CommandHandler;
using chip::Protocols::InteractionModel::Status;
using namespace chip::app::Clusters;
using namespace esp_matter;

/* External references */
extern uint16_t curtain_endpoint_id;

/** @brief Global motor handle, set during init_buttons(). */
static curtain_motor *g_motor_handle = NULL;

/**
 * @brief Guard flag to prevent single-click from firing after a press-down stop.
 *
 * When a press-down stops the motor, pressdown_timer is set 1.5 s into the
 * future. Single-click callbacks check this timer and return early if still
 * within the guard window.
 */
static bool pressdown = true;

/** @brief Timestamp (in microseconds) until which single-click is suppressed. */
static int64_t pressdown_timer = 0;

/** @brief Duration of the pressdown guard window in microseconds (1.5 s). */
#define PRESSDOWN_DELAY_US  1500000

/** @brief True when the device is in set-distance calibration mode. */
static bool set_distance = false;

/** @brief True when a factory reset has been triggered (awaiting button release). */
static bool perform_factory_reset = false;

/** @brief Timestamp of the last Button 3 press-down, for 5-second hold detection. */
static int64_t press_millis = 0;

/* ========================================
 * MATTER ATTRIBUTE REPORTING QUEUE
 * ======================================== */

/**
 * @brief Payload for a single queued Matter attribute report/update.
 *
 * use_update=false → attribute::report() (safe without CHIP lock; for OVERRIDE attributes).
 * use_update=true  → attribute::update() with CHIP lock (writes to attribute store; for non-OVERRIDE attributes like OperationalStatus).
 */
typedef struct {
    uint16_t endpoint_id;           /**< Matter endpoint ID. */
    uint32_t cluster_id;            /**< Cluster ID. */
    uint32_t attribute_id;          /**< Attribute ID. */
    esp_matter_attr_val_t val;      /**< Attribute value to report/update. */
    bool use_update;                /**< If true, use attribute::update(); otherwise attribute::report(). */
} matter_report_event_t;

/** @brief FreeRTOS queue holding pending attribute reports. */
static QueueHandle_t matter_report_queue = NULL;

/**
 * @brief FreeRTOS task that dequeues and delivers Matter attribute reports.
 *
 * Runs at tskIDLE_PRIORITY + 2. Blocks on the queue indefinitely and calls
 * esp_matter::attribute::report() for each received event.
 *
 * @param[in] arg Unused.
 */
static void matter_report_task(void *arg)
{
    matter_report_event_t event;

    while (true) {
        if (xQueueReceive(matter_report_queue, &event, portMAX_DELAY)) {
            if (event.use_update) {
                /* attribute::update() writes to the attribute store and notifies
                 * subscribers. Requires the CHIP stack lock. Used for non-OVERRIDE
                 * attributes such as OperationalStatus and CurrentPosition. */
                chip::DeviceLayer::PlatformMgr().LockChipStack();
                attribute::update(event.endpoint_id, event.cluster_id,
                                  event.attribute_id, &event.val);
                chip::DeviceLayer::PlatformMgr().UnlockChipStack();
            } else {
                /* attribute::report() reads from the OVERRIDE callback and notifies
                 * subscribers. Does not require the CHIP stack lock. */
                attribute::report(event.endpoint_id, event.cluster_id,
                                  event.attribute_id, &event.val);
            }
        }
    }
}

esp_err_t init_matter_reporting(void)
{
    matter_report_queue = xQueueCreate(20, sizeof(matter_report_event_t));
    if (matter_report_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create Matter report queue");
        return ESP_FAIL;
    }

    BaseType_t result = xTaskCreate(matter_report_task, "matter_report", 4096, NULL,
                                    tskIDLE_PRIORITY + 2, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Matter reporting task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t queue_attribute_report(uint16_t endpoint_id, uint32_t cluster_id,
                                  uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    matter_report_event_t event = {
        .endpoint_id = endpoint_id,
        .cluster_id = cluster_id,
        .attribute_id = attribute_id,
        .val = *val,
        .use_update = false,
    };

    if (xQueueSend(matter_report_queue, &event, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send Matter report event to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t queue_attribute_update(uint16_t endpoint_id, uint32_t cluster_id,
                                  uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    matter_report_event_t event = {
        .endpoint_id = endpoint_id,
        .cluster_id = cluster_id,
        .attribute_id = attribute_id,
        .val = *val,
        .use_update = true,
    };

    if (xQueueSend(matter_report_queue, &event, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send Matter update event to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ========================================
 * MOTOR EVENT CALLBACKS
 * (Called by curtain_motor when motor stops - including stall stops)
 * ======================================== */

void on_motor_stop(uint16_t percent100ths)
{
    ESP_LOGI(TAG, "Motor stopped at %d.%02d%%", percent100ths / 100, percent100ths % 100);

    esp_matter_attr_val_t value;

    /* Update CurrentPositionLiftPercentage — uses attribute::update() to write the
     * value to the store and trigger subscription notifications. attribute::report()
     * is insufficient here because the controller may have a cached value and not
     * refresh until the store changes. */
    value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT8;
    value.val.u8 = percent100ths / 100;
    queue_attribute_update(curtain_endpoint_id, WindowCovering::Id,
                           WindowCovering::Attributes::CurrentPositionLiftPercentage::Id, &value);

    /* Update CurrentPositionLiftPercent100ths */
    value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
    value.val.u16 = percent100ths;
    queue_attribute_update(curtain_endpoint_id, WindowCovering::Id,
                           WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, &value);

    /* Notify subscribers that TargetPosition = CurrentPosition.
     * Uses attribute::report() (NOT attribute::update()) to push a subscription
     * notification without writing to the store and without triggering PRE_UPDATE
     * (which calls move_to_percent100ths and would restart the motor).
     *
     * The app caches TargetPosition from the last GoToLiftPercentage command or
     * button-click notification (e.g. 10000 for a close command). Without this
     * report, the app sees current(1341) < cached_target(10000) and keeps showing
     * "closing..." even though OperationalStatus=Stall is set correctly.
     *
     * Because TargetPositionLiftPercent100ths has ATTRIBUTE_FLAG_OVERRIDE, the
     * read callback is called when the report fires. When the motor is stopped,
     * the callback returns motor->get_percent100ths() = current position, so the
     * app receives target == current and clears the "closing..."/"opening..." state.
     *
     * The WC server already triggered "Lift stop" via its own PostAttributeChange
     * logic (reading the callback during CurrentPositionLiftPercent100ths write);
     * this report only updates the subscriber cache on the controller side. */
    value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
    value.val.u16 = percent100ths;
    queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                           WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id, &value);

    /* Update OperationalStatus: Stall (stopped).
     * Uses attribute::update() to write to the store (non-OVERRIDE attribute).
     * Matches Arduino: WindowBlinds.setOperationalState(LIFT, STALL) */
    chip::BitMask<WindowCovering::OperationalStatus> operational_status;
    operational_status.SetField(WindowCovering::OperationalStatus::kLift,
                                static_cast<uint8_t>(WindowCovering::OperationalState::Stall));
    operational_status.SetField(WindowCovering::OperationalStatus::kGlobal,
                                static_cast<uint8_t>(WindowCovering::OperationalState::Stall));
    value.type = ESP_MATTER_VAL_TYPE_BITMAP8;
    value.val.u8 = operational_status.Raw();
    queue_attribute_update(curtain_endpoint_id, WindowCovering::Id,
                           WindowCovering::Attributes::OperationalStatus::Id, &value);

}

/* ========================================
 * MATTER ATTRIBUTE CALLBACKS
 * ======================================== */

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (endpoint_id == curtain_endpoint_id) {
        curtain_motor *motor = (curtain_motor *)priv_data;
        switch (cluster_id) {
        case WindowCovering::Id:
            switch (attribute_id) {
            case WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id:
                ESP_LOGI(TAG, "TargetPositionLiftPercent100ths: %d", val->val.u16);
                if (nullable<uint16_t>(val->val.u16).is_null()) {
                    motor->smooth_stop();
                } else {
                    err = motor->move_to_percent100ths(val->val.u16);
                    /* OperationalStatus is managed by the WC server internally
                     * (see log: "OperationalStatus raw=0x0A global=2 lift=2 tilt=0")
                     * and by on_motor_stop when movement completes. No need to
                     * queue a redundant report here — matches Arduino where
                     * goToLiftPercentage sets it synchronously via setOperationalState. */
                }
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
    }
    return err;
}

esp_err_t app_driver_attribute_read(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                    uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    if (endpoint_id == curtain_endpoint_id) {
        curtain_motor *motor = (curtain_motor *)priv_data;
        switch (cluster_id) {
        case WindowCovering::Id:
            switch (attribute_id) {
            case WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id:
                val->type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
                val->val.u16 = motor->get_percent100ths();
                break;
            case WindowCovering::Attributes::CurrentPositionLiftPercentage::Id:
                val->type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT8;
                val->val.u8 = motor->get_lift_percentage();
                break;
            case WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id:
                val->type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
                if (motor->get_is_moving()) {
                    val->val.u16 = motor->get_target_percent100ths();
                } else {
                    val->val.u16 = motor->get_percent100ths();
                }
                break;
            case WindowCovering::Attributes::OperationalStatus::Id:
            {
                chip::BitMask<WindowCovering::OperationalStatus> operational_status;
                uint8_t state = motor->get_is_moving()
                    ? (motor->get_is_closing()
                        ? static_cast<uint8_t>(WindowCovering::OperationalState::MovingDownOrClose)
                        : static_cast<uint8_t>(WindowCovering::OperationalState::MovingUpOrOpen))
                    : static_cast<uint8_t>(WindowCovering::OperationalState::Stall);
                operational_status.SetField(WindowCovering::OperationalStatus::kLift, state);
                operational_status.SetField(WindowCovering::OperationalStatus::kGlobal, state);
                val->type = ESP_MATTER_VAL_TYPE_BITMAP8;
                val->val.u8 = operational_status.Raw();
            }
            break;
            default:
                break;
            }
            break;
        default:
            break;
        }
    }

    return ESP_OK;
}

/* ========================================
 * BUTTON CALLBACKS
 *
 * Button 1 (Close): GPIO 4
 *   - press_down: stop if moving; if set_distance close mode, finish calibration
 *   - single_click: move to 100% (closed)
 *   - double_click: start set_distance close mode
 *   - long_press: set current position as fully closed (100%)
 *
 * Button 2 (Open): GPIO 3
 *   - press_down: stop if moving; if set_distance open mode, finish calibration
 *   - single_click: move to 0% (open)
 *   - double_click: start set_distance open mode
 *
 * Button 3 (Function): GPIO 7
 *   - double_click: toggle motor direction
 *   - long_press (5s hold + release): factory reset / decommission
 * ======================================== */

/* --- BUTTON 1 (Close) --- */

/**
 * @brief Button 1 press-down callback.
 *
 * If the motor is moving, performs an immediate stop and arms the pressdown
 * guard timer. In set_distance close mode, finalizes calibration and reports
 * 100% (fully closed) position to Matter.
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Pointer to the curtain_motor instance.
 */
static void btn1_press_down_cb(void *button_handle, void *usr_data)
{
    curtain_motor *motor = (curtain_motor *)usr_data;
    ESP_LOGI(TAG, "Button1 press down");

    if (motor->get_is_moving()) {
        ESP_LOGI(TAG, "Stop Flag - IMMEDIATE STOP");
        motor->smooth_stop();

        pressdown = false;
        pressdown_timer = esp_timer_get_time() + PRESSDOWN_DELAY_US;

        if (set_distance) {
            motor->finish_set_distance_close();
            set_distance = false;

            /* Report 100% (fully closed) to Matter */
            esp_matter_attr_val_t value;
            value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
            value.val.u16 = 10000;
            queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                                   WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, &value);

            value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT8;
            value.val.u8 = 100;
            queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                                   WindowCovering::Attributes::CurrentPositionLiftPercentage::Id, &value);

            /* Update OperationalStatus: Stall (stopped) */
            chip::BitMask<WindowCovering::OperationalStatus> status;
            status.SetField(WindowCovering::OperationalStatus::kLift,
                            static_cast<uint8_t>(WindowCovering::OperationalState::Stall));
            status.SetField(WindowCovering::OperationalStatus::kGlobal,
                            static_cast<uint8_t>(WindowCovering::OperationalState::Stall));
            value.type = ESP_MATTER_VAL_TYPE_BITMAP8;
            value.val.u8 = status.Raw();
            queue_attribute_update(curtain_endpoint_id, WindowCovering::Id,
                                   WindowCovering::Attributes::OperationalStatus::Id, &value);
        }
    }
}

/**
 * @brief Button 1 single-click callback.
 *
 * Moves the curtain to 100% (fully closed), or stops if already moving.
 * Suppressed for 1.5 s after a press-down stop to avoid unintended triggers.
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Pointer to the curtain_motor instance.
 */
static void btn1_single_click_cb(void *button_handle, void *usr_data)
{
    curtain_motor *motor = (curtain_motor *)usr_data;
    ESP_LOGI(TAG, "Button1 single click");

    /* Check pressdown guard */
    if (esp_timer_get_time() < pressdown_timer) {
        return;
    }

    if (motor->get_is_moving()) {
        motor->smooth_stop();
    } else {
        motor->move_to_percent100ths(10000);

        /* Notify Matter that movement started (WC server does this automatically
         * for GoToLiftPercentage commands, but buttons bypass the WC server). */
        esp_matter_attr_val_t value;
        value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
        value.val.u16 = 10000;
        queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                               WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id, &value);

        chip::BitMask<WindowCovering::OperationalStatus> status;
        status.SetField(WindowCovering::OperationalStatus::kLift,
                        static_cast<uint8_t>(WindowCovering::OperationalState::MovingDownOrClose));
        status.SetField(WindowCovering::OperationalStatus::kGlobal,
                        static_cast<uint8_t>(WindowCovering::OperationalState::MovingDownOrClose));
        value.type = ESP_MATTER_VAL_TYPE_BITMAP8;
        value.val.u8 = status.Raw();
        queue_attribute_update(curtain_endpoint_id, WindowCovering::Id,
                               WindowCovering::Attributes::OperationalStatus::Id, &value);
    }
}

/**
 * @brief Button 1 double-click callback.
 *
 * Enters set-distance calibration mode for the close direction. The motor
 * begins closing and will be stopped by a subsequent press-down on Button 1.
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Pointer to the curtain_motor instance.
 */
static void btn1_double_click_cb(void *button_handle, void *usr_data)
{
    curtain_motor *motor = (curtain_motor *)usr_data;
    ESP_LOGI(TAG, "Button1 double click - set_distance CLOSE mode");

    set_distance = true;

    /* Match Arduino: WindowBlinds.setTargetLiftPercent100ths(100*100); then start motor.
     * The TargetPosition READ callback returns the actual target from the driver. */
    motor->start_set_distance_close();
}

/**
 * @brief Button 1 long-press callback.
 *
 * Overrides the current motor position to maximum_motor_position (fully closed)
 * without moving the motor. Useful for manual calibration when the curtain is
 * physically at the closed limit.
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Pointer to the curtain_motor instance.
 */
static void btn1_long_press_cb(void *button_handle, void *usr_data)
{
    curtain_motor *motor = (curtain_motor *)usr_data;
    ESP_LOGI(TAG, "Button1 long press - set current as fully closed");

    /* Force motor_position = maximum_motor_position and save to NVS
     * (matches Arduino: motor_position = maximum_motor_position; preferences.putInt(...)) */
    motor->set_position_to_max();

    /* Report 100% to Matter */
    esp_matter_attr_val_t value;
    value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
    value.val.u16 = 10000;
    queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                           WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, &value);

    value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT8;
    value.val.u8 = 100;
    queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                           WindowCovering::Attributes::CurrentPositionLiftPercentage::Id, &value);

    /* Update OperationalStatus: Stall (stopped) */
    chip::BitMask<WindowCovering::OperationalStatus> status;
    status.SetField(WindowCovering::OperationalStatus::kLift,
                    static_cast<uint8_t>(WindowCovering::OperationalState::Stall));
    status.SetField(WindowCovering::OperationalStatus::kGlobal,
                    static_cast<uint8_t>(WindowCovering::OperationalState::Stall));
    value.type = ESP_MATTER_VAL_TYPE_BITMAP8;
    value.val.u8 = status.Raw();
    queue_attribute_update(curtain_endpoint_id, WindowCovering::Id,
                           WindowCovering::Attributes::OperationalStatus::Id, &value);
}

/* --- BUTTON 2 (Open) --- */

/**
 * @brief Button 2 press-down callback.
 *
 * If the motor is moving, performs an immediate stop and arms the pressdown
 * guard timer. In set_distance open mode, finalizes calibration, updates
 * InstalledClosedLimitLift, and reports 0% (fully open) to Matter. Otherwise,
 * reports STALL operational status.
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Pointer to the curtain_motor instance.
 */
static void btn2_press_down_cb(void *button_handle, void *usr_data)
{
    curtain_motor *motor = (curtain_motor *)usr_data;
    ESP_LOGI(TAG, "Button2 press down");

    if (motor->get_is_moving()) {
        ESP_LOGI(TAG, "Stop Flag - IMMEDIATE STOP");
        motor->smooth_stop();

        pressdown = false;
        pressdown_timer = esp_timer_get_time() + PRESSDOWN_DELAY_US;

        if (set_distance) {
            motor->finish_set_distance_open();
            set_distance = false;

            /* Report 0% (fully open) to Matter */
            esp_matter_attr_val_t value;
            value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
            value.val.u16 = 0;
            queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                                   WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, &value);

            value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT8;
            value.val.u8 = 0;
            queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                                   WindowCovering::Attributes::CurrentPositionLiftPercentage::Id, &value);

            /* Update OperationalStatus: Stall (stopped) */
            chip::BitMask<WindowCovering::OperationalStatus> status;
            status.SetField(WindowCovering::OperationalStatus::kLift,
                            static_cast<uint8_t>(WindowCovering::OperationalState::Stall));
            status.SetField(WindowCovering::OperationalStatus::kGlobal,
                            static_cast<uint8_t>(WindowCovering::OperationalState::Stall));
            value.type = ESP_MATTER_VAL_TYPE_BITMAP8;
            value.val.u8 = status.Raw();
            queue_attribute_update(curtain_endpoint_id, WindowCovering::Id,
                                   WindowCovering::Attributes::OperationalStatus::Id, &value);

            /* Update InstalledClosedLimitLift with the new max lift distance.
             * Must hold CHIP stack lock - direct SDK call to attribute storage. */
            chip::DeviceLayer::PlatformMgr().LockChipStack();
            WindowCovering::Attributes::InstalledClosedLimitLift::Set(
                curtain_endpoint_id, motor->get_max_lift_cm());
            chip::DeviceLayer::PlatformMgr().UnlockChipStack();

            ESP_LOGI(TAG, "Set distance OPEN complete. Max lift: %d cm", motor->get_max_lift_cm());
        }
        /* Normal (non-calibration) stop: on_motor_stop fires from position_watcher_task
         * after the motor decelerates (1 s), reporting position + OperationalStatus=Stall
         * as one coherent update. No pre-emptive report needed here. */
    }
}

/**
 * @brief Button 2 single-click callback.
 *
 * Moves the curtain to 0% (fully open), or stops if already moving.
 * Suppressed for 1.5 s after a press-down stop.
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Pointer to the curtain_motor instance.
 */
static void btn2_single_click_cb(void *button_handle, void *usr_data)
{
    curtain_motor *motor = (curtain_motor *)usr_data;
    ESP_LOGI(TAG, "Button2 single click");

    /* Check pressdown guard */
    if (esp_timer_get_time() < pressdown_timer) {
        return;
    }

    if (motor->get_is_moving()) {
        motor->smooth_stop();
    } else {
        motor->move_to_percent100ths(0);

        /* Notify Matter that movement started. */
        esp_matter_attr_val_t value;
        value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
        value.val.u16 = 0;
        queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                               WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id, &value);

        chip::BitMask<WindowCovering::OperationalStatus> status;
        status.SetField(WindowCovering::OperationalStatus::kLift,
                        static_cast<uint8_t>(WindowCovering::OperationalState::MovingUpOrOpen));
        status.SetField(WindowCovering::OperationalStatus::kGlobal,
                        static_cast<uint8_t>(WindowCovering::OperationalState::MovingUpOrOpen));
        value.type = ESP_MATTER_VAL_TYPE_BITMAP8;
        value.val.u8 = status.Raw();
        queue_attribute_update(curtain_endpoint_id, WindowCovering::Id,
                               WindowCovering::Attributes::OperationalStatus::Id, &value);
    }
}

/**
 * @brief Button 2 double-click callback.
 *
 * Enters set-distance calibration mode for the open direction. The motor
 * begins opening and will be stopped by a subsequent press-down on Button 2.
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Pointer to the curtain_motor instance.
 */
static void btn2_double_click_cb(void *button_handle, void *usr_data)
{
    curtain_motor *motor = (curtain_motor *)usr_data;
    ESP_LOGI(TAG, "Button2 double click - set_distance OPEN mode");

    set_distance = true;

    /* Match Arduino: WindowBlinds.setTargetLiftPercent100ths(0*100); then start motor.
     * The TargetPosition READ callback returns the actual target from the driver. */
    motor->start_set_distance_open();
}

/* --- BUTTON 3 (Function/Reset) --- */

/**
 * @brief Button 3 double-click callback.
 *
 * Toggles the motor opening direction and saves to NVS.
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Pointer to the curtain_motor instance.
 */
static void btn3_double_click_cb(void *button_handle, void *usr_data)
{
    curtain_motor *motor = (curtain_motor *)usr_data;
    ESP_LOGI(TAG, "Button3 double click - toggle direction");

    motor->toggle_direction();
}

/**
 * @brief Button 3 press-down callback.
 *
 * Records the press timestamp for the 5-second hold detection used by
 * btn3_long_press_hold_cb().
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Unused.
 */
static void btn3_press_down_cb(void *button_handle, void *usr_data)
{
    press_millis = esp_timer_get_time();
}

/**
 * @brief Button 3 long-press hold callback (called periodically while held).
 *
 * After the button has been held for 5 seconds, sets perform_factory_reset.
 * The actual reset is triggered on button release (btn3_press_up_cb).
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Unused.
 */
static void btn3_long_press_hold_cb(void *button_handle, void *usr_data)
{
    int64_t now = esp_timer_get_time();

    /* Count for 5 seconds (in microseconds) */
    if (now >= (press_millis + 5000000)) {
        if (!perform_factory_reset) {
            ESP_LOGI(TAG, "Factory reset triggered. Release button to confirm.");
            perform_factory_reset = true;
        }
    }
}

/**
 * @brief Button 3 release callback.
 *
 * If perform_factory_reset was set by the hold callback, erases motor NVS
 * data and performs a Matter factory reset. Otherwise does nothing.
 *
 * @param[in] button_handle iot_button handle (unused).
 * @param[in] usr_data      Pointer to the curtain_motor instance.
 */
static void btn3_press_up_cb(void *button_handle, void *usr_data)
{
    curtain_motor *motor = (curtain_motor *)usr_data;
    ESP_LOGI(TAG, "Button3 released");

    if (perform_factory_reset) {
        ESP_LOGI(TAG, "Starting factory reset");
        motor->clean_nvs_values();
        esp_matter::factory_reset();
        perform_factory_reset = false;
    }
}

/* ========================================
 * BUTTON INITIALIZATION
 * ======================================== */

esp_err_t init_buttons(curtain_motor *motor_handle)
{
    g_motor_handle = motor_handle;

    /* --- Button 1: Close (GPIO 4) --- */
    button_config_t btn1_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 3000,
        .short_press_time = 180,
        .gpio_button_config = {
            .gpio_num = BUTTON_CLOSE_PIN,
            .active_level = 0,
        },
    };

    button_handle_t btn1 = iot_button_create(&btn1_cfg);
    if (!btn1) {
        ESP_LOGE(TAG, "Button 1 create failed");
        return ESP_FAIL;
    }

    iot_button_register_cb(btn1, BUTTON_PRESS_DOWN, btn1_press_down_cb, motor_handle);
    iot_button_register_cb(btn1, BUTTON_SINGLE_CLICK, btn1_single_click_cb, motor_handle);
    iot_button_register_cb(btn1, BUTTON_DOUBLE_CLICK, btn1_double_click_cb, motor_handle);
    iot_button_register_cb(btn1, BUTTON_LONG_PRESS_START, btn1_long_press_cb, motor_handle);

    /* --- Button 2: Open (GPIO 3) --- */
    button_config_t btn2_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 3000,
        .short_press_time = 180,
        .gpio_button_config = {
            .gpio_num = BUTTON_OPEN_PIN,
            .active_level = 0,
        },
    };

    button_handle_t btn2 = iot_button_create(&btn2_cfg);
    if (!btn2) {
        ESP_LOGE(TAG, "Button 2 create failed");
        return ESP_FAIL;
    }

    iot_button_register_cb(btn2, BUTTON_PRESS_DOWN, btn2_press_down_cb, motor_handle);
    iot_button_register_cb(btn2, BUTTON_SINGLE_CLICK, btn2_single_click_cb, motor_handle);
    iot_button_register_cb(btn2, BUTTON_DOUBLE_CLICK, btn2_double_click_cb, motor_handle);

    /* --- Button 3: Function/Reset (GPIO 7) --- */
    button_config_t btn3_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = 4000,
        .short_press_time = 300,
        .gpio_button_config = {
            .gpio_num = BUTTON_FUNC_PIN,
            .active_level = 0,
        },
    };

    button_handle_t btn3 = iot_button_create(&btn3_cfg);
    if (!btn3) {
        ESP_LOGE(TAG, "Button 3 create failed");
        return ESP_FAIL;
    }

    iot_button_register_cb(btn3, BUTTON_DOUBLE_CLICK, btn3_double_click_cb, motor_handle);
    iot_button_register_cb(btn3, BUTTON_PRESS_DOWN, btn3_press_down_cb, motor_handle);
    iot_button_register_cb(btn3, BUTTON_LONG_PRESS_HOLD, btn3_long_press_hold_cb, NULL);
    iot_button_register_cb(btn3, BUTTON_PRESS_UP, btn3_press_up_cb, motor_handle);

    ESP_LOGI(TAG, "Buttons initialized");
    return ESP_OK;
}

/**
 * @brief Close the Matter commissioning window if currently open.
 *
 * Acquires the CHIP stack lock before accessing the CommissioningWindowManager.
 */
void close_commissioning_window(void)
{
    chip::DeviceLayer::PlatformMgr().LockChipStack();
    chip::CommissioningWindowManager &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (commissionMgr.IsCommissioningWindowOpen()) {
        commissionMgr.CloseCommissioningWindow();
    }
    chip::DeviceLayer::PlatformMgr().UnlockChipStack();
}
