/**
 * @file app_main.cpp
 * @brief Entry point for the VAL3000 Curtain/Blind Controller (ESP-IDF + ESP-Matter).
 *
 * Initializes NVS, the curtain motor driver, the Matter node with a Window
 * Covering (Drapery) endpoint, physical buttons, and the attribute reporting
 * queue. Handles Matter lifecycle events (commissioning, fabric changes).
 *
 * Converted from the Arduino VAL3000-matter-arduino project.
 * Targets ESP32-C3 with TMC2209 stepper motor driver.
 *
 * @par Device Type
 * Matter Window Covering - Drapery
 *
 * @par Features
 * - Lift position control (0-100%)
 * - StallGuard obstacle detection
 * - 3-button physical control (close, open, function)
 * - Calibration via button double-click (set distance)
 * - Direction reversal
 * - NVS persistence of motor state
 * - Matter commissioning via BLE
 */

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
#include <esp_matter_attribute.h>

#include <app_priv.h>

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#include "curtain_motor.h"

static const char *TAG = "app_main";

/** @brief Matter endpoint ID assigned to the Window Covering device. */
uint16_t curtain_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

/** @brief Timeout (in seconds) for re-opening the commissioning window after last fabric removal. */
constexpr auto k_timeout_seconds = 300;

/**
 * @brief Internal representation of an esp_matter command entry.
 *
 * Used to override the StopMotion command callback by casting the opaque
 * command handle returned by esp_matter::command::get().
 */
typedef struct _command {
    uint32_t command_id;
    uint16_t flags;
    command::callback_t callback;
    struct _command *next;
} _command_t;

/* Forward declarations */
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg);
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data);
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data);
static esp_err_t stop_movement(const ConcreteCommandPath &command_path, TLVReader &tlv_data, void *opaque_ptr);

/**
 * @brief Application entry point.
 *
 * Initialization sequence:
 * 1. NVS flash
 * 2. Curtain motor driver (UART, GPIO, TMC2209, watcher task)
 * 3. Matter node with Window Covering endpoint (Drapery type)
 * 4. Lift and position-aware lift features with overridden attributes
 * 5. Installed lift limits from NVS
 * 6. StopMotion command override
 * 7. Physical buttons
 * 8. Matter attribute reporting queue
 * 9. Matter stack start
 */
extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  VAL3000 Curtain Motor Control System");
    ESP_LOGI(TAG, "  Version: %s", PROJECT_VER);
    ESP_LOGI(TAG, "========================================");

    /* Initialize driver - no deferred loading, everything starts immediately */
    curtain_motor *motor_handle = new curtain_motor(on_motor_stop);
    if (motor_handle->initialize() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motor driver");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);

    /* Create Window Covering endpoint with Drapery type
     * Arduino used: MatterWindowCovering::DRAPERY
     * ESP-Matter equivalent: EndProductType::kCentralCurtain */
    window_covering_device::config_t curtain_config(
        static_cast<uint8_t>(chip::app::Clusters::WindowCovering::EndProductType::kCentralCurtain));

    /* Configure status flags */
    chip::BitMask<chip::app::Clusters::WindowCovering::ConfigStatus> config_status;
    config_status.SetField(chip::app::Clusters::WindowCovering::ConfigStatus::kOperational, true);
    config_status.SetField(chip::app::Clusters::WindowCovering::ConfigStatus::kLiftPositionAware, true);
    curtain_config.window_covering.config_status = config_status.Raw();
    curtain_config.window_covering.type = static_cast<uint8_t>(chip::app::Clusters::WindowCovering::Type::kDrapery);

    endpoint_t *endpoint = window_covering_device::create(node, &curtain_config, ENDPOINT_FLAG_NONE, motor_handle);
    cluster_t *cluster = cluster::get(endpoint, WindowCovering::Id);

    /* Add lift and position-aware lift features */
    esp_matter::cluster::window_covering::feature::lift::config_t lift_config;
    esp_matter::cluster::window_covering::feature::position_aware_lift::config_t position_aware_lift_config;

    if (!node || !endpoint) {
        ESP_LOGE(TAG, "Matter node creation failed");
    }

    curtain_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Curtain endpoint created with endpoint_id %d", curtain_endpoint_id);

    /* Set initial position values from driver state */
    uint16_t initial_percent100ths = motor_handle->get_percent100ths();
    position_aware_lift_config.current_position_lift_percentage = (uint8_t)(initial_percent100ths / 100);
    position_aware_lift_config.target_position_lift_percent_100ths = initial_percent100ths;
    position_aware_lift_config.current_position_lift_percent_100ths = initial_percent100ths;

    esp_matter::cluster::window_covering::feature::lift::add(cluster, &lift_config);

    /* Create overridden attributes (not using NONVOLATILE since we persist in our own NVS) */
    attribute::create(cluster, WindowCovering::Attributes::CurrentPositionLiftPercentage::Id,
        ATTRIBUTE_FLAG_NULLABLE | ATTRIBUTE_FLAG_OVERRIDE,
        esp_matter_nullable_uint8(position_aware_lift_config.current_position_lift_percentage));
    attribute::create(cluster, WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id,
        ATTRIBUTE_FLAG_NULLABLE | ATTRIBUTE_FLAG_OVERRIDE,
        esp_matter_nullable_uint16(position_aware_lift_config.current_position_lift_percent_100ths));
    attribute::create(cluster, WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id,
        ATTRIBUTE_FLAG_NULLABLE | ATTRIBUTE_FLAG_OVERRIDE,
        esp_matter_nullable_uint16(position_aware_lift_config.target_position_lift_percent_100ths));

    esp_matter::cluster::window_covering::feature::position_aware_lift::add(cluster, &position_aware_lift_config);

    /* Set installed lift limits (matches Arduino: setInstalledOpenLimitLift / setInstalledClosedLimitLift)
     * InstalledOpenLimitLift = 0 (MIN_LIFT)
     * InstalledClosedLimitLift = max lift in cm (loaded from NVS) */
    esp_matter_attr_val_t limit_val;
    limit_val = esp_matter_uint16(0);
    attribute::set_val(attribute::get(cluster, WindowCovering::Attributes::InstalledOpenLimitLift::Id), &limit_val);
    limit_val = esp_matter_uint16(motor_handle->get_max_lift_cm());
    attribute::set_val(attribute::get(cluster, WindowCovering::Attributes::InstalledClosedLimitLift::Id), &limit_val);
    ESP_LOGI(TAG, "Installed lift limits: Open=0, Closed=%d cm", motor_handle->get_max_lift_cm());

    /* Create SafetyStatus attribute (not created by default in esp_matter) */
    attribute::create(cluster, WindowCovering::Attributes::SafetyStatus::Id,
        ATTRIBUTE_FLAG_NONE, esp_matter_bitmap16(0));

    /* Override StopMotion command to use our stop handler */
    ((_command_t *)esp_matter::command::get(cluster, WindowCovering::Commands::StopMotion::Id, COMMAND_FLAG_ACCEPTED))->callback = stop_movement;

    /* Initialize buttons */
    init_buttons(motor_handle);

    /* Initialize Matter reporting queue */
    init_matter_reporting();

    /* Start Matter */
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter start failed: %d", err);
    }

    ESP_LOGI(TAG, "VAL3000 Curtain Controller started successfully");

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::init();
#endif
}

/**
 * @brief Matter device event callback.
 *
 * Handles commissioning lifecycle events, fabric changes, and IP address
 * updates. Re-opens the commissioning window when the last fabric is removed.
 *
 * @param[in] event Pointer to the CHIP device event.
 * @param[in] arg   User argument (unused).
 */
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
    {
        ESP_LOGI(TAG, "Fabric removed successfully");
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            chip::CommissioningWindowManager &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
            if (!commissionMgr.IsCommissioningWindowOpen()) {
                CHIP_ERROR chip_err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (chip_err != CHIP_NO_ERROR) {
                    ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, chip_err.Format());
                }
            }
        }
        break;
    }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    default:
        break;
    }
}

/**
 * @brief Matter identification effect callback.
 *
 * Called when the Matter controller triggers an Identify cluster effect.
 * Currently only logs the event.
 *
 * @param[in] type        Identification callback type.
 * @param[in] endpoint_id Endpoint that received the identify command.
 * @param[in] effect_id   Identify effect ID.
 * @param[in] effect_variant Effect variant.
 * @param[in] priv_data   Per-endpoint private data (unused).
 * @return ESP_OK always.
 */
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

/**
 * @brief Matter attribute update/read dispatcher.
 *
 * Routes PRE_UPDATE and READ callbacks to the appropriate app_driver functions
 * which interact with the curtain_motor driver.
 *
 * @param[in]     type         PRE_UPDATE or READ.
 * @param[in]     endpoint_id  Endpoint owning the attribute.
 * @param[in]     cluster_id   Cluster the attribute belongs to.
 * @param[in]     attribute_id Attribute being accessed.
 * @param[in,out] val          Attribute value (input for PRE_UPDATE, output for READ).
 * @param[in]     priv_data    Per-endpoint private data (curtain_motor*).
 * @return ESP_OK on success, or an error code.
 */
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    switch (type) {
    case PRE_UPDATE:
    {
        ESP_LOGI(TAG, "PRE_UPDATE: cluster=0x%04lx, attr=0x%04lx", (unsigned long)cluster_id, (unsigned long)attribute_id);
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val, priv_data);
        break;
    }
    case READ:
    {
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_read(driver_handle, endpoint_id, cluster_id, attribute_id, val, priv_data);
        break;
    }
    default:
        break;
    }

    return err;
}

/**
 * @brief Custom StopMotion command handler for the Window Covering cluster.
 *
 * Overrides the default esp_matter StopMotion handler. Performs a smooth stop
 * on the motor, waits 250 ms for deceleration, then reports the final position.
 *
 * @param[in] command_path The concrete command path (endpoint, cluster, command IDs).
 * @param[in] tlv_data     TLV-encoded command fields (none for StopMotion).
 * @param[in] opaque_ptr   Pointer to the CommandHandler for sending the response.
 * @return ESP_OK on success, ESP_FAIL if the endpoint is unknown or has no motor.
 */
static esp_err_t stop_movement(const ConcreteCommandPath &command_path, TLVReader &tlv_data, void *opaque_ptr)
{
    chip::app::CommandHandler *commandObj = (chip::app::CommandHandler *)opaque_ptr;
    chip::EndpointId endpoint = command_path.mEndpointId;

    ESP_LOGI(TAG, "StopMotion command received for endpoint %d", endpoint);

    if (endpoint == curtain_endpoint_id) {
        curtain_motor *motor = (curtain_motor *)get_priv_data(endpoint);
        if (motor == NULL) {
            commandObj->AddStatus(command_path, chip::Protocols::InteractionModel::Status::Failure);
            return ESP_FAIL;
        }

        commandObj->AddStatus(command_path, chip::Protocols::InteractionModel::Status::Success);
        motor->smooth_stop();
        vTaskDelay(pdMS_TO_TICKS(250));

        /* Report current position as the target position (stopped at current) */
        esp_matter_attr_val_t value;
        value.type = ESP_MATTER_VAL_TYPE_NULLABLE_UINT16;
        value.val.u16 = motor->get_percent100ths();
        queue_attribute_report(curtain_endpoint_id, WindowCovering::Id,
                               WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, &value);

        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Stop command sent to unknown endpoint");
        commandObj->AddStatus(command_path, chip::Protocols::InteractionModel::Status::NotFound);
        return ESP_FAIL;
    }
}
