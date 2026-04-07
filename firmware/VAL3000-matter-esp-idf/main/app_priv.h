/**
 * @file app_priv.h
 * @brief Application-level declarations for the VAL3000 curtain controller.
 *
 * Provides the interface between the Matter application layer and the
 * curtain_motor hardware driver. Declares callbacks for Matter attribute
 * updates/reads, button initialization, motor event handlers, and the
 * queue-based Matter attribute reporting system.
 */

#pragma once

#include "curtain_motor.h"

/** @brief Opaque handle for the application driver (cast to curtain_motor* internally). */
typedef void *app_driver_handle_t;

/* ========================================
 * FUNCTION DECLARATIONS
 * ======================================== */

/**
 * @brief Handle a Matter attribute PRE_UPDATE for the curtain endpoint.
 *
 * Called by the Matter stack before an attribute value is committed.
 * Routes TargetPositionLiftPercent100ths writes to the motor driver
 * and reports the resulting OperationalStatus.
 *
 * @param[in] driver_handle Opaque handle (unused, priv_data carries the motor pointer).
 * @param[in] endpoint_id   Matter endpoint that received the write.
 * @param[in] cluster_id    Cluster the attribute belongs to.
 * @param[in] attribute_id  Attribute being written.
 * @param[in] val           New attribute value proposed by the Matter stack.
 * @param[in] priv_data     Per-endpoint private data (curtain_motor*).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data);

/**
 * @brief Handle a Matter attribute READ for the curtain endpoint.
 *
 * Called by the Matter stack when an external controller reads an attribute.
 * Returns real-time values from the motor driver for position, target, and
 * operational status attributes.
 *
 * @param[in]  driver_handle Opaque handle (unused).
 * @param[in]  endpoint_id   Matter endpoint being read.
 * @param[in]  cluster_id    Cluster the attribute belongs to.
 * @param[in]  attribute_id  Attribute being read.
 * @param[out] val           Filled with the current attribute value.
 * @param[in]  priv_data     Per-endpoint private data (curtain_motor*).
 * @return ESP_OK on success, or an error code.
 */
esp_err_t app_driver_attribute_read(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                    uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data);

/**
 * @brief Create and configure the three physical buttons (Close, Open, Function).
 *
 * Registers press-down, single-click, double-click, and long-press callbacks
 * using the espressif/button (iot_button) component.
 *
 * @param[in] motor_handle Pointer to the curtain_motor instance passed as user data.
 * @return ESP_OK on success, ESP_FAIL if any button creation fails.
 */
esp_err_t init_buttons(curtain_motor *motor_handle);

/**
 * @brief Motor-stop callback invoked by curtain_motor when movement ends.
 *
 * Reports the final position (CurrentPositionLiftPercent100ths,
 * TargetPositionLiftPercent100ths, CurrentPositionLiftPercentage) and sets
 * OperationalStatus to Stall via the Matter reporting queue.
 *
 * @param[in] percent100ths Current lift position in hundredths of a percent (0-10000).
 */
void on_motor_stop(uint16_t percent100ths);

/**
 * @brief Close the Matter commissioning window if it is currently open.
 *
 * Acquires the CHIP stack lock before accessing the CommissioningWindowManager.
 */
void close_commissioning_window(void);

/**
 * @brief Initialize the FreeRTOS queue and task for deferred Matter attribute reporting.
 *
 * Creates a 20-item queue and a dedicated task that calls esp_matter::attribute::report()
 * for each queued event, ensuring reports are made from a safe (non-ISR) context.
 *
 * @return ESP_OK on success, ESP_FAIL if queue or task creation fails.
 */
esp_err_t init_matter_reporting(void);

/**
 * @brief Enqueue a Matter attribute report for asynchronous delivery.
 *
 * Thread-safe; may be called from button callbacks, motor callbacks, or any task.
 * Blocks until space is available in the queue.
 *
 * @param[in] endpoint_id  Matter endpoint owning the attribute.
 * @param[in] cluster_id   Cluster the attribute belongs to.
 * @param[in] attribute_id Attribute to report.
 * @param[in] val          Pointer to the value to report (copied into the queue).
 * @return ESP_OK on success, ESP_FAIL if the send fails.
 */
esp_err_t queue_attribute_report(uint16_t endpoint_id, uint32_t cluster_id,
                                  uint32_t attribute_id, esp_matter_attr_val_t *val);

/**
 * @brief Enqueue a Matter attribute update (write to store) for asynchronous delivery.
 *
 * Uses attribute::update() with the CHIP stack lock. Required for non-OVERRIDE
 * attributes such as OperationalStatus, where the value must be written to the
 * attribute store (not just reported from a callback).
 *
 * @param[in] endpoint_id  Matter endpoint owning the attribute.
 * @param[in] cluster_id   Cluster the attribute belongs to.
 * @param[in] attribute_id Attribute to update.
 * @param[in] val          Pointer to the new value (copied into the queue).
 * @return ESP_OK on success, ESP_FAIL if the send fails.
 */
esp_err_t queue_attribute_update(uint16_t endpoint_id, uint32_t cluster_id,
                                  uint32_t attribute_id, esp_matter_attr_val_t *val);
