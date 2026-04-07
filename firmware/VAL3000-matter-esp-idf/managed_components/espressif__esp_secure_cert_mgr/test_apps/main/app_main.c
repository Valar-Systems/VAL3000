/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ESP Secure Cert Test Application
 *
 * This test application validates the esp_secure_cert_mgr component
 * using the Unity test framework. It supports testing both TLV and
 * legacy formats across different ESP32 targets.
 *
 * Test Groups:
 * - tlv_legacy: Tests for TLV and legacy format operations
 * - crypto: Tests for cryptographic operations
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif
#include "esp_heap_caps.h"

#include "esp_idf_version.h"
#include "soc/soc_caps.h"
#include "esp_secure_cert_tlv_private.h"
/**
 * These includes are needed for the crypto operations,
 * nvs flash operations and esp_secure_cert operations to
 * avoid false memory leak reports.
*/
#if SOC_SHA_SUPPORT_PARALLEL_ENG
#include "sha/sha_parallel_engine.h"
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
#include "sha/sha_core.h"
#else
#include "sha/sha_dma.h"
#endif
#endif
#include "bignum_impl.h"
#include "mbedtls/pk.h"
#if (MBEDTLS_MAJOR_VERSION < 4)
#include "mbedtls/aes.h"
#include "mbedtls/gcm.h"
#endif // MBEDTLS_MAJOR_VERSION < 4
#include "nvs_flash.h"
#include "esp_secure_cert_read.h"
#include "esp_secure_cert_config.h"

// Include Unity headers based on IDF version
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_fixture.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "unity_fixture_extras.h"
#endif

#define TAG "app_main"

#define UNITY_FREERTOS_PRIORITY 5
#define UNITY_FREERTOS_CPU 0
#define UNITY_FREERTOS_STACK_SIZE CONFIG_UNITY_FREERTOS_STACK_SIZE

static void run_all_tests(void)
{
#if CONFIG_TEST_ESP_SECURE_CERT_TLV_LEGACY
    RUN_TEST_GROUP(tlv_legacy);
#endif
#if CONFIG_TEST_ESP_SECURE_CERT_CRYPTO
    RUN_TEST_GROUP(crypto);
#endif
#if CONFIG_TEST_ESP_SECURE_CERT_BASICS
    RUN_TEST_GROUP(basics);
#endif
#if CONFIG_TEST_ESP_SECURE_CERT_SECURE_VERIFICATION
    RUN_TEST_GROUP(secure_verification);
#endif
#if CONFIG_TEST_ESP_SECURE_CERT_WRITE
    RUN_TEST_GROUP(write);
#endif
}

/**
 * @brief Initialize crypto hardware and NVS to prevent false memory leak reports
 *
 * This function performs dummy crypto operations to initialize all hardware locks
 * and initializes NVS partitions. The first crypto operations create hardware mutexes,
 * which are one-time allocations that persist for the application lifetime. Similarly,
 * NVS initialization creates internal structures that persist throughout the application.
 * By performing these initializations before tests start, these allocations won't be
 * counted as memory leaks during test execution.
 */
static void initialize_crypto(void)
{
    ESP_LOGI(TAG, "Initializing crypto hardware");

    // Initialize SHA hardware mutex by performing a dummy SHA operation
    unsigned char sha_output[32] = {0};
    uint8_t input[256] = {0};
    esp_sha(SHA2_256, (const unsigned char *)input, 256, sha_output);
    ESP_LOGI(TAG, "SHA hardware initialized");

#if (MBEDTLS_MAJOR_VERSION < 4)
    // Initialize AES/GCM hardware mutex by performing a dummy GCM operation
    mbedtls_gcm_context gcm_ctx;
    mbedtls_gcm_init(&gcm_ctx);

    // Dummy key for initialization (16 bytes)
    unsigned char dummy_key[16] = {0};
    int ret = mbedtls_gcm_setkey(&gcm_ctx, MBEDTLS_CIPHER_ID_AES, dummy_key, 128);
    if (ret == 0) {
        ESP_LOGI(TAG, "AES/GCM hardware initialized");
    }
    mbedtls_gcm_free(&gcm_ctx);

    esp_mpi_enable_hardware_hw_op();
    esp_mpi_disable_hardware_hw_op();
#else
    unsigned char dummy_key[16] = {0};
    unsigned char dummy_input[16] = {0};
    unsigned char dummy_output[16] = {0};
    esp_secure_cert_crypto_gcm_decrypt(dummy_input, sizeof(dummy_input), dummy_output,
                                      dummy_key, sizeof(dummy_key),
                                      dummy_input, NULL, NULL, 0);
#endif // MBEDTLS_MAJOR_VERSION < 4

    /**
     *  These operations are needed to initialize the related structures
     *  for the crypto operations, nvs flash operations and esp_secure_cert operations
     *  to avoid false memory leak reports.
     */

    uint32_t len = 0;
    char *addr = NULL;
    esp_err_t esp_ret = esp_secure_cert_get_device_cert(&addr, &len);
    if (esp_ret == ESP_OK) {
        esp_ret = esp_secure_cert_free_device_cert(addr);
        if (esp_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to free device cert");
            return;
        }
        ESP_LOGI(TAG, "Device cert initialized");
    }

}

static void test_task(void *pvParameters)
{
    vTaskDelay(2); /* Delay a bit to let the main task be deleted */

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    // IDF 5.0+: Use UNITY_MAIN_FUNC macro
    UNITY_MAIN_FUNC(run_all_tests);
#else
    // IDF 4.x: Call Unity fixture runner directly
    const char *argv[] = {"test", "-v"};
    int argc = sizeof(argv) / sizeof(argv[0]);
    UnityMain(argc, argv, run_all_tests);
#endif

    vTaskDelete(NULL);
}

void app_main(void)
{
    initialize_crypto();
#ifdef CONFIG_HEAP_TRACING_STANDALONE
    // Initialize standalone heap tracing (works in QEMU)
    #define NUM_RECORDS 100
    static heap_trace_record_t trace_records[NUM_RECORDS];
    ESP_ERROR_CHECK(heap_trace_init_standalone(trace_records, NUM_RECORDS));
    ESP_LOGI(TAG, "Standalone heap tracing initialized with %d records", NUM_RECORDS);
#endif

    xTaskCreatePinnedToCore(test_task, "testTask", UNITY_FREERTOS_STACK_SIZE, NULL, UNITY_FREERTOS_PRIORITY, NULL, UNITY_FREERTOS_CPU);
}
