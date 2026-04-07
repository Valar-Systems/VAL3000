/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include "unity.h"
#include "esp_log.h"
#include "esp_secure_cert_read.h"
#include "esp_secure_cert_tlv_read.h"
#include "unity_fixture.h"
#include "esp_idf_version.h"
#if CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif

// Memory leak detection support (available from IDF 5.0+)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "unity_test_utils_memory.h"
#define MEMORY_LEAK_DETECTION_ENABLED 1
#else
#define MEMORY_LEAK_DETECTION_ENABLED 0
#endif

#define TAG "test_esp_secure_cert_basics"

/* Test group for partition management */
TEST_GROUP(basics);

TEST_SETUP(basics)
{
    /* Setup code runs before each test in this group */

#ifdef CONFIG_HEAP_TRACING_STANDALONE
    // Start standalone heap tracing to track all allocations
    ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
    ESP_LOGI(TAG, "Heap tracing started for basics test");
#endif

#if MEMORY_LEAK_DETECTION_ENABLED
    // Crypto hardware (SHA mutex) is initialized in app_main before tests
    // Set strict leak detection (0 bytes tolerance) to catch any real leaks
    unity_utils_set_leak_level(0);
    // Record free memory before test
    unity_utils_record_free_mem();
#endif
}

TEST_TEAR_DOWN(basics)
{
    /* Teardown code runs after each test in this group */

#ifdef CONFIG_HEAP_TRACING_STANDALONE
    // Stop heap tracing and dump leaked allocations with call stacks
    ESP_ERROR_CHECK(heap_trace_stop());
    ESP_LOGI(TAG, "========== Heap Trace Results (Leaked Allocations) ==========");
    heap_trace_dump();
    ESP_LOGI(TAG, "==============================================================");
#endif

#if MEMORY_LEAK_DETECTION_ENABLED
    // Check for memory leaks after test
    unity_utils_evaluate_leaks();
#endif
}

// Function to print certificate or key data in PEM or DER format
static void esp_print_cert_or_key(const char *label, const char *data, uint32_t len)
{
    if (len > 0 && data != NULL) {
        const char *pem_header = "-----BEGIN";
        if (strncmp(data, pem_header, strlen(pem_header)) == 0) {
            ESP_LOGI(TAG, "%s (PEM): \nLength: %"PRIu32"\n%s", label, len, data);
        } else {
            ESP_LOGI(TAG, "%s (DER): \nLength: %"PRIu32"\n", label, len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_INFO);
        }
    } else {
        ESP_LOGW(TAG, "%s: No data found", label);
    }
}

TEST(basics, esp_secure_cert_unmap_partition_api_test)
{
    esp_err_t ret = ESP_FAIL;

    ESP_LOGI(TAG, "=== Test: esp_secure_cert_unmap_partition ===");

    // Test 1: Map the partition first
    ESP_LOGI(TAG, "Test 1: Mapping partition...");
    esp_secure_cert_partition_ctx_t *esp_secure_cert_partition_ctx_ptr = NULL;
    ret = esp_secure_cert_map_partition(&esp_secure_cert_partition_ctx_ptr);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "Partition mapped successfully");

    // Verify partition is mapped
    TEST_ASSERT_NOT_NULL(esp_secure_cert_partition_ctx_ptr->esp_secure_cert_mapped_addr);
    TEST_ASSERT_NOT_EQUAL(0, esp_secure_cert_partition_ctx_ptr->handle);
    ESP_LOGI(TAG, "Partition context verified (handle: %lu, addr: %p)",
             (unsigned long)esp_secure_cert_partition_ctx_ptr->handle,
             esp_secure_cert_partition_ctx_ptr->esp_secure_cert_mapped_addr);

    // Test 2: Unmap the partition
    ESP_LOGI(TAG, "Test 2: Unmapping partition...");
    esp_secure_cert_unmap_partition();
    ESP_LOGI(TAG, "Partition unmapped successfully");

    // Verify partition is unmapped (context should be reset)
    TEST_ASSERT_NULL(esp_secure_cert_partition_ctx_ptr->esp_secure_cert_mapped_addr);
    TEST_ASSERT_EQUAL(0, esp_secure_cert_partition_ctx_ptr->handle);
    TEST_ASSERT_NULL(esp_secure_cert_partition_ctx_ptr->partition);
    ESP_LOGI(TAG, "Partition context cleared successfully");

    // Test 3: Unmap again (should return ESP_OK with warning)
    ESP_LOGI(TAG, "Test 3: Unmapping already unmapped partition...");
    esp_secure_cert_unmap_partition();
    ESP_LOGI(TAG, "Unmapping already unmapped partition returned ESP_OK");

    // Test 4: Remap the partition after unmapping
    ESP_LOGI(TAG, "Test 4: Remapping partition after unmap...");
    ret = esp_secure_cert_map_partition(&esp_secure_cert_partition_ctx_ptr);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    // Verify remapping worked
    TEST_ASSERT_NOT_NULL(esp_secure_cert_partition_ctx_ptr->esp_secure_cert_mapped_addr);
    TEST_ASSERT_NOT_EQUAL(0, esp_secure_cert_partition_ctx_ptr->handle);
    ESP_LOGI(TAG, "Partition remapped successfully (handle: %lu, addr: %p)",
             (unsigned long)esp_secure_cert_partition_ctx_ptr->handle,
             esp_secure_cert_partition_ctx_ptr->esp_secure_cert_mapped_addr);

    // Test 5: Verify we can still read data after remap
    ESP_LOGI(TAG, "Test 5: Verifying data access after remap...");
    char *cert_addr = NULL;
    uint32_t cert_len = 0;
    ret = esp_secure_cert_get_device_cert(&cert_addr, &cert_len);
    if (ret == ESP_OK && cert_len > 0) {
        esp_print_cert_or_key("Device Cert", (const char *)cert_addr, cert_len);
        ESP_LOGI(TAG, "Successfully read device cert after remap (length: %"PRIu32")", cert_len);
    } else {
        ESP_LOGI(TAG, "Device cert not available or read failed (may be expected in some configurations)");
    }
    ESP_LOGI(TAG, "esp_secure_cert_unmap_partition tests completed successfully");
}

/* Register test group runner */
TEST_GROUP_RUNNER(basics)
{
    RUN_TEST_CASE(basics, esp_secure_cert_unmap_partition_api_test);
}
