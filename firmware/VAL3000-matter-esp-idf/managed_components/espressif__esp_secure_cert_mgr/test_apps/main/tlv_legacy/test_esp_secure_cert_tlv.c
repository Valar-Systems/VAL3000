/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_secure_cert_read.h"
#include "esp_secure_cert_tlv_read.h"
#include "esp_idf_version.h"

#if CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
#endif

#if (MBEDTLS_MAJOR_VERSION < 4)
#include "mbedtls/sha256.h"
#else
#include "psa/crypto.h"
#endif
#if SOC_SHA_SUPPORT_PARALLEL_ENG
#include "sha/sha_parallel_engine.h"
#else
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
#include "sha/sha_core.h"
#else
#include "sha/sha_dma.h"
#endif
#endif

// Memory leak detection support (available from IDF 5.0+)
#include "unity.h"
#include "unity_fixture.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "unity_test_utils_memory.h"
#define MEMORY_LEAK_DETECTION_ENABLED 1
#else
#define MEMORY_LEAK_DETECTION_ENABLED 0
#endif

#define TAG "test_esp_secure_cert_tlv"

/* Test group for TLV and Legacy format tests */
TEST_GROUP(tlv_legacy);

TEST_SETUP(tlv_legacy)
{
#ifdef CONFIG_HEAP_TRACING_STANDALONE
    // Start standalone heap tracing to track all allocations
    ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
    ESP_LOGI(TAG, "Heap tracing started for test");
#endif

#if MEMORY_LEAK_DETECTION_ENABLED
#ifdef CONFIG_ESP_SECURE_CERT_SUPPORT_LEGACY_FORMATS
    /**
     * Set leak detection level to 100 bytes to catch leaks due to mapping of partition
     * in case of cust_flash format and cust_flash_legacy format. Because in these formats,
     * the partition is mapped to the memory for evert esp_secure_cert operation.
     */
    unity_utils_set_leak_level(100);
#else
    /**
     * There should be no leaks in case of TLV format.
     */
    unity_utils_set_leak_level(0);
#endif
    // Record free memory before test
    unity_utils_record_free_mem();
#endif
}

TEST_TEAR_DOWN(tlv_legacy)
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

#if (MBEDTLS_MAJOR_VERSION < 4)
static esp_err_t get_sha256(const char *data, uint32_t len, unsigned char *sha256)
{
    esp_sha(SHA2_256, (const unsigned char *)data, len, sha256);
    return ESP_OK;
}
#else
static esp_err_t get_sha256(const char *data, uint32_t len, uint8_t *sha256)
{
    size_t hash_len = 0;
    psa_status_t status = psa_hash_compute(PSA_ALG_SHA_256, (const uint8_t *)data, len, sha256, 32, &hash_len);
    if (status != PSA_SUCCESS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
#endif

static void print_sha256_of_data(const char *label, const char *data, uint32_t len)
{
    if (len > 0 && data != NULL) {
        char sha256_str[65] = {0};
        uint8_t sha256[32] = {0};
        get_sha256(data, len, sha256);
        for (int i = 0; i < 32; i++) {
            snprintf(&sha256_str[i * 2], 3, "%02X", sha256[i]);
        }
        ESP_LOGI(TAG, "SHA256 of %s: %s", label, sha256_str);
    } else {
        ESP_LOGW(TAG, "%s: No data found for sha256 calculation", label);
    }
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
        // Optimize: handle PEM (null-terminated) and DER (binary) length for SHA256 calculation
        size_t actual_len = len;
        if (strncmp(data, pem_header, strlen(pem_header)) == 0) {
            actual_len -= 1; // Remove the last '\0' from the PEM data for sha256 calculation
        }
        print_sha256_of_data(label, data, actual_len);
    } else {
        ESP_LOGW(TAG, "%s: No data found", label);
    }
}

TEST(tlv_legacy, esp_secure_cert_read_certificates_and_keys)
{
    esp_err_t esp_ret = ESP_FAIL;

    ESP_LOGI(TAG, "Starting ESP Secure Cert TLV Test Application");

    // Read Device Certificate using TLV format
    // Use the standard API to get device and CA certs
    uint32_t len = 0;
    char *addr = NULL;

    // Read Device Certificate using standard API
    esp_ret = esp_secure_cert_get_device_cert(&addr, &len);
    if (esp_ret == ESP_OK) {
        esp_print_cert_or_key("Device Cert", (const char *)addr, len);
        TEST_ASSERT_NOT_NULL(addr);
        TEST_ASSERT_GREATER_THAN(0, len);
        esp_ret = esp_secure_cert_free_device_cert(addr);
        TEST_ASSERT_EQUAL(ESP_OK, esp_ret);
        addr = NULL;
        len = 0;
    } else {
        ESP_LOGE(TAG, "Failed to obtain flash address of device cert");
    }

    // Read CA Certificate using standard API
    esp_ret = esp_secure_cert_get_ca_cert(&addr, &len);
    if (esp_ret == ESP_OK) {
        esp_print_cert_or_key("CA Cert", (const char *)addr, len);
        TEST_ASSERT_NOT_NULL(addr);
        TEST_ASSERT_GREATER_THAN(0, len);
        esp_ret = esp_secure_cert_free_ca_cert(addr);
        TEST_ASSERT_EQUAL(ESP_OK, esp_ret);
        addr = NULL;
        len = 0;
    } else {
        ESP_LOGE(TAG, "Failed to obtain flash address of ca_cert");
    }

#ifndef CONFIG_ESP_SECURE_CERT_DS_PERIPHERAL
    // Read Private Key using standard format (for comparison)
    uint32_t priv_key_len = 0;
    char *priv_key_addr = NULL;
    esp_ret = esp_secure_cert_get_priv_key(&priv_key_addr, &priv_key_len);
    if (esp_ret == ESP_OK) {
        esp_print_cert_or_key("Private Key", (const char *)priv_key_addr, priv_key_len);
        TEST_ASSERT_NOT_NULL(priv_key_addr);
        TEST_ASSERT_GREATER_THAN(0, priv_key_len);
        esp_ret = esp_secure_cert_free_priv_key(priv_key_addr);
        TEST_ASSERT_EQUAL(ESP_OK, esp_ret);
        priv_key_addr = NULL;
        priv_key_len = 0;
    } else {
        ESP_LOGE(TAG, "Failed to read Private Key using standard format: %s", esp_err_to_name(esp_ret));
    }
#endif
#ifndef CONFIG_ESP_SECURE_CERT_SUPPORT_LEGACY_FORMATS
    // List all TLV entries
    ESP_LOGI(TAG, "Listing all TLV entries:");
    esp_secure_cert_list_tlv_entries();
#endif

    if (esp_ret == ESP_OK) {
        ESP_LOGI(TAG, "Test application completed successfully");
    } else {
        ESP_LOGE(TAG, "Test application failed");
    }
}

/* Register test group runner */
TEST_GROUP_RUNNER(tlv_legacy)
{
    RUN_TEST_CASE(tlv_legacy, esp_secure_cert_read_certificates_and_keys);
}
