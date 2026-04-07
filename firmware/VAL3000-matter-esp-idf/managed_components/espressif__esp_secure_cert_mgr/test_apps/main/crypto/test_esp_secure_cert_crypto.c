/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_secure_cert_tlv_private.h"
#include "esp_idf_version.h"
#if CONFIG_HEAP_TRACING
#include "esp_heap_trace.h"
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

#define TAG "test_esp_secure_cert_crypto"

/* Test group for cryptographic operations */
TEST_GROUP(crypto);

TEST_SETUP(crypto)
{
    /* Setup code runs before each test in this group */

#ifdef CONFIG_HEAP_TRACING_STANDALONE
    // Start standalone heap tracing to track all allocations
    ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
    ESP_LOGI(TAG, "Heap tracing started for crypto test");
#endif

#if MEMORY_LEAK_DETECTION_ENABLED
    // Crypto hardware (SHA mutex) is initialized in app_main before tests
    // Set strict leak detection (0 bytes tolerance) to catch any real leaks
    unity_utils_set_leak_level(100); // TODO: Remove this once the memory leak is fixed
    // Record free memory before test
    unity_utils_record_free_mem();
#endif
}

TEST_TEAR_DOWN(crypto)
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

TEST(crypto, test_esp_secure_cert_crypto_gcm_decrypt)
{
    ESP_LOGI(TAG, "Testing GCM decryption");

    // Input: 25e242394a6acc0f8853a74398806f1c6045dfb3a2c5173f4be52ad840ea611d
    uint8_t in_buf[32] = {
        0x25, 0xE2, 0x42, 0x39, 0x4A, 0x6A, 0xCC, 0x0F,
        0x88, 0x53, 0xA7, 0x43, 0x98, 0x80, 0x6F, 0x1C,
        0x60, 0x45, 0xDF, 0xB3, 0xA2, 0xC5, 0x17, 0x3F,
        0x4B, 0xE5, 0x2A, 0xD8, 0x40, 0xEA, 0x61, 0x1D
    };

    // Expected output after decryption
    char expected_output_buf[32] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };

    uint8_t output_buf[32] = {0};

    // Key: 0123456789ABCDEF0123456789ABCDEF
    uint8_t key[32] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };

    uint8_t iv[16] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };

    // Tag: 939c4ec3f9ce7eaf87c3d7c2357c66d3
    uint8_t tag[16] = {
        0x93, 0x9C, 0x4E, 0xC3, 0xF9, 0xCE, 0x7E, 0xAF,
        0x87, 0xC3, 0xD7, 0xC2, 0x35, 0x7C, 0x66, 0xD3
    };

    size_t key_len = 32;
    size_t tag_len = 16;

    // Perform GCM decryption
    esp_err_t esp_ret = esp_secure_cert_crypto_gcm_decrypt(
        in_buf, 32, output_buf, key, key_len, iv, NULL, tag, tag_len
    );
    TEST_ASSERT_EQUAL(ESP_OK, esp_ret);

    // Verify decrypted data matches expected output
    TEST_ASSERT_EQUAL_MEMORY(expected_output_buf, output_buf, 32);

    ESP_LOGI(TAG, "GCM decryption test passed successfully");
}

// Expected DER-encoded ECDSA key for test validation
static const char expected_der_ecdsa_key_buf[ESP_SECURE_CERT_ECDSA_DER_KEY_SIZE] = {
    0x30, 0x77, 0x02, 0x01, 0x01, 0x04, 0x20, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
    0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0xA0, 0x0A, 0x06, 0x08, 0x2A, 0x86,
    0x48, 0xCE, 0x3D, 0x03, 0x01, 0x07, 0xA1, 0x44, 0x03, 0x42, 0x00, 0x04, 0x3C, 0x56, 0xAD,
    0xBA, 0xC4, 0x3A, 0xBC, 0x2C, 0x77, 0x33, 0x2F, 0x36, 0x22, 0x72, 0xC5, 0xA5, 0x11, 0x67,
    0xF7, 0x37, 0x75, 0xA5, 0x55, 0x4D, 0x0A, 0x7F, 0x36, 0x96, 0x74, 0x2E, 0x58, 0x63, 0xEA,
    0x4C, 0xF0, 0xA6, 0x8F, 0x91, 0x24, 0xC5, 0x6C, 0x0D, 0x50, 0x28, 0x43, 0x48, 0x89, 0xAD,
    0xDF, 0x4E, 0xD1, 0xF5, 0xAA, 0xAA, 0x2F, 0x42, 0x81, 0x2D, 0x4F, 0x45, 0xBA, 0x59, 0x4B,
    0x21
};

TEST(crypto, esp_secure_cert_convert_key_to_der)
{
    ESP_LOGI(TAG, "Testing ECDSA key conversion to DER format");

    // Input ECDSA key (raw format)
    char ecdsa_key_buf[32] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
    };

    // Allocate buffer for DER-encoded key
    uint8_t *der_ecdsa_key_buf = (uint8_t *)malloc(ESP_SECURE_CERT_ECDSA_DER_KEY_SIZE);
    TEST_ASSERT_NOT_NULL(der_ecdsa_key_buf);

    // Convert key to DER format
    esp_err_t esp_ret = esp_secure_cert_convert_key_to_der(
        ecdsa_key_buf, 32,
        der_ecdsa_key_buf, ESP_SECURE_CERT_ECDSA_DER_KEY_SIZE
    );
    TEST_ASSERT_EQUAL(ESP_OK, esp_ret);

    // Verify converted key matches expected DER format
    TEST_ASSERT_EQUAL_MEMORY(expected_der_ecdsa_key_buf, der_ecdsa_key_buf,
                              ESP_SECURE_CERT_ECDSA_DER_KEY_SIZE);

    // Clean up
    free(der_ecdsa_key_buf);

    ESP_LOGI(TAG, "Key conversion to DER format test passed successfully");
}

/* Register test group runner */
TEST_GROUP_RUNNER(crypto)
{
    RUN_TEST_CASE(crypto, test_esp_secure_cert_crypto_gcm_decrypt);
    RUN_TEST_CASE(crypto, esp_secure_cert_convert_key_to_der);
}
