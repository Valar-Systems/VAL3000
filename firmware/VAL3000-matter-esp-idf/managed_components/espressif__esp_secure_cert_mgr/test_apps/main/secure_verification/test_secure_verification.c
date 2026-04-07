/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "unity.h"
#include "unity_fixture.h"
#include "esp_log.h"
#include "esp_secure_cert_signature_verify.h"
#include "esp_err.h"
#include "esp_efuse.h"
#include "esp_efuse_table.h"
#include "esp_secure_boot.h"
#include "esp_system.h"

#define TAG "test_secure_verification"

/* Test group for secure verification */
TEST_GROUP(secure_verification);

TEST_SETUP(secure_verification)
{
    /* Setup code runs before each test in this group */
}

TEST_TEAR_DOWN(secure_verification)
{
    /* Teardown code runs after each test in this group */
}

// Test case constants
#define TEST_CASE_1_EFUSE_BLOCK 0  // First eFuse block for secure boot key
#define TEST_CASE_2_EFUSE_BLOCK 1  // Second eFuse block for fake key
#define TEST_CASE_3_EFUSE_BLOCK 2  // Third eFuse block for new secure boot key

// Key file paths (embedded as constants for QEMU testing)
extern const char secure_boot_key_pem[];
extern const char fake_key_pem[];

#ifndef CONFIG_TEST_APP_SECURE_VERIFICATION_CORRUPT_PARTITION
// 32-byte array for the secure boot key digest: 640cb9b4793b6bce72fc92e3f3c977cd69acd2cd287ae0e533cf48f38e34093f
static uint8_t secure_boot_digest[32] = {
    0x64, 0x0c, 0xb9, 0xb4, 0x79, 0x3b, 0x6b, 0xce,
    0x72, 0xfc, 0x92, 0xe3, 0xf3, 0xc9, 0x77, 0xcd,
    0x69, 0xac, 0xd2, 0xcd, 0x28, 0x7a, 0xe0, 0xe5,
    0x33, 0xcf, 0x48, 0xf3, 0x8e, 0x34, 0x09, 0x3f
};

static uint8_t fake_key_digest[32] = {
    0x26, 0xef, 0xa2, 0xbe, 0x14, 0xac, 0xc9, 0x8c,
    0xa3, 0x19, 0x92, 0xbf, 0x9d, 0x7e, 0x45, 0x89,
    0x74, 0x16, 0xfb, 0x77, 0x47, 0x68, 0x44, 0xbf,
    0x55, 0x63, 0x52, 0xf6, 0xc4, 0x9d, 0x32, 0x50
};
#endif

#ifndef CONFIG_TEST_APP_SECURE_VERIFICATION_CORRUPT_PARTITION
// Test Case 1: Burn secure boot key and verify (should pass)
TEST(secure_verification, test_case_1_secure_boot_key_burn_and_verify)
{
    ESP_LOGI(TAG, "TEST CASE 1: Burn secure boot key to eFuse and verify");
    ESP_LOGI(TAG, "Expected: Secure verification should PASS");

    // Secure boot key is already burned to eFuse block 0
    ESP_LOGI(TAG, "Performing secure verification...");
    esp_err_t ret = esp_secure_cert_verify_partition_signature(NULL);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TEST CASE 1: esp_secure_cert partition signature verification PASSED");
    } else {
        ESP_LOGE(TAG, "TEST CASE 1: esp_secure_cert partition signature verification FAILED");
    }

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "TEST CASE 1: Completed successfully");
}

// Test Case 2: Revoke secure boot key and verify (should fail)
TEST(secure_verification, test_case_2_revoke_secure_boot_key_and_verify)
{
    ESP_LOGI(TAG, "TEST CASE 2: Revoke secure boot key from eFuse and verify");
    ESP_LOGI(TAG, "Expected: Secure verification should FAIL");

    // Revoke the secure boot key from first eFuse block
    esp_err_t ret = esp_efuse_set_digest_revoke(0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "Failed to revoke secure boot key");

    ESP_LOGI(TAG, "Performing secure verification...");
    ret = esp_secure_cert_verify_partition_signature(NULL);

    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "TEST CASE 2: Secure verification succeeded unexpectedly");
    } else {
        ESP_LOGI(TAG, "TEST CASE 2: Secure verification FAILED as expected");
    }

    TEST_ASSERT_NOT_EQUAL_MESSAGE(ESP_OK, ret, "Verification should fail with revoked key");
    ESP_LOGI(TAG, "TEST CASE 2: Completed successfully");
}

// Test Case 3: Burn fake key and verify (should fail)
TEST(secure_verification, test_case_3_burn_fake_key_and_verify)
{
    ESP_LOGI(TAG, "TEST CASE 3: Burn fake key to eFuse and verify");
    ESP_LOGI(TAG, "Expected: Secure verification should FAIL");

    // Burn the fake key to second eFuse block
#if CONFIG_SECURE_FLASH_ENC_ENABLED
    esp_err_t ret = esp_efuse_write_key(EFUSE_BLK_KEY2,
        ESP_EFUSE_KEY_PURPOSE_SECURE_BOOT_DIGEST1,
        fake_key_digest, 32);
#else
    esp_err_t ret = esp_efuse_write_key(EFUSE_BLK_KEY1,
        ESP_EFUSE_KEY_PURPOSE_SECURE_BOOT_DIGEST1,
        fake_key_digest, 32);
#endif
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "Failed to burn fake key");

    // Perform secure verification - should fail
    ESP_LOGI(TAG, "Performing secure verification...");
    ret = esp_secure_cert_verify_partition_signature(NULL);

    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "TEST CASE 3: Secure verification succeeded unexpectedly");
    } else {
        ESP_LOGI(TAG, "TEST CASE 3: Secure verification FAILED as expected");
    }

    TEST_ASSERT_NOT_EQUAL_MESSAGE(ESP_OK, ret, "Verification should fail with fake key");
    ESP_LOGI(TAG, "TEST CASE 3: Completed successfully");
}

// Test Case 4: Revoke fake key and burn secure boot key and verify (should PASS)
TEST(secure_verification, test_case_4_revoke_fake_key_and_burn_secure_boot_key_and_verify)
{
    ESP_LOGI(TAG, "TEST CASE 4: Revoke fake key from eFuse and burn secure boot key and verify");
    ESP_LOGI(TAG, "Expected: Secure verification should PASS");

    // Revoke the fake key from second eFuse block
    ESP_LOGI(TAG, "Revoking fake key from eFuse block 1");
    esp_err_t ret = esp_efuse_set_digest_revoke(1);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "Failed to revoke fake key");

    // Burn the secure boot key to third eFuse block (slot 2) since slots 0 and 1 are revoked
#if CONFIG_SECURE_FLASH_ENC_ENABLED
    ESP_LOGI(TAG, "Burning secure boot key to eFuse block 3");
    ret = esp_efuse_write_key(EFUSE_BLK_KEY3,
        ESP_EFUSE_KEY_PURPOSE_SECURE_BOOT_DIGEST2,
        secure_boot_digest, 32);
#else
    ESP_LOGI(TAG, "Burning secure boot key to eFuse block 2");
    ret = esp_efuse_write_key(EFUSE_BLK_KEY2,
        ESP_EFUSE_KEY_PURPOSE_SECURE_BOOT_DIGEST2,
        secure_boot_digest, 32);
#endif
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "Failed to burn secure boot key");

    // Perform secure verification - should PASS
    ESP_LOGI(TAG, "Performing secure verification...");
    ret = esp_secure_cert_verify_partition_signature(NULL);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "TEST CASE 4: esp_secure_cert partition signature verification PASSED");
    } else {
        ESP_LOGE(TAG, "TEST CASE 4: esp_secure_cert partition signature verification FAILED");
    }

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "TEST CASE 4: Completed successfully");
}
#else
// Test Case 5: Corrupt secure cert partition and verify (should fail)
TEST(secure_verification, test_case_5_corrupt_secure_cert_partition_and_verify)
{
    ESP_LOGI(TAG, "TEST CASE 5: Corrupt secure cert partition and verify");
    ESP_LOGI(TAG, "Expected: Secure verification should FAIL");

    // Perform secure verification - should FAIL
    ESP_LOGI(TAG, "Performing secure verification...");
    esp_err_t ret = esp_secure_cert_verify_partition_signature(NULL);

    if (ret == ESP_OK) {
        ESP_LOGE(TAG, "TEST CASE 5: Secure verification succeeded unexpectedly");
    } else {
        ESP_LOGI(TAG, "TEST CASE 5: Secure verification FAILED as expected");
    }

    TEST_ASSERT_NOT_EQUAL_MESSAGE(ESP_OK, ret, "Verification should fail with corrupt partition");
    ESP_LOGI(TAG, "TEST CASE 5: Completed successfully");
}
#endif

/* Register test group runner */
TEST_GROUP_RUNNER(secure_verification)
{
    ESP_LOGI(TAG, "================================================================");
    ESP_LOGI(TAG, "Starting Comprehensive Secure Verification Test Suite");
    ESP_LOGI(TAG, "This test suite will run test cases to verify secure boot key management");
    ESP_LOGI(TAG, "================================================================");

#ifndef CONFIG_TEST_APP_SECURE_VERIFICATION_CORRUPT_PARTITION
    // Run all test cases in sequence - these MUST run in order
    // as each test depends on the state set by previous tests
    RUN_TEST_CASE(secure_verification, test_case_1_secure_boot_key_burn_and_verify);
    RUN_TEST_CASE(secure_verification, test_case_2_revoke_secure_boot_key_and_verify);
    RUN_TEST_CASE(secure_verification, test_case_3_burn_fake_key_and_verify);
    RUN_TEST_CASE(secure_verification, test_case_4_revoke_fake_key_and_burn_secure_boot_key_and_verify);
#else
    // Run test case 5 for corrupt partition scenario
    RUN_TEST_CASE(secure_verification, test_case_5_corrupt_secure_cert_partition_and_verify);
#endif

    ESP_LOGI(TAG, "================================================================");
    ESP_LOGI(TAG, "Secure Verification Test Suite Completed");
    ESP_LOGI(TAG, "================================================================");
}
