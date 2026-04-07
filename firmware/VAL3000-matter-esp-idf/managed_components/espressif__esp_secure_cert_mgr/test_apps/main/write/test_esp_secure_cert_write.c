/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "unity.h"
#include "unity_fixture.h"
#include "esp_log.h"
#include "esp_secure_cert_read.h"
#include "esp_secure_cert_tlv_read.h"
#include "esp_secure_cert_write.h"
#include "esp_secure_cert_write_errors.h"
#include "esp_secure_cert_tlv_config.h"
#include "esp_secure_cert_tlv_private.h"
#include "esp_idf_version.h"
#include "soc/soc_caps.h"

#if SOC_HMAC_SUPPORTED
#include "esp_efuse.h"
#include "mbedtls/pk.h"  /* Must be included first to get MBEDTLS_MAJOR_VERSION */
#if (MBEDTLS_MAJOR_VERSION < 4)
#include "mbedtls/ecdsa.h"
#include "mbedtls/sha256.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/asn1.h"
#else
#include "psa/crypto.h"
#endif
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "unity_test_utils_memory.h"
#endif

#define TAG "test_write"

/* Simple test data */
static const char test_cert[] = "-----BEGIN CERTIFICATE-----\nTEST_CERT_DATA\n-----END CERTIFICATE-----\n";
static const uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

TEST_GROUP(write);

TEST_SETUP(write)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    unity_utils_set_leak_level(100);
    unity_utils_record_free_mem();
#endif
}

TEST_TEAR_DOWN(write)
{
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    unity_utils_evaluate_leaks();
#endif
}

/**
 * Test basic write and read back
 */
TEST(write, test_write_and_read)
{
    ESP_LOGI(TAG, "Testing write and read back");

    /* Erase partition */
    esp_err_t ret = esp_secure_cert_erase_partition();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Write a TLV entry */
    esp_secure_cert_tlv_info_t tlv_info = {
        .type = ESP_SECURE_CERT_USER_DATA_1,
        .subtype = ESP_SECURE_CERT_SUBTYPE_0,
        .data = (char *)test_data,
        .length = sizeof(test_data),
        .flags = 0
    };

    ret = esp_secure_cert_append_tlv(&tlv_info, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Remap partition to read from flash */
    esp_secure_cert_unmap_partition();

    /* Read back and verify */
    esp_secure_cert_tlv_config_t config = {
        .type = ESP_SECURE_CERT_USER_DATA_1,
        .subtype = ESP_SECURE_CERT_SUBTYPE_0
    };
    esp_secure_cert_tlv_info_t read_info = {0};

    ret = esp_secure_cert_get_tlv_info(&config, &read_info);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(sizeof(test_data), read_info.length);
    TEST_ASSERT_EQUAL_MEMORY(test_data, read_info.data, sizeof(test_data));

    esp_secure_cert_free_tlv_info(&read_info);
    ESP_LOGI(TAG, "Write and read test PASSED");
}

/**
 * Test batch write
 */
TEST(write, test_batch_write)
{
    ESP_LOGI(TAG, "Testing batch write");

    esp_err_t ret = esp_secure_cert_erase_partition();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Prepare batch */
    esp_secure_cert_tlv_info_t entries[2] = {
        {
            .type = ESP_SECURE_CERT_DEV_CERT_TLV,
            .subtype = ESP_SECURE_CERT_SUBTYPE_0,
            .data = (char *)test_cert,
            .length = strlen(test_cert) + 1,
            .flags = 0
        },
        {
            .type = ESP_SECURE_CERT_USER_DATA_1,
            .subtype = ESP_SECURE_CERT_SUBTYPE_0,
            .data = (char *)test_data,
            .length = sizeof(test_data),
            .flags = 0
        }
    };

    ret = esp_secure_cert_append_tlv_batch(entries, 2, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Verify both entries exist */
    esp_secure_cert_unmap_partition();

    esp_secure_cert_tlv_iterator_t iter = {0};
    int count = 0;
    while (esp_secure_cert_iterate_to_next_tlv(&iter) == ESP_OK) {
        count++;
    }
    TEST_ASSERT_EQUAL(2, count);

    ESP_LOGI(TAG, "Batch write test PASSED");
}

/**
 * Test duplicate rejection and error handling
 */
TEST(write, test_error_handling)
{
    ESP_LOGI(TAG, "Testing error handling");

    esp_err_t ret = esp_secure_cert_erase_partition();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Write first entry */
    esp_secure_cert_tlv_info_t tlv_info = {
        .type = ESP_SECURE_CERT_USER_DATA_1,
        .subtype = ESP_SECURE_CERT_SUBTYPE_0,
        .data = (char *)test_data,
        .length = sizeof(test_data),
        .flags = 0
    };

    ret = esp_secure_cert_append_tlv(&tlv_info, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Try duplicate - should fail */
    ret = esp_secure_cert_append_tlv(&tlv_info, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_SECURE_CERT_TLV_ALREADY_EXISTS, ret);

    /* Test NULL pointer */
    ret = esp_secure_cert_append_tlv(NULL, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Test invalid type */
    tlv_info.type = ESP_SECURE_CERT_TLV_MAX;
    ret = esp_secure_cert_append_tlv(&tlv_info, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_SECURE_CERT_TLV_INVALID_TYPE, ret);

    ESP_LOGI(TAG, "Error handling test PASSED");
}

/**
 * Test buffer mode write
 */
TEST(write, test_buffer_mode)
{
    ESP_LOGI(TAG, "Testing buffer mode");

    uint8_t buffer[256] = {0};
    size_t bytes_written = 0;

    esp_secure_cert_write_config_t config;
    esp_secure_cert_write_config_init(&config, ESP_SECURE_CERT_WRITE_MODE_BUFFER);
    config.buffer.buffer = buffer;
    config.buffer.buffer_size = sizeof(buffer);
    config.buffer.bytes_written = &bytes_written;

    esp_secure_cert_tlv_info_t tlv_info = {
        .type = ESP_SECURE_CERT_USER_DATA_1,
        .subtype = ESP_SECURE_CERT_SUBTYPE_0,
        .data = (char *)test_data,
        .length = sizeof(test_data),
        .flags = 0
    };

    esp_err_t ret = esp_secure_cert_append_tlv(&tlv_info, &config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_GREATER_THAN(0, bytes_written);

    /* Test buffer overflow */
    config.buffer.buffer_size = 4;  /* Too small */
    config.buffer.bytes_written = &bytes_written;
    bytes_written = 0;

    ret = esp_secure_cert_append_tlv(&tlv_info, &config);
    TEST_ASSERT_EQUAL(ESP_ERR_SECURE_CERT_BUFFER_OVERFLOW, ret);

    ESP_LOGI(TAG, "Buffer mode test PASSED");
}

/**
 * Test HMAC-ECDSA derivation write (without HMAC key - expects failure)
 * This test verifies the API behavior when no HMAC key is burned in eFuse
 */
#if SOC_HMAC_SUPPORTED
TEST(write, test_hmac_ecdsa_derivation_no_key)
{
    ESP_LOGI(TAG, "Testing HMAC-ECDSA derivation write without HMAC key");

    esp_err_t ret = esp_secure_cert_erase_partition();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Try to write HMAC-ECDSA derivation - should fail if no HMAC_UP key */
    uint8_t salt[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};

    /* Check if HMAC_UP key exists */
    esp_efuse_block_t efuse_block = EFUSE_BLK_MAX;
    bool has_hmac_key = esp_efuse_find_purpose(ESP_EFUSE_KEY_PURPOSE_HMAC_UP, &efuse_block);

    ret = esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(
        salt, sizeof(salt),
        ESP_SECURE_CERT_SUBTYPE_0,
        NULL
    );

    if (has_hmac_key) {
        /* If HMAC key exists, should succeed */
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "HMAC-ECDSA derivation write succeeded (HMAC key present)");

        /* Verify the salt was written */
        esp_secure_cert_unmap_partition();
        esp_secure_cert_tlv_config_t config = {
            .type = ESP_SECURE_CERT_HMAC_ECDSA_KEY_SALT,
            .subtype = ESP_SECURE_CERT_SUBTYPE_0
        };
        esp_secure_cert_tlv_info_t read_info = {0};
        ret = esp_secure_cert_get_tlv_info(&config, &read_info);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        TEST_ASSERT_EQUAL(sizeof(salt), read_info.length);
        TEST_ASSERT_EQUAL_MEMORY(salt, read_info.data, sizeof(salt));
        esp_secure_cert_free_tlv_info(&read_info);

        /* Verify the private key marker was written with derivation flag */
        config.type = ESP_SECURE_CERT_PRIV_KEY_TLV;
        ret = esp_secure_cert_get_tlv_info(&config, &read_info);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        /* The marker TLV should have the derivation flag set */
        TEST_ASSERT_TRUE(ESP_SECURE_CERT_HMAC_ECDSA_KEY_DERIVATION(read_info.flags));
        esp_secure_cert_free_tlv_info(&read_info);
    } else {
        /* If no HMAC key, should fail */
        TEST_ASSERT_EQUAL(ESP_ERR_SECURE_CERT_HMAC_KEY_NOT_FOUND, ret);
        ESP_LOGI(TAG, "HMAC-ECDSA derivation write correctly failed (no HMAC key)");
    }

    ESP_LOGI(TAG, "HMAC-ECDSA derivation test PASSED");
}

/**
 * Test HMAC-ECDSA derivation parameter validation
 */
TEST(write, test_hmac_ecdsa_derivation_invalid_params)
{
    ESP_LOGI(TAG, "Testing HMAC-ECDSA derivation invalid parameters");

    /* Test NULL salt */
    esp_err_t ret = esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(
        NULL, 16,
        ESP_SECURE_CERT_SUBTYPE_0,
        NULL
    );
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    /* Test zero salt length */
    uint8_t salt[16] = {0};
    ret = esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(
        salt, 0,
        ESP_SECURE_CERT_SUBTYPE_0,
        NULL
    );
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, ret);

    ESP_LOGI(TAG, "HMAC-ECDSA invalid params test PASSED");
}

/**
 * Test HMAC-ECDSA derivation buffer mode
 */
TEST(write, test_hmac_ecdsa_derivation_buffer_mode)
{
    ESP_LOGI(TAG, "Testing HMAC-ECDSA derivation buffer mode");

    /* Check if HMAC_UP key exists - buffer mode still needs to verify eFuse */
    esp_efuse_block_t efuse_block = EFUSE_BLK_MAX;
    bool has_hmac_key = esp_efuse_find_purpose(ESP_EFUSE_KEY_PURPOSE_HMAC_UP, &efuse_block);

    if (!has_hmac_key) {
        ESP_LOGI(TAG, "Skipping buffer mode test - no HMAC key in eFuse");
        TEST_IGNORE_MESSAGE("No HMAC_UP key in eFuse, skipping buffer mode test");
        return;
    }

    uint8_t buffer[512] = {0};
    size_t bytes_written = 0;

    esp_secure_cert_write_config_t config;
    esp_secure_cert_write_config_init(&config, ESP_SECURE_CERT_WRITE_MODE_BUFFER);
    config.buffer.buffer = buffer;
    config.buffer.buffer_size = sizeof(buffer);
    config.buffer.bytes_written = &bytes_written;

    uint8_t salt[16] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22,
                        0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00};

    esp_err_t ret = esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(
        salt, sizeof(salt),
        ESP_SECURE_CERT_SUBTYPE_0,
        &config
    );

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_GREATER_THAN(0, bytes_written);

    /* Buffer should contain at least 2 TLV entries (salt + marker) */
    /* Each TLV has: 12-byte header + data + padding + 4-byte footer */
    size_t min_expected = 2 * (sizeof(esp_secure_cert_tlv_header_t) +
                               sizeof(esp_secure_cert_tlv_footer_t));
    TEST_ASSERT_GREATER_OR_EQUAL(min_expected, bytes_written);

    ESP_LOGI(TAG, "HMAC-ECDSA buffer mode test PASSED (wrote %zu bytes)", bytes_written);
}

/**
 * Test HMAC-ECDSA key derivation with sign/verify
 *
 * This test verifies:
 * 1. esp_secure_cert_derive_hmac_ecdsa_key() generates a valid key pair
 * 2. The derived private key can perform ECDSA signing
 * 3. The returned public key can verify the signature
 *
 * Note: This test BURNS an HMAC key to eFuse (permanent operation).
 * It will be skipped if an HMAC_UP key already exists.
 */
TEST(write, test_hmac_ecdsa_derive_sign_verify)
{
    ESP_LOGI(TAG, "Testing HMAC-ECDSA derive with sign/verify");

    /* This test is NOT run by default because:
     * 1. eFuse burning is a PERMANENT operation (can only run once per device)
     * 2. PBKDF2 with 2048 iterations is too slow for QEMU (triggers watchdog)
     * 3. Requires functional hardware HMAC peripheral
     *
     * To enable this test, define CONFIG_TEST_HMAC_ECDSA_DERIVE_FULL in sdkconfig.
     * Only enable when testing on real hardware for actual provisioning. */
#ifndef CONFIG_TEST_HMAC_ECDSA_DERIVE_FULL
    TEST_IGNORE_MESSAGE("Test disabled - enable CONFIG_TEST_HMAC_ECDSA_DERIVE_FULL for real hardware");
    return;
#endif

    /* Check if HMAC_UP key already exists - skip if so (can't burn twice) */
    esp_efuse_block_t efuse_block = EFUSE_BLK_MAX;
    bool has_hmac_key = esp_efuse_find_purpose(ESP_EFUSE_KEY_PURPOSE_HMAC_UP, &efuse_block);

    if (has_hmac_key) {
        ESP_LOGI(TAG, "HMAC_UP key already exists in eFuse, skipping derive test");
        TEST_IGNORE_MESSAGE("HMAC_UP key already burned, skipping derive test");
        return;
    }

    /* Erase partition before test */
    esp_err_t ret = esp_secure_cert_erase_partition();
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    /* Generate HMAC-ECDSA key and get public key */
    uint8_t pub_key[65] = {0};
    size_t pub_key_len = sizeof(pub_key);

    ret = esp_secure_cert_derive_hmac_ecdsa_key(
        pub_key, &pub_key_len,
        ESP_SECURE_CERT_SUBTYPE_0,
        NULL  /* Use default flash write mode */
    );

    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(65, pub_key_len);
    TEST_ASSERT_EQUAL(0x04, pub_key[0]);  /* Uncompressed format marker */
    ESP_LOGI(TAG, "Key derivation successful, public key length: %zu", pub_key_len);

    /* Remap partition to get the private key */
    esp_secure_cert_unmap_partition();

    /* Get the derived private key (in DER format) */
    char *priv_key_der = NULL;
    uint32_t priv_key_der_len = 0;
    ret = esp_secure_cert_get_priv_key(&priv_key_der, &priv_key_der_len);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL(priv_key_der);
    TEST_ASSERT_GREATER_THAN(0, priv_key_der_len);
    ESP_LOGI(TAG, "Retrieved derived private key, length: %"PRIu32, priv_key_der_len);

    /* Test data to sign */
    const char *test_message = "Test message for ECDSA sign/verify";
    uint8_t hash[32] = {0};

#if (MBEDTLS_MAJOR_VERSION < 4)
    /* mbedtls 3.x: Use mbedtls APIs for hashing, signing, and verification */

    /* Compute SHA-256 hash of the message */
    int mbedtls_ret = mbedtls_sha256((const unsigned char *)test_message,
                                      strlen(test_message), hash, 0);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "SHA-256 hash failed");

    /* Initialize mbedtls structures for signing */
    mbedtls_pk_context pk_ctx;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;

    mbedtls_pk_init(&pk_ctx);
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);

    /* Seed the random number generator */
    const char *pers = "ecdsa_sign_verify_test";
    mbedtls_ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                         (const unsigned char *)pers, strlen(pers));
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "RNG seed failed");

    /* Parse the private key (DER format) */
    mbedtls_ret = mbedtls_pk_parse_key(&pk_ctx, (const unsigned char *)priv_key_der,
                                        priv_key_der_len + 1, NULL, 0,
                                        mbedtls_ctr_drbg_random, &ctr_drbg);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "Failed to parse private key");

    /* Sign the hash */
    uint8_t signature[MBEDTLS_ECDSA_MAX_LEN] = {0};
    size_t sig_len = 0;

    mbedtls_ret = mbedtls_pk_sign(&pk_ctx, MBEDTLS_MD_SHA256, hash, sizeof(hash),
                                   signature, sizeof(signature), &sig_len,
                                   mbedtls_ctr_drbg_random, &ctr_drbg);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "ECDSA signing failed");
    ESP_LOGI(TAG, "ECDSA signature created, length: %zu", sig_len);

    mbedtls_pk_free(&pk_ctx);

    /* Verify using the public key */
    mbedtls_ecdsa_context ecdsa_ctx;
    mbedtls_ecdsa_init(&ecdsa_ctx);

    /* Load the SECP256R1 curve */
    mbedtls_ret = mbedtls_ecp_group_load(&ecdsa_ctx.MBEDTLS_PRIVATE(grp), MBEDTLS_ECP_DP_SECP256R1);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "Failed to load ECP group");

    /* Import public key point (uncompressed format: 0x04 || X || Y) */
    mbedtls_ret = mbedtls_ecp_point_read_binary(&ecdsa_ctx.MBEDTLS_PRIVATE(grp),
                                                 &ecdsa_ctx.MBEDTLS_PRIVATE(Q),
                                                 pub_key, pub_key_len);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "Failed to import public key");

    /* Parse DER-encoded signature to extract r and s */
    mbedtls_mpi r, s;
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    const unsigned char *p = signature;
    const unsigned char *end = signature + sig_len;
    size_t len;

    mbedtls_ret = mbedtls_asn1_get_tag((unsigned char **)&p, end, &len,
                                        MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "Failed to parse signature sequence");

    mbedtls_ret = mbedtls_asn1_get_mpi((unsigned char **)&p, end, &r);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "Failed to parse signature r");

    mbedtls_ret = mbedtls_asn1_get_mpi((unsigned char **)&p, end, &s);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "Failed to parse signature s");

    /* Verify signature */
    mbedtls_ret = mbedtls_ecdsa_verify(&ecdsa_ctx.MBEDTLS_PRIVATE(grp),
                                        hash, sizeof(hash),
                                        &ecdsa_ctx.MBEDTLS_PRIVATE(Q),
                                        &r, &s);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "ECDSA signature verification failed");
    ESP_LOGI(TAG, "ECDSA signature verification successful!");

    /* Cleanup mbedtls 3.x */
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);
    mbedtls_ecdsa_free(&ecdsa_ctx);
    mbedtls_ctr_drbg_free(&ctr_drbg);
    mbedtls_entropy_free(&entropy);

#else
    /* mbedtls 4.x: Use PSA Crypto APIs */

    /* Initialize PSA Crypto */
    psa_status_t status = psa_crypto_init();
    TEST_ASSERT_EQUAL(PSA_SUCCESS, status);

    /* Compute SHA-256 hash of the message using PSA */
    size_t hash_len = 0;
    status = psa_hash_compute(PSA_ALG_SHA_256,
                               (const uint8_t *)test_message, strlen(test_message),
                               hash, sizeof(hash), &hash_len);
    TEST_ASSERT_EQUAL_MESSAGE(PSA_SUCCESS, status, "SHA-256 hash failed");

    /* Import the private key for signing */
    psa_key_attributes_t priv_key_attrs = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&priv_key_attrs, PSA_KEY_USAGE_SIGN_HASH);
    psa_set_key_algorithm(&priv_key_attrs, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
    psa_set_key_type(&priv_key_attrs, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&priv_key_attrs, 256);

    /* Parse DER private key via mbedtls_pk then import into PSA */
    mbedtls_pk_context pk_ctx;
    mbedtls_pk_init(&pk_ctx);
    int mbedtls_ret = mbedtls_pk_parse_key(&pk_ctx, (const unsigned char *)priv_key_der,
                                            priv_key_der_len + 1, NULL, 0);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "Failed to parse private key");

    psa_key_id_t sign_key_id = 0;
    psa_key_attributes_t sign_key_attrs = PSA_KEY_ATTRIBUTES_INIT;
    mbedtls_ret = mbedtls_pk_get_psa_attributes(&pk_ctx, PSA_KEY_USAGE_SIGN_HASH, &sign_key_attrs);
    TEST_ASSERT_EQUAL_MESSAGE(0, mbedtls_ret, "Failed to get PSA attributes");

    psa_set_key_algorithm(&sign_key_attrs, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
    status = mbedtls_pk_import_into_psa(&pk_ctx, &sign_key_attrs, &sign_key_id);
    TEST_ASSERT_EQUAL_MESSAGE(PSA_SUCCESS, status, "Failed to import private key into PSA");
    mbedtls_pk_free(&pk_ctx);

    /* Sign the hash */
    uint8_t signature[PSA_SIGNATURE_MAX_SIZE] = {0};
    size_t sig_len = 0;
    status = psa_sign_hash(sign_key_id, PSA_ALG_ECDSA(PSA_ALG_SHA_256),
                            hash, hash_len, signature, sizeof(signature), &sig_len);
    TEST_ASSERT_EQUAL_MESSAGE(PSA_SUCCESS, status, "ECDSA signing failed");
    ESP_LOGI(TAG, "ECDSA signature created via PSA, length: %zu", sig_len);
    psa_destroy_key(sign_key_id);

    /* Import the public key into PSA for verification */
    psa_key_attributes_t pub_key_attrs = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&pub_key_attrs, PSA_KEY_USAGE_VERIFY_HASH);
    psa_set_key_algorithm(&pub_key_attrs, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
    psa_set_key_type(&pub_key_attrs, PSA_KEY_TYPE_ECC_PUBLIC_KEY(PSA_ECC_FAMILY_SECP_R1));

    psa_key_id_t pub_key_id = 0;
    status = psa_import_key(&pub_key_attrs, pub_key, pub_key_len, &pub_key_id);
    TEST_ASSERT_EQUAL_MESSAGE(PSA_SUCCESS, status, "Failed to import public key into PSA");

    /* Verify the signature */
    status = psa_verify_hash(pub_key_id, PSA_ALG_ECDSA(PSA_ALG_SHA_256),
                              hash, hash_len, signature, sig_len);
    TEST_ASSERT_EQUAL_MESSAGE(PSA_SUCCESS, status, "ECDSA signature verification failed");
    ESP_LOGI(TAG, "ECDSA signature verification successful via PSA!");

    /* Cleanup PSA */
    psa_destroy_key(pub_key_id);
#endif /* MBEDTLS_MAJOR_VERSION */

    /* Get the key type to verify it's HMAC_DERIVED_ECDSA */
    esp_secure_cert_key_type_t key_type;
    ret = esp_secure_cert_get_priv_key_type(&key_type);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "Private key type: %d (expecting HMAC_DERIVED_ECDSA=%d)",
             key_type, ESP_SECURE_CERT_HMAC_DERIVED_ECDSA_KEY);
    TEST_ASSERT_EQUAL(ESP_SECURE_CERT_HMAC_DERIVED_ECDSA_KEY, key_type);

    /* Cleanup */
    esp_secure_cert_free_priv_key(priv_key_der);

    ESP_LOGI(TAG, "HMAC-ECDSA derive sign/verify test PASSED");
}
#endif /* SOC_HMAC_SUPPORTED */

TEST_GROUP_RUNNER(write)
{
    RUN_TEST_CASE(write, test_write_and_read);
    RUN_TEST_CASE(write, test_batch_write);
    RUN_TEST_CASE(write, test_error_handling);
    RUN_TEST_CASE(write, test_buffer_mode);
#if SOC_HMAC_SUPPORTED
    RUN_TEST_CASE(write, test_hmac_ecdsa_derivation_no_key);
    RUN_TEST_CASE(write, test_hmac_ecdsa_derivation_invalid_params);
    RUN_TEST_CASE(write, test_hmac_ecdsa_derivation_buffer_mode);
    RUN_TEST_CASE(write, test_hmac_ecdsa_derive_sign_verify);
#endif
}
