/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_err.h"
#include "esp_log.h"
#include "soc/soc_caps.h"
#include "esp_secure_cert_tlv_private.h"
#include "mbedtls/pk.h"

#if (MBEDTLS_MAJOR_VERSION < 4)
    #include "mbedtls/gcm.h"
    #include "mbedtls/pkcs5.h"
    #include "mbedtls/md.h"
#if __has_include("esp_idf_version.h")
    #include "esp_idf_version.h"
#endif /* __has_include("esp_idf_version.h") */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    #include "esp_random.h"
#else
    #include "esp_system.h"
#endif
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    #include "spi_flash_mmap.h"
    #include "esp_memory_utils.h"
    #include "entropy_poll.h"
#else
    #include "mbedtls/entropy_poll.h"
    #include "soc/soc_memory_layout.h"
#endif
#else
    #include "psa/crypto.h"
#endif /* (MBEDTLS_MAJOR_VERSION < 4) */

#if (MBEDTLS_VERSION_NUMBER < 0x03000000)
/* mbedtls 2.x backward compatibility */
#define MBEDTLS_2X_COMPAT 1
/**
 * Mbedtls-3.0 forward compatibility
 */
#ifndef MBEDTLS_PRIVATE
#define MBEDTLS_PRIVATE(member) member
#endif
#endif /* (MBEDTLS_VERSION_NUMBER < 0x03000000) */

#define HMAC_ENCRYPTION_IV_LEN                          (16)
static const char *TAG = "esp_secure_cert_crypto";

#if SOC_HMAC_SUPPORTED
#include "esp_hmac.h"
#define SHA256_MD_SIZE 32

int esp_pbkdf2_hmac_sha256(hmac_key_id_t hmac_key_id, const unsigned char *salt, size_t salt_len,
                           size_t iteration_count, size_t key_length, unsigned char *output)
{
    int ret = -1;
    int j;
    unsigned int i;
    unsigned char md1[SHA256_MD_SIZE] = {};
    unsigned char work[SHA256_MD_SIZE] = {};
    // Considering that we only have SHA256, fixing the MD_SIZE to 32 bytes
    const size_t MD_SIZE = SHA256_MD_SIZE;
    size_t use_len;
    unsigned char *out_p = output;
    unsigned char counter[4] = {};

    counter[3] = 1;
    uint8_t *hmac_input;
    esp_err_t esp_ret = ESP_FAIL;
    hmac_input = (uint8_t *) heap_caps_calloc(1, salt_len + sizeof(counter) + 1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (hmac_input == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for hmac input");
        return -1;
    }

    while (key_length) {
        // U1 ends up in work
        size_t hmac_input_len = 0;
        memcpy(hmac_input, salt, salt_len);
        hmac_input_len = hmac_input_len + salt_len;
        memcpy(hmac_input + salt_len, counter, sizeof(counter));
        hmac_input_len = hmac_input_len + sizeof(counter);
        esp_ret = esp_hmac_calculate(hmac_key_id, hmac_input, hmac_input_len, work);
        if (esp_ret != ESP_OK) {
            ESP_LOGE(TAG, "Could not calculate the HMAC value, returned %04X", esp_ret);
            ret = -1;
            goto cleanup;
        }

        memcpy(md1, work, MD_SIZE);

        for (i = 1; i < iteration_count; i++) {
            // U2 ends up in md1
            esp_ret = esp_hmac_calculate(hmac_key_id, md1, MD_SIZE, md1);
            if (esp_ret != ESP_OK) {
                ESP_LOGE(TAG, "Could not calculate the HMAC value, returned %04X", esp_ret);
                ret = -1;
                goto cleanup;
            }
            // U1 xor U2
            //
            for (j = 0; j < MD_SIZE; j++) {
                work[j] ^= md1[j];
            }
        }

        use_len = (key_length < MD_SIZE) ? key_length : MD_SIZE;
        memcpy(out_p, work, use_len);

        key_length -= (uint32_t) use_len;
        out_p += use_len;

        for (i = 4; i > 0; i--) {
            if (++counter[i - 1] != 0) {
                break;
            }
        }
    }
    //Success
    ret = 0;
cleanup:

    /* Zeroise buffers to clear sensitive data from memory. */
    free(hmac_input);
    memset(work, 0, SHA256_MD_SIZE);
    memset(md1, 0, SHA256_MD_SIZE);
    return ret;
}
#endif

#if (MBEDTLS_MAJOR_VERSION < 4)
#define HMAC_ENCRYPTION_RANDOM_DELAY_LIMIT 100
static esp_err_t esp_secure_cert_crypto_gcm_decrypt_internal(const uint8_t *in_buf, uint32_t len, uint8_t *output_buf, const uint8_t *key, size_t key_len,
                                const uint8_t *iv, const uint8_t *aad, const uint8_t *tag, size_t tag_len)
{
    mbedtls_gcm_context gcm_ctx;
    mbedtls_gcm_init(&gcm_ctx);
    int ret = mbedtls_gcm_setkey(&gcm_ctx, MBEDTLS_CIPHER_ID_AES, key, key_len * 8);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failure at mbedtls_gcm_setkey with error code : -0x%04X", -ret);
        mbedtls_gcm_free(&gcm_ctx);
        return ESP_FAIL;
    }

    uint32_t rand_delay;
    rand_delay = esp_random() % HMAC_ENCRYPTION_RANDOM_DELAY_LIMIT;
    esp_rom_delay_us(rand_delay);

    // len = len - HMAC_ENCRYPTION_TAG_LEN;
    ret = mbedtls_gcm_auth_decrypt(&gcm_ctx, len, iv,
                                   HMAC_ENCRYPTION_IV_LEN, NULL, 0,
                                   tag,
                                   tag_len,
                                   in_buf,
                                   output_buf);

    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to decrypt the data, mbedtls_gcm_crypt_and_tag returned %02X", ret);
        mbedtls_gcm_free(&gcm_ctx);
        return ESP_FAIL;
    }

    return ESP_OK;
}
#else
static esp_err_t esp_secure_cert_crypto_gcm_decrypt_internal(const uint8_t *in_buf, uint32_t len, uint8_t *output_buf, const uint8_t *key, size_t key_len,
                                const uint8_t *iv, const uint8_t *aad, const uint8_t *tag, size_t tag_len)
{
    int ret = ESP_FAIL;
    psa_status_t status;
    psa_aead_operation_t operation = PSA_AEAD_OPERATION_INIT;

    psa_key_id_t key_id = 0;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DECRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_GCM);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, key_len * 8);
    status = psa_import_key(&attributes, key, key_len, &key_id);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to import the key, returned %04X", status);
        goto exit;
    }

    status = psa_aead_decrypt_setup(&operation, key_id, PSA_ALG_GCM);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to setup the operation, returned %04X", status);
        goto exit;
    }
    status = psa_aead_set_nonce(&operation, iv, HMAC_ENCRYPTION_IV_LEN);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to set the nonce, returned %04X", status);
        goto exit;
    }
    status = psa_aead_update_ad(&operation, aad, 0);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to update the ad, returned %04X", status);
        goto exit;
    }
    size_t olen = 0;
    size_t olen_tag = 0;
    status = psa_aead_update(&operation, in_buf, len, output_buf, len, &olen);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to update the data, returned %04X", status);
        goto exit;
    }
    status = psa_aead_verify(&operation, output_buf + olen, len - olen, &olen_tag, tag, tag_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to finish the operation, returned %d", status);
        goto exit;
    }

    ret = ESP_OK;

exit:
    psa_destroy_key(key_id);
    return ret;
}
#endif

esp_err_t esp_secure_cert_crypto_gcm_decrypt(const uint8_t *in_buf, uint32_t len, uint8_t *output_buf, const uint8_t *key,
                                size_t key_len, const uint8_t *iv, const uint8_t *aad, const uint8_t *tag, size_t tag_len)
{
    return esp_secure_cert_crypto_gcm_decrypt_internal(in_buf, len, output_buf, key, key_len, iv, aad, tag, tag_len);
}

#if (MBEDTLS_MAJOR_VERSION < 4)

static int myrand(void *rng_state, unsigned char *output, size_t len)
{
    size_t olen;
    (void) olen;
    return mbedtls_hardware_poll(rng_state, output, len, &olen);
}

static esp_err_t esp_secure_cert_convert_key_to_der_internal(const char *key_buf, size_t key_buf_len, uint8_t* output_buf, size_t output_buf_len)
{
    esp_err_t ret = ESP_FAIL;
    // Convert the private key to der
    mbedtls_pk_context key;
    mbedtls_pk_init(&key);
    ret = mbedtls_pk_setup(&key, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY));
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to setup pk key, returned %04X", ret);
        goto exit;
    }

    mbedtls_ecdsa_context *key_ctx = mbedtls_pk_ec(key);
    ret = mbedtls_ecp_group_load(&key_ctx->MBEDTLS_PRIVATE(grp), MBEDTLS_ECP_DP_SECP256R1);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to load the ecp group, returned %04X", ret);
        goto exit;
    }

    ret = mbedtls_mpi_read_binary(&key_ctx->MBEDTLS_PRIVATE(d), (const unsigned char *) key_buf, key_buf_len);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to read binary, returned %04X", ret);
        goto exit;
    }

    // Calculate the public key
    ret = mbedtls_ecp_mul(&key_ctx->MBEDTLS_PRIVATE(grp), &key_ctx->MBEDTLS_PRIVATE(Q), &key_ctx->MBEDTLS_PRIVATE(d), &key_ctx->MBEDTLS_PRIVATE(grp).G, myrand, NULL);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to generate public key, returned %04X", ret);
        goto exit;
    }

    // Write the private key in DER format
    int mbedtls_ret = mbedtls_pk_write_key_der(&key, output_buf, output_buf_len);
    if (mbedtls_ret != output_buf_len) {
        ESP_LOGE(TAG, "Failed to write the pem key, returned -%d", mbedtls_ret);
        goto exit;
    }
    ret = ESP_OK;

exit:
    mbedtls_pk_free(&key);
    return ret;
}
#else
static esp_err_t esp_secure_cert_convert_key_to_der_internal(const char *key_buf, size_t key_buf_len, uint8_t* output_buf, size_t output_buf_len)
{
    esp_err_t ret = ESP_FAIL;
    psa_status_t status;
    psa_key_id_t key_id = 0;
    mbedtls_pk_context key;
    mbedtls_pk_init(&key);

    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_EXPORT);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECDH);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attributes, key_buf_len * 8);

    status = psa_import_key(&attributes, (unsigned char *)key_buf, key_buf_len, &key_id);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to import the key, returned %04X", status);
        goto exit;
    }

    int mbedtls_ret = mbedtls_pk_copy_from_psa(key_id, &key);
    if (mbedtls_ret != 0) {
        ESP_LOGE(TAG, "Failed to copy the key, returned %04X", mbedtls_ret);
        goto exit;
    }

    mbedtls_ret = mbedtls_pk_write_key_der(&key, output_buf, output_buf_len);
    if (mbedtls_ret != ESP_SECURE_CERT_ECDSA_DER_KEY_SIZE) {
        ESP_LOGE(TAG, "Failed to write the pem key, returned %04X", mbedtls_ret);
        goto exit;
    }

    ret = ESP_OK;

exit:
    mbedtls_pk_free(&key);
    psa_destroy_key(key_id);
    return ret;
}
#endif

esp_err_t esp_secure_cert_convert_key_to_der(const char *key_buf, size_t key_buf_len, uint8_t* output_buf, size_t output_buf_len)
{
    return esp_secure_cert_convert_key_to_der_internal(key_buf, key_buf_len, output_buf, output_buf_len);
}

#if SOC_HMAC_SUPPORTED

/* Include headers based on mbedtls version */
#if (MBEDTLS_MAJOR_VERSION < 4)
#include "mbedtls/ecp.h"
#include "mbedtls/bignum.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/md.h"
#else
#include "psa/crypto.h"
#endif

/**
 * @brief Validate that a raw key is a valid ECDSA private key for SECP256R1
 *
 * A valid private key d must satisfy: 1 < d < N, where N is the curve order.
 *
 * @param[in] key_buf Raw key buffer (32 bytes for SECP256R1)
 * @param[in] key_len Length of the key buffer
 *
 * @return ESP_OK if valid, ESP_FAIL otherwise
 */
esp_err_t esp_secure_cert_validate_ecdsa_key(const uint8_t *key_buf, size_t key_len)
{
    if (key_buf == NULL || key_len != ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Quick checks for obviously invalid keys */
    bool all_zero = true;
    bool all_ones = true;
    for (size_t i = 0; i < key_len; i++) {
        if (key_buf[i] != 0) {
            all_zero = false;
        }
        if (key_buf[i] != 0xFF) {
            all_ones = false;
        }
    }
    if (all_zero) {
        ESP_LOGD(TAG, "Key is all zeros, invalid");
        return ESP_FAIL;
    }
    if (all_ones) {
        ESP_LOGD(TAG, "Key is all 0xFF, likely invalid");
        return ESP_FAIL;
    }

#if (MBEDTLS_MAJOR_VERSION < 4)
    /* mbedtls 3.x: Use bignum and ECP to validate 1 < d < N */
    esp_err_t ret = ESP_FAIL;
    mbedtls_mpi d;
    mbedtls_mpi one;
    mbedtls_ecp_group grp;

    mbedtls_mpi_init(&d);
    mbedtls_mpi_init(&one);
    mbedtls_ecp_group_init(&grp);

    /* Load SECP256R1 curve parameters */
    int mbedtls_ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1);
    if (mbedtls_ret != 0) {
        ESP_LOGE(TAG, "Failed to load ECP group: -0x%04X", -mbedtls_ret);
        goto cleanup_mbedtls3;
    }

    /* Read the key as a big integer */
    mbedtls_ret = mbedtls_mpi_read_binary(&d, key_buf, key_len);
    if (mbedtls_ret != 0) {
        ESP_LOGE(TAG, "Failed to read key as MPI: -0x%04X", -mbedtls_ret);
        goto cleanup_mbedtls3;
    }

    /* Set one = 1 for comparison */
    mbedtls_ret = mbedtls_mpi_lset(&one, 1);
    if (mbedtls_ret != 0) {
        ESP_LOGE(TAG, "Failed to set one: -0x%04X", -mbedtls_ret);
        goto cleanup_mbedtls3;
    }

    /* Check: d > 1 */
    if (mbedtls_mpi_cmp_mpi(&d, &one) <= 0) {
        ESP_LOGD(TAG, "Key is <= 1, invalid");
        goto cleanup_mbedtls3;
    }

    /* Check: d < N (curve order) */
    if (mbedtls_mpi_cmp_mpi(&d, &grp.N) >= 0) {
        ESP_LOGD(TAG, "Key is >= curve order N, invalid");
        goto cleanup_mbedtls3;
    }

    ret = ESP_OK;

cleanup_mbedtls3:
    mbedtls_mpi_free(&d);
    mbedtls_mpi_free(&one);
    mbedtls_ecp_group_free(&grp);
    return ret;

#else
    /* mbedtls 4.x: Use PSA to validate by attempting to import */
    psa_status_t status;

    /* Ensure PSA crypto is initialized */
    status = psa_crypto_init();
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_crypto_init failed: %d", (int)status);
        return ESP_FAIL;
    }

    /* Try to import the key as an ECC private key - PSA will validate it */
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attributes, 256);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH | PSA_KEY_USAGE_EXPORT);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECDSA(PSA_ALG_SHA_256));

    psa_key_id_t key_id = 0;
    status = psa_import_key(&attributes, key_buf, key_len, &key_id);

    if (status != PSA_SUCCESS) {
        ESP_LOGD(TAG, "psa_import_key failed (invalid key): %d", (int)status);
        return ESP_FAIL;
    }

    /* Key is valid, clean up */
    psa_destroy_key(key_id);
    return ESP_OK;
#endif /* MBEDTLS_MAJOR_VERSION */
}

/**
 * @brief Derive ECDSA key using software PBKDF2-HMAC-SHA256
 *
 * This function performs PBKDF2 derivation in software,
 * which is used during key generation before the HMAC key is burned to eFuse.
 *
 * @param[in] hmac_key The HMAC key (32 bytes)
 * @param[in] hmac_key_len Length of HMAC key
 * @param[in] salt Salt value
 * @param[in] salt_len Length of salt
 * @param[in] iterations Number of PBKDF2 iterations
 * @param[out] output Output buffer for derived key
 * @param[in] output_len Length of output (32 bytes for ECDSA)
 *
 * @return ESP_OK on success, ESP_FAIL on failure
 */
esp_err_t esp_secure_cert_sw_pbkdf2_hmac_sha256(const uint8_t *hmac_key, size_t hmac_key_len,
                                                  const uint8_t *salt, size_t salt_len,
                                                  uint32_t iterations,
                                                  uint8_t *output, size_t output_len)
{
    if (hmac_key == NULL || salt == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#if (MBEDTLS_MAJOR_VERSION < 4)
    int ret;

#if (MBEDTLS_VERSION_NUMBER < 0x03000000)
    /* mbedtls 2.x: Use mbedtls_pkcs5_pbkdf2_hmac with md_context */
    mbedtls_md_context_t md_ctx;
    mbedtls_md_init(&md_ctx);

    ret = mbedtls_md_setup(&md_ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_md_setup failed: -0x%04X", -ret);
        mbedtls_md_free(&md_ctx);
        return ESP_FAIL;
    }

    ret = mbedtls_pkcs5_pbkdf2_hmac(&md_ctx, hmac_key, hmac_key_len,
                                     salt, salt_len, iterations,
                                     output_len, output);
    mbedtls_md_free(&md_ctx);
#else
    /* mbedtls 3.x: Use mbedtls_pkcs5_pbkdf2_hmac_ext (context-free API) */
    ret = mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256,
                                         hmac_key, hmac_key_len,
                                         salt, salt_len, iterations,
                                         output_len, output);
#endif /* MBEDTLS_VERSION_NUMBER */

    if (ret != 0) {
        ESP_LOGE(TAG, "PBKDF2 derivation failed: -0x%04X", -ret);
        return ESP_FAIL;
    }

    return ESP_OK;

#else
    /* mbedtls 4.x: Use PSA key derivation */
    psa_status_t status;

    /* Ensure PSA crypto is initialized */
    status = psa_crypto_init();
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_crypto_init failed: %d", (int)status);
        return ESP_FAIL;
    }

    /* Import the password (HMAC key) as a PSA key */
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_DERIVE);
    psa_set_key_algorithm(&attributes, PSA_ALG_PBKDF2_HMAC(PSA_ALG_SHA_256));
    psa_set_key_type(&attributes, PSA_KEY_TYPE_PASSWORD);

    psa_key_id_t key_id = 0;
    status = psa_import_key(&attributes, hmac_key, hmac_key_len, &key_id);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_import_key failed: %d", (int)status);
        return ESP_FAIL;
    }

    /* Set up the key derivation operation */
    psa_key_derivation_operation_t operation = PSA_KEY_DERIVATION_OPERATION_INIT;
    status = psa_key_derivation_setup(&operation, PSA_ALG_PBKDF2_HMAC(PSA_ALG_SHA_256));
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_key_derivation_setup failed: %d", (int)status);
        psa_destroy_key(key_id);
        return ESP_FAIL;
    }

    /* Provide the cost (iteration count) */
    status = psa_key_derivation_input_integer(&operation, PSA_KEY_DERIVATION_INPUT_COST, iterations);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_key_derivation_input_integer failed: %d", (int)status);
        psa_key_derivation_abort(&operation);
        psa_destroy_key(key_id);
        return ESP_FAIL;
    }

    /* Provide the salt */
    status = psa_key_derivation_input_bytes(&operation, PSA_KEY_DERIVATION_INPUT_SALT, salt, salt_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_key_derivation_input_bytes (salt) failed: %d", (int)status);
        psa_key_derivation_abort(&operation);
        psa_destroy_key(key_id);
        return ESP_FAIL;
    }

    /* Provide the password (from the imported key) */
    status = psa_key_derivation_input_key(&operation, PSA_KEY_DERIVATION_INPUT_PASSWORD, key_id);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_key_derivation_input_key failed: %d", (int)status);
        psa_key_derivation_abort(&operation);
        psa_destroy_key(key_id);
        return ESP_FAIL;
    }

    /* Derive the output key material */
    status = psa_key_derivation_output_bytes(&operation, output, output_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_key_derivation_output_bytes failed: %d", (int)status);
        psa_key_derivation_abort(&operation);
        psa_destroy_key(key_id);
        return ESP_FAIL;
    }

    /* Clean up */
    psa_key_derivation_abort(&operation);
    psa_destroy_key(key_id);

    return ESP_OK;
#endif /* MBEDTLS_MAJOR_VERSION */
}

/**
 * @brief Calculate public key from private key for SECP256R1
 *
 * Computes Q = d * G where d is the private key and G is the generator point.
 *
 * @param[in] priv_key_buf Raw private key (32 bytes)
 * @param[in] priv_key_len Private key length
 * @param[out] pub_key_buf Output buffer for uncompressed public key (65 bytes: 0x04 || X || Y)
 * @param[in,out] pub_key_len Input: buffer size, Output: actual size written
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_secure_cert_calc_public_key(const uint8_t *priv_key_buf, size_t priv_key_len,
                                           uint8_t *pub_key_buf, size_t *pub_key_len)
{
    if (priv_key_buf == NULL || pub_key_buf == NULL || pub_key_len == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (priv_key_len != ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Uncompressed point format: 0x04 || X (32 bytes) || Y (32 bytes) = 65 bytes */
    if (*pub_key_len < 65) {
        return ESP_ERR_INVALID_SIZE;
    }

#if (MBEDTLS_MAJOR_VERSION < 4)
    /* mbedtls 3.x: Use ECP to compute Q = d * G */
    esp_err_t ret = ESP_FAIL;
    mbedtls_ecp_group grp;
    mbedtls_mpi d;
    mbedtls_ecp_point Q;

    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);

    int mbedtls_ret = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1);
    if (mbedtls_ret != 0) {
        ESP_LOGE(TAG, "Failed to load ECP group: -0x%04X", -mbedtls_ret);
        goto cleanup_mbedtls3_pub;
    }

    mbedtls_ret = mbedtls_mpi_read_binary(&d, priv_key_buf, priv_key_len);
    if (mbedtls_ret != 0) {
        ESP_LOGE(TAG, "Failed to read private key: -0x%04X", -mbedtls_ret);
        goto cleanup_mbedtls3_pub;
    }

    /* Q = d * G */
    mbedtls_ret = mbedtls_ecp_mul(&grp, &Q, &d, &grp.G, myrand, NULL);
    if (mbedtls_ret != 0) {
        ESP_LOGE(TAG, "Failed to calculate public key: -0x%04X", -mbedtls_ret);
        goto cleanup_mbedtls3_pub;
    }

    /* Write public key in uncompressed format */
    size_t olen = 0;
    mbedtls_ret = mbedtls_ecp_point_write_binary(&grp, &Q, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                                  &olen, pub_key_buf, *pub_key_len);
    if (mbedtls_ret != 0) {
        ESP_LOGE(TAG, "Failed to write public key: -0x%04X", -mbedtls_ret);
        goto cleanup_mbedtls3_pub;
    }

    *pub_key_len = olen;
    ret = ESP_OK;

cleanup_mbedtls3_pub:
    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);
    mbedtls_ecp_group_free(&grp);
    return ret;

#else
    /* mbedtls 4.x: Use PSA to import and export */
    psa_status_t status;

    /* Ensure PSA crypto is initialized */
    status = psa_crypto_init();
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "psa_crypto_init failed: %d", (int)status);
        return ESP_FAIL;
    }

    /* Import private key */
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_EXPORT);
    psa_set_key_algorithm(&attributes, PSA_ALG_ECDSA_ANY);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attributes, 256);

    psa_key_id_t key_id = 0;
    status = psa_import_key(&attributes, priv_key_buf, priv_key_len, &key_id);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to import private key: %d", (int)status);
        return ESP_FAIL;
    }

    /* Export public key (uncompressed format) */
    size_t pub_key_actual_len = 0;
    status = psa_export_public_key(key_id, pub_key_buf, *pub_key_len, &pub_key_actual_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to export public key: %d", (int)status);
        psa_destroy_key(key_id);
        return ESP_FAIL;
    }

    *pub_key_len = pub_key_actual_len;
    psa_destroy_key(key_id);
    return ESP_OK;
#endif /* MBEDTLS_MAJOR_VERSION */
}
#endif /* SOC_HMAC_SUPPORTED */
