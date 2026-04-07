/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ESP Secure Cert App

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "soc/soc_caps.h"
#include "esp_secure_cert_read.h"
#include "esp_secure_cert_tlv_read.h"
#include "esp_secure_cert_signature_verify.h"

#ifdef CONFIG_EXAMPLE_ESP_SECURE_CERT_WRITE_DEMO
#include "esp_secure_cert_write.h"
#include "esp_secure_cert_tlv_private.h"
#include "esp_partition.h"
#endif

#include "mbedtls/ssl.h"
#include "mbedtls/pk.h"
#include "mbedtls/x509.h"
#if (MBEDTLS_VERSION_NUMBER < 0x04000000)
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#else
#include "psa/crypto.h"
#endif // MBEDTLS_VERSION_NUMBER < 0x04000000
#include "mbedtls/error.h"
#include "esp_idf_version.h"

#if SOC_ECDSA_SUPPORTED
#if (MBEDTLS_VERSION_NUMBER < 0x04000000)
#include "ecdsa/ecdsa_alt.h"
#else
#include "psa_crypto_driver_esp_ecdsa.h"
#endif
#endif

#define TAG "esp_secure_cert_app"

// Modular function to print certificate or key data in PEM or DER format
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

#ifdef CONFIG_ESP_SECURE_CERT_DS_PERIPHERAL

#define SIG_SIZE 1000

static esp_err_t test_ciphertext_validity(esp_ds_data_ctx_t *ds_data, unsigned char *dev_cert, size_t dev_cert_len)
{
    mbedtls_x509_crt crt;
    mbedtls_x509_crt_init(&crt);
    unsigned char *sig = NULL;

    if (ds_data == NULL || dev_cert == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int ret = mbedtls_x509_crt_parse(&crt, dev_cert, dev_cert_len);
    if (ret < 0) {
        ESP_LOGE(TAG, "Parsing of device certificate failed, returned %02X", ret);
        goto exit;
    }

#if (MBEDTLS_VERSION_NUMBER >= 0x04000000)
    psa_key_id_t key_id = PSA_KEY_ID_NULL;
    psa_status_t status = PSA_ERROR_GENERIC_ERROR;
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_algorithm_t alg = PSA_ALG_RSA_PKCS1V15_SIGN(PSA_ALG_SHA_256);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_RSA_KEY_PAIR);
    psa_set_key_bits(&attributes, ds_data->rsa_length_bits);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH);
    psa_set_key_algorithm(&attributes, alg);
    psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_ESP_RSA_DS);
    status = psa_import_key(&attributes, (const uint8_t *)ds_data, sizeof(esp_ds_data_ctx_t), &key_id);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to import the DS key, returned %d", status);
        goto exit;
    }
#else
    esp_err_t esp_ret = esp_ds_init_data_ctx(ds_data);
    if (esp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialze the DS context");
        return esp_ret;
    }
#endif /* MBEDTLS_VERSION_NUMBER >= 0x04000000 */

    const size_t sig_len = 256;
    uint32_t hash[8] = {[0 ... 7] = 0xAABBCCDD};

    sig = (unsigned char *) calloc(1, SIG_SIZE * sizeof(char));
    if (sig == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for signature");
        goto exit;
    }
    size_t sig_len_out = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    ret = esp_ds_rsa_sign(NULL, NULL, NULL, 0, MBEDTLS_MD_SHA256, 0, (const unsigned char *) hash, sig);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to sign the data with rsa key, returned %02X", ret);
        goto exit;
    }
    esp_ds_release_ds_lock();
#elif ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    ret = esp_ds_rsa_sign(NULL, NULL, NULL, MBEDTLS_MD_SHA256, 0, (const unsigned char *) hash, sig);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to sign the data with rsa key, returned %02X", ret);
        goto exit;
    }
    esp_ds_release_ds_lock();
#else
    status = psa_sign_hash(key_id, alg, (const unsigned char *) hash, sizeof(hash), sig, SIG_SIZE, &sig_len_out);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to sign the data with rsa key, returned %d", status);
        goto exit;
    }
    psa_destroy_key(key_id);
    key_id = PSA_KEY_ID_NULL;
#endif

    ret = mbedtls_pk_verify(&crt.pk, MBEDTLS_MD_SHA256, (const unsigned char *) hash, 0, sig, sig_len);
    if (ret != 0) {
        printf("\nFailed to verify the data\n");
        goto exit;
    }
    free(sig);
    return ESP_OK;
exit:
    free(sig);
    mbedtls_x509_crt_free(&crt);
    printf("\nFailed to verify the ciphertext\n");
#if (MBEDTLS_VERSION_NUMBER < 0x04000000)
    esp_ds_release_ds_lock();
#else
    if (key_id != PSA_KEY_ID_NULL) {
        psa_destroy_key(key_id);
    }
#endif
    return ESP_FAIL;
}
#else
static esp_err_t test_priv_key_validity(unsigned char* priv_key, size_t priv_key_len, unsigned char *dev_cert, size_t dev_cert_len)
{
    mbedtls_x509_crt crt;
#if (MBEDTLS_VERSION_NUMBER < 0x04000000)
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
#else
    psa_key_id_t priv_key_id = 0;
    psa_key_id_t pub_key_id = 0;
    psa_algorithm_t sign_alg = 0;
    psa_algorithm_t verify_alg = 0;
    psa_status_t status = PSA_ERROR_GENERIC_ERROR;
#endif // MBEDTLS_VERSION_NUMBER < 0x04000000
    mbedtls_pk_context pk;
    unsigned char *sig = NULL;

#if (MBEDTLS_VERSION_NUMBER < 0x04000000)
    mbedtls_ctr_drbg_init(&ctr_drbg);
    mbedtls_entropy_init(&entropy);
#endif // MBEDTLS_VERSION_NUMBER < 0x04000000
    mbedtls_x509_crt_init(&crt);
    mbedtls_pk_init(&pk);
    esp_err_t esp_ret = ESP_FAIL;
    if (priv_key == NULL || dev_cert == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int ret = mbedtls_x509_crt_parse(&crt, dev_cert, dev_cert_len);
    if (ret != 0) {
        ESP_LOGE(TAG, "Parsing of device certificate failed, returned %02X", ret);
        esp_ret = ESP_FAIL;
        goto exit;
    } else {
        ESP_LOGI(TAG, "Successfully parsed the certificate");
    }

#if (MBEDTLS_VERSION_NUMBER < 0x04000000)
    static const char *pers = "Hello";
    ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy, (const unsigned char *) pers, strlen(pers));
    if (ret != 0) {
        ESP_LOGE(TAG, "mbedtls_ctr_drbg_seed returned -0x%04x", -ret );
        esp_ret = ESP_FAIL;
        goto exit;
    }
#endif // MBEDTLS_VERSION_NUMBER < 0x04000000

    esp_secure_cert_key_type_t key_type = ESP_SECURE_CERT_DEFAULT_FORMAT_KEY;
#ifndef CONFIG_ESP_SECURE_CERT_SUPPORT_LEGACY_FORMATS
    esp_ret = esp_secure_cert_get_priv_key_type(&key_type);
    if (esp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to obtain the priv key type");
        goto exit;
    }
#endif
#if SOC_ECDSA_SUPPORTED
    if (key_type == ESP_SECURE_CERT_ECDSA_PERIPHERAL_KEY) {
        ESP_LOGI(TAG, "Setting up the ECDSA key from eFuse");
        uint8_t efuse_block_id;
        esp_ret = esp_secure_cert_get_priv_key_efuse_id(&efuse_block_id);
        if (esp_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to obtain efuse key id");
            goto exit;
        }
        ESP_LOGI(TAG, "Using key from eFuse block %d for ECDSA key", efuse_block_id);

#if (MBEDTLS_VERSION_NUMBER < 0x04000000)
        esp_ecdsa_pk_conf_t pk_conf = {
            .grp_id = MBEDTLS_ECP_DP_SECP256R1,
            .efuse_block = efuse_block_id,
        };
        if (esp_ecdsa_set_pk_context(&pk, &pk_conf) != 0) {
            ESP_LOGE(TAG, "Failed to set ECDSA context");
            esp_ret = ESP_FAIL;
            goto exit;
        }
        ESP_LOGI(TAG, "Successfully set ECDSA key context");
#else
        psa_key_attributes_t key_attr = PSA_KEY_ATTRIBUTES_INIT;

#if CONFIG_MBEDTLS_ECDSA_DETERMINISTIC && SOC_ECDSA_SUPPORT_DETERMINISTIC_MODE
        sign_alg = PSA_ALG_DETERMINISTIC_ECDSA(PSA_ALG_SHA_256);
#else
        sign_alg = PSA_ALG_ECDSA(PSA_ALG_SHA_256);
#endif
        // Set attributes for opaque private key
        psa_set_key_type(&key_attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
        psa_set_key_bits(&key_attr, 256);
        psa_set_key_usage_flags(&key_attr, PSA_KEY_USAGE_SIGN_HASH);
        psa_set_key_algorithm(&key_attr, sign_alg);
        psa_set_key_lifetime(&key_attr, PSA_KEY_LIFETIME_ESP_ECDSA_VOLATILE);

        esp_ecdsa_opaque_key_t opaque_key = {
            .curve = ESP_ECDSA_CURVE_SECP256R1,
            .efuse_block = efuse_block_id,
            .use_km_key = false,
        };

        // Import opaque key reference
        status = psa_import_key(&key_attr, (uint8_t*) &opaque_key, sizeof(opaque_key), &priv_key_id);
        psa_reset_key_attributes(&key_attr);
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "Failed to import opaque key reference");
            esp_ret = ESP_FAIL;
            goto exit;
        }
#endif
    } else
#endif /* SOC_ECDSA_SUPPORTED */
    {
#if (MBEDTLS_VERSION_NUMBER > 0x03000000) && (MBEDTLS_VERSION_NUMBER < 0x04000000)
        ret = mbedtls_pk_parse_key(&pk, (const uint8_t *)priv_key, priv_key_len, NULL, 0, mbedtls_ctr_drbg_random, &ctr_drbg);
#else
        ret = mbedtls_pk_parse_key(&pk, (const uint8_t *)priv_key, priv_key_len, NULL, 0);
#endif
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to parse the key");
            esp_ret = ESP_FAIL;
            goto exit;
        } else {
            ESP_LOGI(TAG, "Successfully parsed the key");
        }
    }

    static uint32_t hash[8] = {[0 ... 7] = 0xAABBCCDD};
#define SIG_SIZE 1024
    sig = (unsigned char*)calloc(1, SIG_SIZE * sizeof(char));
    if (sig == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        esp_ret = ESP_FAIL;
        goto exit;
    }
    size_t sig_len = 0;
#if (MBEDTLS_VERSION_NUMBER < 0x03000000)
    ret = mbedtls_pk_sign(&pk, MBEDTLS_MD_SHA256, (const unsigned char *) hash, 0, sig, &sig_len, mbedtls_ctr_drbg_random, &ctr_drbg);
#elif (MBEDTLS_VERSION_NUMBER < 0x04000000)
    ret = mbedtls_pk_sign(&pk, MBEDTLS_MD_SHA256, (const unsigned char *) hash, 0, sig, SIG_SIZE, &sig_len, mbedtls_ctr_drbg_random, &ctr_drbg);
#else
    if (key_type != ESP_SECURE_CERT_ECDSA_PERIPHERAL_KEY) {
        psa_key_attributes_t priv_key_attr = PSA_KEY_ATTRIBUTES_INIT;
        ret = mbedtls_pk_get_psa_attributes(&pk, PSA_KEY_USAGE_SIGN_HASH, &priv_key_attr);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to get the psa attributes");
            esp_ret = ESP_FAIL;
            goto exit;
        }

        psa_key_type_t key_type = psa_get_key_type(&priv_key_attr);
        if (PSA_KEY_TYPE_IS_RSA(key_type)) {
            sign_alg = PSA_ALG_RSA_PKCS1V15_SIGN(PSA_ALG_SHA_256);
        } else if (PSA_KEY_TYPE_IS_ECC(key_type)) {
#if CONFIG_MBEDTLS_ECDSA_DETERMINISTIC
            sign_alg = PSA_ALG_DETERMINISTIC_ECDSA(PSA_ALG_SHA_256);
#else
            sign_alg = PSA_ALG_ECDSA(PSA_ALG_SHA_256);
#endif
        }

        // Import the private key into PSA
        status = mbedtls_pk_import_into_psa(&pk, &priv_key_attr, &priv_key_id);
        if (status != PSA_SUCCESS) {
            ESP_LOGE(TAG, "Failed to import key into PSA with error %d", status);
            esp_ret = ESP_FAIL;
            goto exit;
        }
        psa_reset_key_attributes(&priv_key_attr);
    }

    status = psa_sign_hash(priv_key_id, sign_alg, (const unsigned char *) hash, sizeof(hash), sig, SIG_SIZE, &sig_len);
    if (status != PSA_SUCCESS) {
        ret = -1;
    } else {
        ret = 0;
    }
    psa_destroy_key(priv_key_id);
#endif
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to sign the data");
        esp_ret = ESP_FAIL;
        goto exit;
    } else {
        ESP_LOGI(TAG, "Successfully signed the data");
    }

#if (MBEDTLS_VERSION_NUMBER >= 0x04000000)
    psa_key_attributes_t pub_key_attr = PSA_KEY_ATTRIBUTES_INIT;
    ret = mbedtls_pk_get_psa_attributes(&crt.pk, PSA_KEY_USAGE_VERIFY_HASH, &pub_key_attr);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to get the psa attributes, returned %d", ret);
        esp_ret = ESP_FAIL;
        goto exit;
    }

    key_type = psa_get_key_type(&pub_key_attr);

    if (PSA_KEY_TYPE_IS_RSA(key_type)) {
        verify_alg = PSA_ALG_RSA_PKCS1V15_SIGN(PSA_ALG_SHA_256);
    } else if (PSA_KEY_TYPE_IS_ECC(key_type)) {
#if CONFIG_MBEDTLS_ECDSA_DETERMINISTIC
        verify_alg = PSA_ALG_DETERMINISTIC_ECDSA(PSA_ALG_SHA_256);
#else
        verify_alg = PSA_ALG_ECDSA(PSA_ALG_SHA_256);
#endif
    }

    // Import the public key into PSA
    status = mbedtls_pk_import_into_psa(&crt.pk, &pub_key_attr, &pub_key_id);
    psa_reset_key_attributes(&pub_key_attr);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to import key into PSA with error %d", status);
        esp_ret = ESP_FAIL;
        goto exit;
    }

    status = psa_verify_hash(pub_key_id, verify_alg, (const unsigned char *) hash, sizeof(hash), sig, sig_len);
    if (status != PSA_SUCCESS) {
        ESP_LOGE(TAG, "Failed to verify the signed data with error %d", status);
        ret = -1;
    } else {
        ret = 0;
    }
    psa_destroy_key(pub_key_id);
#else
    ret = mbedtls_pk_verify(&crt.pk, MBEDTLS_MD_SHA256, (const unsigned char *) hash, 0, sig, sig_len);
#endif
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to verify the signed data");
        esp_ret = ESP_FAIL;
        goto exit;
    }
    esp_ret = ESP_OK;
exit:
    free(sig);
    mbedtls_pk_free(&pk);
#if (MBEDTLS_VERSION_NUMBER < 0x04000000)
    mbedtls_entropy_free(&entropy);
    mbedtls_ctr_drbg_free(&ctr_drbg);
#endif // MBEDTLS_VERSION_NUMBER < 0x04000000
    mbedtls_x509_crt_free(&crt);
    return esp_ret;
}
#endif

static esp_err_t test_read_existing_data(void)
{
    ESP_LOGI(TAG, "\n==================== ESP SECURE CERT READ DEMO ====================");

    uint32_t len = 0;
    char *addr = NULL;
    esp_err_t esp_ret = ESP_FAIL;

#if CONFIG_ESP_SECURE_CERT_SECURE_VERIFICATION
    // Perform signature verification at startup
    ESP_LOGI(TAG, "Starting esp_secure_cert partition signature verification...");
    esp_err_t sig_ret = esp_secure_cert_verify_partition_signature(NULL);
    if (sig_ret == ESP_OK) {
        ESP_LOGI(TAG, "esp_secure_cert partition signature verification PASSED");
    } else {
        ESP_LOGE(TAG, "esp_secure_cert partition signature verification FAILED");
    }
#endif

    esp_ret = esp_secure_cert_get_device_cert(&addr, &len);
    if (esp_ret == ESP_OK) {
        esp_print_cert_or_key("Device Cert", (const char *)addr, len);
    } else {
        ESP_LOGE(TAG, "Failed to obtain flash address of device cert");
    }

    esp_ret = esp_secure_cert_get_ca_cert(&addr, &len);
    if (esp_ret == ESP_OK) {
        esp_print_cert_or_key("CA Cert", (const char *)addr, len);
    } else {
        ESP_LOGE(TAG, "Failed to obtain flash address of ca_cert");
    }

#ifndef CONFIG_ESP_SECURE_CERT_DS_PERIPHERAL
    esp_ret = esp_secure_cert_get_priv_key(&addr, &len);
    if (esp_ret == ESP_OK) {
        esp_print_cert_or_key("Private Key", (const char *)addr, len);
    } else {
        ESP_LOGE(TAG, "Failed to obtain flash address of private_key");
    }
    uint32_t dev_cert_len = 0;
    char *dev_cert_addr = NULL;
    esp_ret = esp_secure_cert_get_device_cert(&dev_cert_addr, &dev_cert_len);
    if (esp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to obtain the dev cert flash address");
    }

    esp_ret = test_priv_key_validity((unsigned char *)addr, len, (unsigned char *)dev_cert_addr, dev_cert_len);
    if (esp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to validate the private key and device certificate");
    }
#else
    esp_ds_data_ctx_t *ds_data = NULL;
    ds_data = esp_secure_cert_get_ds_ctx();
    if (ds_data != NULL) {
        ESP_LOGI(TAG, "Successfully obtained the ds context");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, ds_data->esp_ds_data->c, ESP_DS_C_LEN, ESP_LOG_DEBUG);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, ds_data->esp_ds_data->iv, ESP_DS_IV_LEN, ESP_LOG_DEBUG);
        ESP_LOGI(TAG, "The value of rsa length is %d", ds_data->rsa_length_bits);
        ESP_LOGI(TAG, "The value of efuse key id is %d", ds_data->efuse_key_id);
    } else {
        ESP_LOGE(TAG, "Failed to obtain the ds context");
    }

    /* Read the dev_cert addr again */
    esp_ret = esp_secure_cert_get_device_cert(&addr, &len);
    if (esp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to obtain the dev cert flash address");
    }

    esp_ret = test_ciphertext_validity(ds_data, (unsigned char *)addr, len);
    if (esp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to validate ciphertext");
    } else {
        ESP_LOGI(TAG, "Ciphertext validated succcessfully");
    }
#endif
    if (esp_ret == ESP_OK) {
        ESP_LOGI(TAG, "Successfully obtained and verified the contents of esp_secure_cert partition");
    } else {
        ESP_LOGE(TAG, "Failed to obtain and verify the contents of the esp_secure_cert partition");
    }
#ifndef CONFIG_ESP_SECURE_CERT_SUPPORT_LEGACY_FORMATS
    esp_secure_cert_tlv_config_t tlv_config = {};
    tlv_config.type = ESP_SECURE_CERT_DEV_CERT_TLV;
    tlv_config.subtype = ESP_SECURE_CERT_SUBTYPE_0;
    esp_secure_cert_tlv_info_t tlv_info = {};
    esp_ret = esp_secure_cert_get_tlv_info(&tlv_config, &tlv_info);
    if (esp_ret == ESP_OK) {
        esp_print_cert_or_key("Device Cert", (const char *)tlv_info.data, tlv_info.length);
    }

    ESP_LOGI(TAG, "Printing a list of TLV entries");
    esp_secure_cert_list_tlv_entries();
#endif

    ESP_LOGI(TAG, "==================== ESP SECURE CERT READ DEMO END ==================");
    return esp_ret;
}

#ifdef CONFIG_EXAMPLE_ESP_SECURE_CERT_WRITE_DEMO
/**
 * @brief Write demo with backup and restore
 *
 * Demonstrates real flash write/read cycle without destroying
 * pre-provisioned data:
 * 1. Back up original partition contents
 * 2. Erase, write demo data, read back and verify
 * 3. Restore original partition contents
 */
static esp_err_t run_write_demo(void)
{
    ESP_LOGI(TAG, "=== Write Demo: Basic TLV write/read ===");

    /* Find the partition to get its size for backup */
    const esp_partition_t *part = esp_partition_find_first(
        ESP_SECURE_CERT_TLV_PARTITION_TYPE, ESP_PARTITION_SUBTYPE_ANY,
        ESP_SECURE_CERT_TLV_PARTITION_NAME);
    if (part == NULL) {
        ESP_LOGE(TAG, "esp_secure_cert partition not found");
        return ESP_ERR_NOT_FOUND;
    }

    /* Step 1: Back up original partition contents */
    ESP_LOGI(TAG, "Backing up partition (%"PRIu32" bytes)...", part->size);
    uint8_t *backup = malloc(part->size);
    if (backup == NULL) {
        ESP_LOGE(TAG, "Failed to allocate backup buffer");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_partition_read(part, 0, backup, part->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read partition for backup: %s", esp_err_to_name(err));
        free(backup);
        return err;
    }
    ESP_LOGI(TAG, "Partition backed up");

    /* Unmap before we modify flash */
    esp_secure_cert_unmap_partition();

    /* Step 2: Erase, write demo data, read back */
    err = esp_secure_cert_erase_partition();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase partition: %s", esp_err_to_name(err));
        goto restore;
    }
    ESP_LOGI(TAG, "Partition erased");

    const char *sample_data = "Hello from esp_secure_cert write demo!";
    esp_secure_cert_tlv_info_t tlv_info = {
        .type = ESP_SECURE_CERT_USER_DATA_1,
        .subtype = ESP_SECURE_CERT_SUBTYPE_0,
        .data = (char *)sample_data,
        .length = strlen(sample_data) + 1,
        .flags = 0
    };

    err = esp_secure_cert_append_tlv(&tlv_info, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write TLV: %s", esp_err_to_name(err));
        goto restore;
    }
    ESP_LOGI(TAG, "Wrote user data TLV");

    /* Read back and verify */
    esp_secure_cert_tlv_config_t read_cfg = {
        .type = ESP_SECURE_CERT_USER_DATA_1,
        .subtype = ESP_SECURE_CERT_SUBTYPE_0
    };
    esp_secure_cert_tlv_info_t read_info = {};

    err = esp_secure_cert_get_tlv_info(&read_cfg, &read_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read back TLV: %s", esp_err_to_name(err));
        goto restore;
    }

    ESP_LOGI(TAG, "Read back: \"%s\" (len=%"PRIu32")", read_info.data, read_info.length);
    esp_secure_cert_free_tlv_info(&read_info);

    ESP_LOGI(TAG, "Write demo verified successfully");

restore:
    /* Step 3: Restore original partition contents */
    ESP_LOGI(TAG, "Restoring original partition contents...");
    esp_secure_cert_unmap_partition();

    esp_err_t restore_err = esp_partition_erase_range(part, 0, part->size);
    if (restore_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase for restore: %s", esp_err_to_name(restore_err));
        free(backup);
        return (err != ESP_OK) ? err : restore_err;
    }

    restore_err = esp_partition_write(part, 0, backup, part->size);
    free(backup);
    if (restore_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write backup data: %s", esp_err_to_name(restore_err));
        return (err != ESP_OK) ? err : restore_err;
    }

    /* Unmap so next read gets the restored data */
    esp_secure_cert_unmap_partition();
    ESP_LOGI(TAG, "Original partition contents restored");

    return err;
}
#endif /* CONFIG_EXAMPLE_ESP_SECURE_CERT_WRITE_DEMO */

void app_main()
{
    ESP_LOGI(TAG, "Starting ESP Secure Cert App");

    esp_err_t result = ESP_OK;

    // Read and display existing partition data first (before any writes)
    esp_err_t read_result = test_read_existing_data();
    if (read_result != ESP_OK) {
        result = read_result;
    }

#ifdef CONFIG_EXAMPLE_ESP_SECURE_CERT_WRITE_DEMO
    // Run write demo after reading - temporarily erases partition, then restores
    ESP_LOGI(TAG, "Running write demo (partition will be backed up and restored)...");
    esp_err_t write_result = run_write_demo();
    if (write_result != ESP_OK) {
        ESP_LOGE(TAG, "Write demo failed: %s", esp_err_to_name(write_result));
        if (result == ESP_OK) {
            result = write_result;
        }
    }
#endif

    ESP_LOGI(TAG, "=====================================================");
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "ESP Secure Cert App completed successfully!");
    } else {
        ESP_LOGE(TAG, "ESP Secure Cert App completed with errors: %s", esp_err_to_name(result));
    }
    ESP_LOGI(TAG, "=====================================================");
}
