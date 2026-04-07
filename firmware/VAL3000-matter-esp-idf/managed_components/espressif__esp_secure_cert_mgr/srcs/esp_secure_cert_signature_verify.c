/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_partition.h"
#include "esp_crc.h"
#include "esp_secure_cert_read.h"
#include "esp_secure_cert_tlv_config.h"
#include "esp_secure_cert_tlv_private.h"
#include "esp_secure_cert_signature_verify.h"
#include "esp_secure_cert_tlv_read.h"
#include "esp_secure_boot.h"
#include "soc/soc_caps.h"

/* Include ROM headers for ECDSA curve constants */
#if SOC_ECDSA_SUPPORTED
#include "rom/ecdsa.h"
#endif

#if (MBEDTLS_MAJOR_VERSION < 4)
#include "mbedtls/sha256.h"
#if defined(MBEDTLS_SHA384_C)
#include "mbedtls/sha512.h"
#endif
#else
#include "psa/crypto.h"
#endif

static const char *TAG = "esp_secure_cert_sig_verify";

#define SECURE_VERIFICATION_VERSION_1   0x01

#define SHA256_ALGORITHM                 0

/* ECDSA key and signature sizes */
#define ECDSA192_KEY_SIZE                48
#define ECDSA192_SIG_SIZE                48
#define ECDSA256_KEY_SIZE                64
#define ECDSA256_SIG_SIZE                64
#define ECDSA384_KEY_SIZE                96
#define ECDSA384_SIG_SIZE                96

/* ECDSA signature block size - use sizeof for type safety */
#define ECDSA_SIG_BLOCK_SIZE             sizeof(ets_secure_boot_sig_block_t)

/* ECDSA signature block version */
#define ECDSA_SIG_BLOCK_VERSION          0x03

#define ECDSA_SHA_VERSION_384            1

#define SHA384_DIGEST_SIZE               48  /* For P-384 curve */

typedef struct {
    uint8_t version;
    uint8_t reserved1[3];
    uint32_t sign_data_offset;
    uint32_t sign_data_length;
    uint8_t algorithm;
    uint8_t reserved2[3];
#if CONFIG_SECURE_SIGNED_APPS_RSA_SCHEME
    ets_secure_boot_sig_block_t rsa_signature_block;
#elif CONFIG_SECURE_SIGNED_APPS_ECDSA_V2_SCHEME
    struct {
        uint8_t curve_id;
#if CONFIG_SECURE_BOOT_ECDSA_KEY_LEN_192_BITS
        uint8_t pubkey_point[ECDSA192_KEY_SIZE];
        uint8_t signature[ECDSA192_SIG_SIZE];
#elif CONFIG_SECURE_BOOT_ECDSA_KEY_LEN_256_BITS
        uint8_t pubkey_point[ECDSA256_KEY_SIZE];
        uint8_t signature[ECDSA256_SIG_SIZE];
#elif CONFIG_SECURE_BOOT_ECDSA_KEY_LEN_384_BITS
        uint8_t pubkey_point[ECDSA384_KEY_SIZE];
        uint8_t signature[ECDSA384_SIG_SIZE];
#endif
    } ecdsa;
#endif
} __attribute__((packed)) esp_secure_cert_signature_t;

static esp_err_t calculate_partition_hash(const void *partition_addr, size_t partition_size,
                                          uint8_t *hash_out, size_t *hash_len_out, uint8_t curve_id)
{
    char *sig_block_data = NULL;
    uint32_t sig_block_data_len = 0;

    esp_err_t ret = esp_secure_cert_tlv_get_addr(ESP_SECURE_CERT_SIGNATURE_BLOCK_TLV,
                                                  ESP_SECURE_CERT_SUBTYPE_0,
                                                  &sig_block_data, &sig_block_data_len);
    if (ret != ESP_OK || sig_block_data == NULL) {
        ESP_LOGE(TAG, "Failed to get signature block");
        return ESP_FAIL;
    }

    const uint8_t *sig_block_header = (const uint8_t *)sig_block_data - sizeof(esp_secure_cert_tlv_header_t);
    size_t hash_size = sig_block_header - (const uint8_t *)partition_addr;

    bool use_sha384 = false;
#if CONFIG_SECURE_SIGNED_APPS_ECDSA_V2_SCHEME && CONFIG_SECURE_BOOT_ECDSA_KEY_LEN_384_BITS
    /* Determine hash algorithm based on curve */
    use_sha384 = (curve_id == ECDSA_CURVE_P384);
#endif
    (void)curve_id;

#if (MBEDTLS_MAJOR_VERSION < 4)
    if (use_sha384) {
#if defined(MBEDTLS_SHA384_C)
        mbedtls_sha512_context ctx;
        mbedtls_sha512_init(&ctx);

        if (mbedtls_sha512_starts(&ctx, 1) != 0 ||  /* 1 = SHA-384 */
            mbedtls_sha512_update(&ctx, (const uint8_t *)partition_addr, hash_size) != 0 ||
            mbedtls_sha512_finish(&ctx, hash_out) != 0) {
            mbedtls_sha512_free(&ctx);
            ESP_LOGE(TAG, "SHA384 calculation failed");
            return ESP_FAIL;
        }
        mbedtls_sha512_free(&ctx);
        *hash_len_out = SHA384_DIGEST_SIZE;
#else
        ESP_LOGE(TAG, "SHA384 not supported in mbedTLS");
        return ESP_ERR_NOT_SUPPORTED;
#endif
    } else {
        mbedtls_sha256_context ctx;
        mbedtls_sha256_init(&ctx);

        if (mbedtls_sha256_starts(&ctx, SHA256_ALGORITHM) != 0 ||
            mbedtls_sha256_update(&ctx, (const uint8_t *)partition_addr, hash_size) != 0 ||
            mbedtls_sha256_finish(&ctx, hash_out) != 0) {
            mbedtls_sha256_free(&ctx);
            ESP_LOGE(TAG, "SHA256 calculation failed");
            return ESP_FAIL;
        }
        mbedtls_sha256_free(&ctx);
        *hash_len_out = ESP_SECURE_BOOT_DIGEST_LEN;
    }
#else
    if (use_sha384) {
        size_t hash_length = 0;
        psa_status_t status = psa_hash_compute(PSA_ALG_SHA_384,
                                              (const uint8_t *)partition_addr,
                                              hash_size,
                                              hash_out, SHA384_DIGEST_SIZE, &hash_length);
        if (status != PSA_SUCCESS || hash_length != SHA384_DIGEST_SIZE) {
            ESP_LOGE(TAG, "SHA384 calculation failed");
            return ESP_FAIL;
        }
        *hash_len_out = SHA384_DIGEST_SIZE;
    } else {
        size_t hash_length = 0;
        psa_status_t status = psa_hash_compute(PSA_ALG_SHA_256,
                                              (const uint8_t *)partition_addr,
                                              hash_size,
                                              hash_out, ESP_SECURE_BOOT_DIGEST_LEN, &hash_length);
        if (status != PSA_SUCCESS || hash_length != ESP_SECURE_BOOT_DIGEST_LEN) {
            ESP_LOGE(TAG, "SHA256 calculation failed");
            return ESP_FAIL;
        }
        *hash_len_out = ESP_SECURE_BOOT_DIGEST_LEN;
    }
#endif

    return ESP_OK;
}

#if CONFIG_SECURE_SIGNED_APPS_ECDSA_V2_SCHEME
static inline void get_ecdsa_params(uint8_t curve_id, size_t *pubkey_len, size_t *sig_len, size_t *hash_len)
{
    switch (curve_id) {
    case ECDSA_CURVE_P192:
        *pubkey_len = ECDSA192_KEY_SIZE;
        *sig_len = ECDSA192_SIG_SIZE;
        if (hash_len) *hash_len = ESP_SECURE_BOOT_DIGEST_LEN;
        break;
    case ECDSA_CURVE_P256:
        *pubkey_len = ECDSA256_KEY_SIZE;
        *sig_len = ECDSA256_SIG_SIZE;
        if (hash_len) *hash_len = ESP_SECURE_BOOT_DIGEST_LEN;
        break;
#if CONFIG_SECURE_BOOT_ECDSA_KEY_LEN_384_BITS
    case ECDSA_CURVE_P384:
        *pubkey_len = ECDSA384_KEY_SIZE;
        *sig_len = ECDSA384_SIG_SIZE;
        if (hash_len) *hash_len = SHA384_DIGEST_SIZE;
        break;
#endif
    default:
        *pubkey_len = 0;
        *sig_len = 0;
        if (hash_len) *hash_len = 0;
        break;
    }
}

static esp_err_t construct_ecdsa_sig_block(const uint8_t *hash, size_t hash_len,
                                           uint8_t curve_id, const uint8_t *pubkey,
                                           size_t pubkey_len, const uint8_t *signature,
                                           size_t sig_len, ets_secure_boot_sig_block_t *block)
{
    if (!hash || !pubkey || !signature || !block) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(block, 0, ECDSA_SIG_BLOCK_SIZE);
    uint8_t *block_bytes = (uint8_t *)block;

    block->magic_byte = ETS_SECURE_BOOT_V2_SIGNATURE_MAGIC;
    block->version = ECDSA_SIG_BLOCK_VERSION;
    block->ecdsa.key.curve_id = curve_id;

#if CONFIG_SECURE_BOOT_ECDSA_KEY_LEN_384_BITS
    if (curve_id == ECDSA_CURVE_P384) {
        block_bytes[2] = ECDSA_SHA_VERSION_384;
    }
#endif

    memcpy(block->image_digest, hash, hash_len);
    memcpy(block->ecdsa.key.point, pubkey, pubkey_len);
    memcpy(block->ecdsa.signature, signature, sig_len);

    uint32_t crc = esp_crc32_le(0, block_bytes, offsetof(ets_secure_boot_sig_block_t, block_crc));
    block->block_crc = crc;

    return ESP_OK;
}
#endif

static esp_err_t get_signature_blocks(const uint8_t *partition_hash,
                                     size_t partition_hash_len,
                                     ets_secure_boot_signature_t *sig_blocks)
{
    int found_count = 0;
    esp_err_t ret = ESP_FAIL;

    memset(sig_blocks, 0, sizeof(ets_secure_boot_signature_t));

    for (int subtype = 0; subtype < SECURE_BOOT_NUM_BLOCKS; subtype++) {
        char *sig_data = NULL;
        uint32_t sig_len = 0;

        ret = esp_secure_cert_tlv_get_addr(ESP_SECURE_CERT_SIGNATURE_BLOCK_TLV,
                                           subtype, &sig_data, &sig_len);
        if (ret != ESP_OK || !sig_data || sig_len < sizeof(esp_secure_cert_signature_t)) {
            continue;
        }

        esp_secure_cert_signature_t *esp_sig = (esp_secure_cert_signature_t *)sig_data;
        if (esp_sig->version != SECURE_VERIFICATION_VERSION_1) {
            ESP_LOGE(TAG, "Unsupported signature version: %d", esp_sig->version);
            return ESP_FAIL;
        }

#if CONFIG_SECURE_SIGNED_APPS_RSA_SCHEME
        memcpy(&sig_blocks->block[subtype], &esp_sig->rsa_signature_block,
               sizeof(ets_secure_boot_sig_block_t));
#else /* CONFIG_SECURE_SIGNED_APPS_ECDSA_V2_SCHEME */
        size_t pubkey_len, sig_size, hash_len;
        get_ecdsa_params(esp_sig->ecdsa.curve_id, &pubkey_len, &sig_size, &hash_len);

        if (partition_hash_len != hash_len) {
            ESP_LOGE(TAG, "Hash length mismatch: got %zu, expected %zu", partition_hash_len, hash_len);
            return ESP_FAIL;
        }

        ret = construct_ecdsa_sig_block(partition_hash, hash_len, esp_sig->ecdsa.curve_id,
                                        esp_sig->ecdsa.pubkey_point, pubkey_len,
                                        esp_sig->ecdsa.signature, sig_size,
                                        &sig_blocks->block[subtype]);
        if (ret != ESP_OK) {
            return ret;
        }
#endif
        found_count++;
    }

    if (found_count == 0) {
        ESP_LOGW(TAG, "No signature blocks found");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t verify_signature(const ets_secure_boot_signature_t *sig_blocks,
                                  const uint8_t *hash, size_t hash_len)
{
    void *verify_buf = calloc(SECURE_BOOT_NUM_BLOCKS, hash_len);
    if (!verify_buf) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return ESP_FAIL;
    }

    esp_err_t ret = esp_secure_boot_verify_sbv2_signature_block(sig_blocks, hash, verify_buf);
    free(verify_buf);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Signature verification failed");
    }
    return ret;
}

esp_err_t esp_secure_cert_verify_partition_signature(esp_sign_verify_ctx_t *sign_verify_ctx)
{
    (void)sign_verify_ctx; /* Not used at the moment */
    esp_secure_cert_partition_ctx_t *ctx = NULL;
    uint8_t partition_hash[SHA384_DIGEST_SIZE];
    size_t partition_hash_len = 0;
    uint8_t curve_id = 0;

    esp_err_t ret = esp_secure_cert_map_partition(&ctx);
    if (ret != ESP_OK || !ctx->esp_secure_cert_mapped_addr) {
        ESP_LOGE(TAG, "Failed to map partition");
        return ESP_FAIL;
    }

    char *sig_data = NULL;
    uint32_t sig_len = 0;
    ret = esp_secure_cert_tlv_get_addr(ESP_SECURE_CERT_SIGNATURE_BLOCK_TLV,
                                       ESP_SECURE_CERT_SUBTYPE_0, &sig_data, &sig_len);
    if (ret != ESP_OK || !sig_data || sig_len < sizeof(esp_secure_cert_signature_t)) {
        ESP_LOGE(TAG, "Failed to get signature block");
        return ESP_FAIL;
    }

#if CONFIG_SECURE_SIGNED_APPS_ECDSA_V2_SCHEME
    curve_id = ((esp_secure_cert_signature_t *)sig_data)->ecdsa.curve_id;
#endif

    ret = calculate_partition_hash(ctx->esp_secure_cert_mapped_addr, ctx->partition->size,
                                   partition_hash, &partition_hash_len, curve_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calculate partition hash");
        return ESP_FAIL;
    }

    ets_secure_boot_signature_t *sig_blocks = calloc(1, sizeof(ets_secure_boot_signature_t));
    if (!sig_blocks) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return ESP_FAIL;
    }

    ret = get_signature_blocks(partition_hash, partition_hash_len, sig_blocks);
    if (ret == ESP_OK) {
        ret = verify_signature(sig_blocks, partition_hash, partition_hash_len);
    } else {
        ESP_LOGE(TAG, "Failed to get signature blocks");
    }

    free(sig_blocks);
    return ret;
}
