/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_err.h"
#include "esp_secure_cert_tlv_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct esp_sign_verify_ctx {
    /* No context needed for now, This is for future use (kept for compatibility reasons)*/
} esp_sign_verify_ctx_t;

/**
 * @brief Verify the signature of esp_secure_cert partition
 *
 * This function:
 * 1. Reads the entire esp_secure_cert partition
 * 2. Extracts the signature block (last TLV entry with type ESP_SECURE_CERT_SIGNATURE_BLOCK_TLV)
 * 3. Calculates hash of all TLV entries except the signature block
 * 4. Verifies the signature using the public key from the signature block
 *
 * NOTE: Call this function before parsing esp_secure_cert partition.
 *
 * @return ESP_OK if signature verification passes, ESP_FAIL otherwise
 */
esp_err_t esp_secure_cert_verify_partition_signature(esp_sign_verify_ctx_t *ctx);

#ifdef __cplusplus
}
#endif
