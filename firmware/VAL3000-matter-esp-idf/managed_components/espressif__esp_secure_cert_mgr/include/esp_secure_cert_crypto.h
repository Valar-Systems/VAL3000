/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include "esp_err.h"
#include "soc/soc_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SOC_HMAC_SUPPORTED
#include "esp_hmac.h"
/**
 * @brief PBKDF2-HMAC-SHA256 using hardware HMAC peripheral
 *
 * Password-based key derivation function using hardware HMAC peripheral.
 * The HMAC key is read from eFuse and cannot be accessed by software.
 *
 * @param[in] hmac_key_id eFuse key block ID containing the HMAC key
 * @param[in] salt Salt buffer
 * @param[in] salt_len Salt length in bytes
 * @param[in] iteration_count Number of PBKDF2 iterations
 * @param[in] key_length Expected derived key length
 * @param[out] output Buffer for derived key (must be key_length bytes)
 *
 * @return 0 on success, -1 on failure
 */
int esp_pbkdf2_hmac_sha256(hmac_key_id_t hmac_key_id, const unsigned char *salt, size_t salt_len,
                           size_t iteration_count, size_t key_length, unsigned char *output);

/**
 * @brief Validate that a raw key is a valid ECDSA private key for SECP256R1
 *
 * A valid private key d must satisfy: 1 < d < N, where N is the curve order.
 *
 * @param[in] key_buf Raw key buffer (32 bytes for SECP256R1)
 * @param[in] key_len Length of the key buffer
 *
 * @return ESP_OK if valid, ESP_FAIL if invalid, ESP_ERR_INVALID_ARG for bad params
 */
esp_err_t esp_secure_cert_validate_ecdsa_key(const uint8_t *key_buf, size_t key_len);

/**
 * @brief Derive ECDSA key using software PBKDF2-HMAC-SHA256
 *
 * Uses mbedtls to perform PBKDF2 derivation in software. This is used during
 * key generation before the HMAC key is burned to eFuse.
 *
 * @param[in] hmac_key HMAC key buffer (32 bytes)
 * @param[in] hmac_key_len HMAC key length
 * @param[in] salt Salt buffer
 * @param[in] salt_len Salt length
 * @param[in] iterations Number of PBKDF2 iterations
 * @param[out] output Derived key buffer
 * @param[in] output_len Output buffer length (32 bytes for ECDSA)
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_secure_cert_sw_pbkdf2_hmac_sha256(const uint8_t *hmac_key, size_t hmac_key_len,
                                                  const uint8_t *salt, size_t salt_len,
                                                  uint32_t iterations,
                                                  uint8_t *output, size_t output_len);

/**
 * @brief Calculate public key from private key for SECP256R1
 *
 * @param[in] priv_key_buf Raw private key (32 bytes)
 * @param[in] priv_key_len Private key length
 * @param[out] pub_key_buf Buffer for uncompressed public key (65 bytes: 0x04 || X || Y)
 * @param[in,out] pub_key_len Input: buffer size, Output: actual size written
 *
 * @return ESP_OK on success, error code on failure
 */
esp_err_t esp_secure_cert_calc_public_key(const uint8_t *priv_key_buf, size_t priv_key_len,
                                           uint8_t *pub_key_buf, size_t *pub_key_len);
#endif /* SOC_HMAC_SUPPORTED */

#ifdef __cplusplus
}
#endif
