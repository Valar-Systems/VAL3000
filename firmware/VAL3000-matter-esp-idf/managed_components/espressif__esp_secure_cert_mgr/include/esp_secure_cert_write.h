/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <string.h>
#include <stdbool.h>
#include "esp_secure_cert_read.h"
#include "esp_secure_cert_tlv_config.h"
#include "esp_secure_cert_tlv_read.h"
#include "esp_secure_cert_write_errors.h"
#include "soc/soc_caps.h"

#ifndef ESP_SECURE_CERT_WRITE_SUPPORT
#warning "esp_secure_cert write APIs require ESP-IDF >= 5.3. Write functions are not available in this build."
#else /* ESP_SECURE_CERT_WRITE_SUPPORT */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Write operation modes for TLV operations
 */
typedef enum {
    ESP_SECURE_CERT_WRITE_MODE_FLASH = 0,    /**< Write directly to flash partition */
    ESP_SECURE_CERT_WRITE_MODE_BUFFER = 1,   /**< Write to memory buffer (for host generation) */
} esp_secure_cert_write_mode_t;

/**
 * @brief Configuration structure for TLV write operations
 *
 * This structure provides flexibility for future extensions without
 * breaking API compatibility.
 */
typedef struct {
    esp_secure_cert_write_mode_t mode;       /**< Write operation mode */
    union {
        struct {
            bool check_erase;                /**< Check if flash area is erased before writing */
            bool auto_erase;                 /**< Automatically erase if needed (use with caution) */
        } flash;                             /**< Flash-specific options */
        struct {
            uint8_t *buffer;                 /**< Target memory buffer */
            size_t buffer_size;              /**< Size of the buffer */
            size_t *bytes_written;           /**< Pointer to store actual bytes written */
        } buffer;                            /**< Buffer-specific options */
    };
    uint32_t reserved[4];                    /**< Reserved for future use, must be zero */
} esp_secure_cert_write_config_t;

/**
 * @brief Initialize write configuration with default values
 *
 * @param[out] config Configuration structure to initialize
 * @param[in] mode Write operation mode
 */
static inline void esp_secure_cert_write_config_init(esp_secure_cert_write_config_t *config,
                                                     esp_secure_cert_write_mode_t mode)
{
    if (config) {
        memset(config, 0, sizeof(esp_secure_cert_write_config_t));
        config->mode = mode;
        if (mode == ESP_SECURE_CERT_WRITE_MODE_FLASH) {
            config->flash.check_erase = true;
            config->flash.auto_erase = false;
        }
    }
}

/**
 * @brief Erase the esp_secure_cert partition
 *
 * This function completely erases the esp_secure_cert partition, removing all
 * existing TLV entries. Use with caution as this operation is irreversible.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_SECURE_CERT_PARTITION_NOT_FOUND if partition is not found
 *      - ESP_ERR_SECURE_CERT_ERASE_FAILED if erase operation failed
 */
esp_err_t esp_secure_cert_erase_partition(void);

/**
 * @brief Check if a flash region is erased (all bytes are 0xFF)
 *
 * @param[in] offset Offset from partition start
 * @param[in] size Size to check in bytes
 * @param[out] is_erased Pointer to store result (true if erased)
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if parameters are invalid
 *      - ESP_ERR_SECURE_CERT_PARTITION_NOT_FOUND if partition is not found
 *      - ESP_ERR_INVALID_SIZE if size is zero
 *      - ESP_ERR_SECURE_CERT_FLASH_READ_FAILED if read operation failed
 *      - ESP_ERR_SECURE_CERT_ERASE_CHECK_NO_MEMORY if memory allocation failed
 */
esp_err_t esp_secure_cert_check_flash_erased(size_t offset, size_t size, bool *is_erased);

/**
 * @brief Write a TLV entry to the esp_secure_cert partition or memory buffer
 *
 * This unified function handles both flash and memory buffer operations based on
 * the write configuration. It supports erase checking, buffer operations for host
 * generation, and maintains backward compatibility.
 *
 * When write_config is NULL, the function uses default flash mode:
 * - Writes directly to flash partition
 * - Checks if TLV of the given type and subtype already exists
 * - Performs erase checking (verifies flash is erased before writing)
 *
 * @note Flash encryption compatibility: When flash encryption is enabled,
 *       ensure that data alignment requirements are met.
 *
 * @param[in] tlv_info The TLV information structure for the entry
 * @param[in] write_config Write operation configuration (NULL for legacy flash mode)
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG when invalid arguments are provided
 *      - ESP_ERR_SECURE_CERT_TLV_ALREADY_EXISTS when TLV already exists (flash mode)
 *      - ESP_ERR_SECURE_CERT_WRITE_NO_MEMORY when memory allocation fails
 *      - ESP_ERR_SECURE_CERT_PARTITION_NOT_FOUND when partition not found (flash mode)
 *      - ESP_ERR_SECURE_CERT_FLASH_NOT_ERASED when flash area not erased (if check enabled)
 *      - Other specific ESP_ERR_SECURE_CERT_* error codes on failure
 */
esp_err_t esp_secure_cert_append_tlv(esp_secure_cert_tlv_info_t *tlv_info,
                                     const esp_secure_cert_write_config_t *write_config);

/**
 * @brief Write multiple TLV entries efficiently
 *
 * This function allows writing multiple TLV entries in a single operation,
 * optimizing for both flash writes and buffer operations.
 *
 * @param[in] tlv_entries Array of TLV info structures
 * @param[in] num_entries Number of entries in the array
 * @param[in] write_config Write operation configuration (NULL for legacy flash mode)
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG when invalid arguments are provided
 *      - ESP_ERR_SECURE_CERT_TLV_ALREADY_EXISTS when any TLV already exists (flash mode)
 *      - ESP_ERR_SECURE_CERT_WRITE_NO_MEMORY when memory allocation fails
 *      - Other specific ESP_ERR_SECURE_CERT_* error codes on failure
 */
esp_err_t esp_secure_cert_append_tlv_batch(esp_secure_cert_tlv_info_t *tlv_entries,
                                           size_t num_entries,
                                           const esp_secure_cert_write_config_t *write_config);

#if SOC_HMAC_SUPPORTED
/**
 * @brief Write a TLV entry with HMAC-based encryption
 *
 * This function encrypts the TLV data using HMAC-based AES-GCM encryption
 * before writing using the specified configuration. It requires an HMAC key
 * to be burned in an eFuse key block with purpose set to HMAC_UP.
 *
 * @param[in] tlv_info The TLV information structure for the entry
 * @param[in] write_config Write operation configuration (NULL for legacy flash mode)
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG when invalid arguments are provided
 *      - ESP_ERR_SECURE_CERT_HMAC_KEY_NOT_FOUND when HMAC_UP key block not found
 *      - ESP_ERR_SECURE_CERT_HMAC_ENCRYPTION_FAILED when encryption fails
 *      - ESP_ERR_SECURE_CERT_WRITE_NO_MEMORY when memory allocation fails
 *      - Other specific ESP_ERR_SECURE_CERT_* error codes on failure
 */
esp_err_t esp_secure_cert_append_tlv_with_hmac_encryption(esp_secure_cert_tlv_info_t *tlv_info,
                                                          const esp_secure_cert_write_config_t *write_config);

/**
 * @brief Write TLV entries for HMAC-based ECDSA key derivation
 *
 * This function sets up the partition for HMAC-based ECDSA private key derivation.
 * Instead of storing the actual private key, it stores:
 * 1. A salt value (ESP_SECURE_CERT_HMAC_ECDSA_KEY_SALT TLV)
 * 2. A marker TLV for private key with derivation flag set
 *
 * At read time, the private key is derived using PBKDF2-HMAC-SHA256 with:
 * - HMAC key from eFuse (with HMAC_UP purpose)
 * - Salt from partition
 * - 2048 iterations
 * - Output: 32-byte raw ECDSA private key (converted to DER format)
 *
 * This approach provides:
 * - Private key never stored in flash (derived on-demand)
 * - Deterministic key generation from salt + HMAC key
 * - Key can be regenerated if salt is preserved
 * - Requires hardware HMAC peripheral
 *
 * @note Requires HMAC key burned in eFuse with ESP_EFUSE_KEY_PURPOSE_HMAC_UP
 *
 * @param[in] salt Salt value for key derivation (typically 32 bytes for P-256, can be random)
 * @param[in] salt_len Length of salt in bytes
 * @param[in] subtype Subtype for the TLV entries (index, allows multiple keys)
 * @param[in] write_config Write operation configuration (NULL for legacy flash mode)
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG when invalid arguments are provided
 *      - ESP_ERR_SECURE_CERT_HMAC_KEY_NOT_FOUND when HMAC_UP key block not found in eFuse
 *      - ESP_ERR_SECURE_CERT_TLV_ALREADY_EXISTS when TLV already exists (flash mode)
 *      - Other specific ESP_ERR_SECURE_CERT_* error codes on failure
 *
 * Example usage:
 * @code
 * // Generate random salt
 * uint8_t salt[32];
 * esp_fill_random(salt, sizeof(salt));
 *
 * // Write HMAC-ECDSA derivation configuration
 * esp_err_t err = esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(
 *     salt, sizeof(salt),
 *     ESP_SECURE_CERT_SUBTYPE_0,
 *     NULL  // Use default flash write mode
 * );
 *
 * // Later, when reading the private key:
 * // esp_secure_cert_get_priv_key() will automatically derive the key
 * // using PBKDF2-HMAC-SHA256(salt, hmac_key, 2048 iterations)
 * @endcode
 */
esp_err_t esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(const uint8_t *salt, size_t salt_len,
                                                                 esp_secure_cert_tlv_subtype_t subtype,
                                                                 const esp_secure_cert_write_config_t *write_config);

/**
 * @brief Generate and configure HMAC-based ECDSA key derivation
 *
 * This function performs the complete HMAC-based ECDSA key setup:
 * 1. Generates random salt (32 bytes)
 * 2. Generates random HMAC key (32 bytes)
 * 3. Derives ECDSA private key using PBKDF2-HMAC-SHA256 (2048 iterations)
 * 4. Validates the derived key is valid for SECP256R1 (1 < d < N)
 * 5. Burns HMAC key to eFuse with HMAC_UP purpose
 * 6. Verifies hardware HMAC can reproduce the same key
 * 7. Stores salt in esp_secure_cert partition
 *
 * After calling this function:
 * - The HMAC key is permanently stored in eFuse (cannot be changed)
 * - The salt is stored in the esp_secure_cert partition
 * - Reading the private key via esp_secure_cert_get_priv_key() will derive it on-demand
 *
 * @note This function will fail if an HMAC_UP key already exists in eFuse.
 *       Use esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation() if you want
 *       to configure derivation with an existing HMAC key.
 *
 * @param[out] pub_key_buf Optional buffer for uncompressed public key (65 bytes: 0x04 || X || Y)
 *                         Pass NULL if public key is not needed.
 * @param[in,out] pub_key_len Input: buffer size (must be >= 65), Output: actual size written.
 *                            Pass NULL if pub_key_buf is NULL.
 * @param[in] subtype Subtype for the TLV entries (allows multiple keys with different subtypes)
 * @param[in] write_config Write operation configuration (NULL for default flash mode)
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_SECURE_CERT_HMAC_KEY_ALREADY_EXISTS if HMAC_UP key already in eFuse
 *      - ESP_ERR_SECURE_CERT_ECDSA_KEY_GEN_FAILED if valid key generation failed after max attempts
 *      - ESP_ERR_SECURE_CERT_EFUSE_WRITE_FAILED if eFuse write failed
 *      - ESP_ERR_SECURE_CERT_KEY_VERIFICATION_FAILED if hardware verification failed
 *      - ESP_ERR_SECURE_CERT_WRITE_NO_MEMORY if memory allocation failed
 *      - Other ESP_ERR_SECURE_CERT_* error codes on failure
 *
 * Example usage:
 * @code
 * // Generate HMAC-ECDSA key and get public key
 * uint8_t pub_key[65];
 * size_t pub_key_len = sizeof(pub_key);
 *
 * esp_err_t err = esp_secure_cert_derive_hmac_ecdsa_key(
 *     pub_key, &pub_key_len,
 *     ESP_SECURE_CERT_SUBTYPE_0,
 *     NULL  // Use default flash write mode
 * );
 *
 * if (err == ESP_OK) {
 *     // Public key is available in pub_key buffer
 *     // Private key can be derived via esp_secure_cert_get_priv_key()
 * }
 * @endcode
 */
esp_err_t esp_secure_cert_derive_hmac_ecdsa_key(uint8_t *pub_key_buf, size_t *pub_key_len,
                                                  esp_secure_cert_tlv_subtype_t subtype,
                                                  const esp_secure_cert_write_config_t *write_config);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ESP_SECURE_CERT_WRITE_SUPPORT */
