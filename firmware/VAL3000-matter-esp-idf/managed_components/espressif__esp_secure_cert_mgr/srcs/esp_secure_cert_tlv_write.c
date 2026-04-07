/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_partition.h"
#include "esp_crc.h"
#include "esp_secure_cert_tlv_config.h"
#include "esp_secure_cert_tlv_private.h"
#include "esp_secure_cert_tlv_read.h"
#include "esp_secure_cert_read.h"
#include "esp_secure_cert_write.h"
#include "esp_secure_cert_write_errors.h"
#include "soc/soc_caps.h"
#include "esp_heap_caps.h"
#include "mbedtls/pk.h"
#include "mbedtls/platform_util.h"

#if SOC_HMAC_SUPPORTED
#include "esp_efuse.h"
#include "esp_random.h"
#include "esp_secure_cert_crypto.h"
#if (MBEDTLS_MAJOR_VERSION < 4)
#include <mbedtls/gcm.h>
#else
#include "psa/crypto.h"
#endif /* (MBEDTLS_MAJOR_VERSION < 4) */
#endif /* SOC_HMAC_SUPPORTED */


static const char *TAG = "esp_secure_cert_write";

/* Conditional logging macros to reduce binary size.
 * Error logs are always enabled for production debuggability. */
#ifdef CONFIG_ESP_SECURE_CERT_WRITE_ENABLE_LOGGING
#define WRITE_LOGI ESP_LOGI
#define WRITE_LOGD ESP_LOGD
#define WRITE_LOGW ESP_LOGW
#else
#define WRITE_LOGI(tag, format, ...)
#define WRITE_LOGD(tag, format, ...)
#define WRITE_LOGW(tag, format, ...)
#endif
#define WRITE_LOGE ESP_LOGE



/* Forward declarations for internal functions */
static esp_err_t esp_secure_cert_write_to_flash(size_t offset, const unsigned char *data_buf, size_t data_len);
static esp_err_t esp_secure_cert_write_to_buffer(uint8_t *buffer, size_t buffer_size, size_t *offset,
                                                 const unsigned char *data_buf, size_t data_len);
static esp_err_t esp_secure_cert_prepare_tlv_compact(esp_secure_cert_tlv_type_t type, const unsigned char *value,
                                                     size_t value_len, unsigned char *output_buf, size_t *output_len,
                                                     uint8_t flags, uint8_t subtype);
static esp_err_t esp_secure_cert_get_next_write_offset(size_t *offset);
static esp_err_t esp_secure_cert_validate_tlv_info(const esp_secure_cert_tlv_info_t *tlv_info);
static const esp_partition_t *esp_secure_cert_get_partition_handle(void);

#if SOC_HMAC_SUPPORTED
static esp_err_t esp_secure_cert_encrypt_with_hmac(const esp_secure_cert_tlv_info_t *tlv_info,
                                                    unsigned char **encrypted_data, size_t *encrypted_len);
#endif

/*
 * Acquire write lock using atomic compare-and-swap.
 * Returns ESP_OK if lock acquired, ESP_ERR_SECURE_CERT_WRITE_IN_PROGRESS if already locked.
 * This is a non-blocking (fail-fast) implementation for efficiency.
 */
static esp_err_t esp_secure_cert_acquire_write_lock(void)
{
    esp_secure_cert_partition_ctx_t *ctx = NULL;
    if (esp_secure_cert_map_partition(&ctx) != ESP_OK || ctx == NULL) {
        WRITE_LOGE(TAG, "Failed to get partition context for lock");
        return ESP_ERR_SECURE_CERT_PARTITION_ACCESS_FAILED;
    }

    bool expected = false;
    if (atomic_compare_exchange_strong(&ctx->write_lock, &expected, true)) {
        WRITE_LOGD(TAG, "Write lock acquired");
        return ESP_OK;
    }

    WRITE_LOGW(TAG, "Write operation already in progress");
    return ESP_ERR_SECURE_CERT_WRITE_IN_PROGRESS;
}

/*
 * Release write lock.
 */
static void esp_secure_cert_release_write_lock(void)
{
    esp_secure_cert_partition_ctx_t *ctx = NULL;
    if (esp_secure_cert_map_partition(&ctx) == ESP_OK && ctx != NULL) {
        atomic_store(&ctx->write_lock, false);
        WRITE_LOGD(TAG, "Write lock released");
    }
}

/* Internal erase function - must be called with lock held */
static esp_err_t esp_secure_cert_erase_partition_internal(void)
{
    const esp_partition_t *part = esp_secure_cert_get_partition_handle();
    if (part == NULL) {
        WRITE_LOGE(TAG, "Partition not found");
        return ESP_ERR_SECURE_CERT_PARTITION_NOT_FOUND;
    }

    esp_err_t err = esp_partition_erase_range(part, 0, part->size);
    if (err != ESP_OK) {
        WRITE_LOGE(TAG, "Partition erase failed: %s", esp_err_to_name(err));
        return ESP_ERR_SECURE_CERT_ERASE_FAILED;
    }

    WRITE_LOGI(TAG, "Successfully erased esp_secure_cert partition");
    return ESP_OK;
}

esp_err_t esp_secure_cert_erase_partition(void)
{
    /* Acquire write lock */
    esp_err_t err = esp_secure_cert_acquire_write_lock();
    if (err != ESP_OK) {
        return err;
    }

    err = esp_secure_cert_erase_partition_internal();

    esp_secure_cert_release_write_lock();

    /* Unmap partition so subsequent operations get fresh data from flash.
     * This is critical because the mmap'd memory view becomes stale after
     * flash erase/write operations. */
    esp_secure_cert_unmap_partition();

    return err;
}

esp_err_t esp_secure_cert_check_flash_erased(size_t offset, size_t size, bool *is_erased)
{
    if (is_erased == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *part = esp_secure_cert_get_partition_handle();
    if (part == NULL) {
        return ESP_ERR_SECURE_CERT_PARTITION_NOT_FOUND;
    }

    if (size > part->size || offset > part->size - size) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* Use stack buffer for small reads to avoid heap allocation overhead.
     * Use aligned attribute to ensure safe uint32_t* cast for word comparison */
    uint8_t check_buf[64] __attribute__((aligned(4)));
    const size_t chunk_size = sizeof(check_buf);

    *is_erased = true;
    size_t remaining = size;
    size_t current_offset = offset;

    while (remaining > 0 && *is_erased) {
        size_t read_size = (remaining > chunk_size) ? chunk_size : remaining;

        if (esp_partition_read(part, current_offset, check_buf, read_size) != ESP_OK) {
            return ESP_ERR_SECURE_CERT_FLASH_READ_FAILED;
        }

        /* Use word-aligned comparison for better performance */
        uint32_t *word_ptr = (uint32_t *)check_buf;
        size_t word_count = read_size / sizeof(uint32_t);
        for (size_t i = 0; i < word_count; i++) {
            if (word_ptr[i] != 0xFFFFFFFF) {
                *is_erased = false;
                break;
            }
        }

        /* Check remaining bytes */
        if (*is_erased) {
            for (size_t i = word_count * sizeof(uint32_t); i < read_size; i++) {
                if (check_buf[i] != 0xFF) {
                    *is_erased = false;
                    break;
                }
            }
        }

        current_offset += read_size;
        remaining -= read_size;
    }

    return ESP_OK;
}

/* Get partition handle using shared context from read code */
static const esp_partition_t *esp_secure_cert_get_partition_handle(void)
{
    esp_secure_cert_partition_ctx_t *ctx = NULL;
    if (esp_secure_cert_map_partition(&ctx) != ESP_OK) {
        WRITE_LOGE(TAG, "Failed to get partition handle");
        return NULL;
    }
    return ctx->partition;
}

/* Write data to flash at particular offset with verification */
static esp_err_t esp_secure_cert_write_to_flash(size_t offset, const unsigned char *data_buf, size_t data_len)
{
    if (data_buf == NULL || data_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const esp_partition_t *part = esp_secure_cert_get_partition_handle();
    if (part == NULL) {
        return ESP_ERR_SECURE_CERT_PARTITION_NOT_FOUND;
    }

    /* Validate bounds (overflow-safe check) */
    if (data_len > part->size || offset > part->size - data_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* Write to flash */
    esp_err_t err = esp_partition_write(part, offset, data_buf, data_len);
    if (err != ESP_OK) {
        WRITE_LOGE(TAG, "Flash write failed: %s", esp_err_to_name(err));
        return ESP_ERR_SECURE_CERT_FLASH_WRITE_FAILED;
    }

    /* Verify write by reading back and comparing CRC
     * Use stack buffer for small chunks to avoid heap allocation */
    uint8_t verify_buf[64];
    const size_t chunk_size = sizeof(verify_buf);
    size_t remaining = data_len;
    size_t current_offset = 0;

    while (remaining > 0) {
        size_t read_size = (remaining > chunk_size) ? chunk_size : remaining;

        err = esp_partition_read(part, offset + current_offset, verify_buf, read_size);
        if (err != ESP_OK) {
            WRITE_LOGE(TAG, "Verify read failed");
            return ESP_ERR_SECURE_CERT_FLASH_READ_FAILED;
        }

        if (memcmp(verify_buf, data_buf + current_offset, read_size) != 0) {
            WRITE_LOGE(TAG, "Write verification failed at offset %zu", offset + current_offset);
            return ESP_ERR_SECURE_CERT_FLASH_WRITE_FAILED;
        }

        current_offset += read_size;
        remaining -= read_size;
    }

    WRITE_LOGD(TAG, "Wrote and verified %zu bytes at offset %zu", data_len, offset);

    /* Unmap partition after successful write to invalidate stale mmap cache.
     * Next access will automatically remap and get fresh data from flash. */
    esp_secure_cert_unmap_partition();

    return ESP_OK;
}

/* Write data to memory buffer */
static esp_err_t esp_secure_cert_write_to_buffer(uint8_t *buffer, size_t buffer_size, size_t *offset,
                                                 const unsigned char *data_buf, size_t data_len)
{
    if (buffer == NULL || offset == NULL || data_buf == NULL || data_len == 0) {
        WRITE_LOGE(TAG, "Invalid buffer write parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (data_len > buffer_size || *offset > buffer_size - data_len) {
        WRITE_LOGE(TAG, "Buffer write overflow: offset=%zu, len=%zu, buf_size=%zu",
                 *offset, data_len, buffer_size);
        return ESP_ERR_SECURE_CERT_BUFFER_OVERFLOW;
    }

    memcpy(buffer + *offset, data_buf, data_len);
    *offset += data_len;

    return ESP_OK;
}

/* Compact TLV preparation - optimized for size */
static esp_err_t esp_secure_cert_prepare_tlv_compact(esp_secure_cert_tlv_type_t type, const unsigned char *value,
                                                     size_t value_len, unsigned char *output_buf, size_t *output_len,
                                                     uint8_t flags, uint8_t subtype)
{
    if (output_buf == NULL || output_len == NULL) {
        WRITE_LOGE(TAG, "Invalid TLV preparation parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (value == NULL && value_len > 0) {
        WRITE_LOGE(TAG, "TLV data pointer null but length non-zero");
        return ESP_ERR_SECURE_CERT_TLV_INVALID_DATA;
    }

    const size_t header_size = sizeof(esp_secure_cert_tlv_header_t);
    const size_t footer_size = sizeof(esp_secure_cert_tlv_footer_t);
    const uint8_t padding_len = (MIN_ALIGNMENT_REQUIRED - (value_len % MIN_ALIGNMENT_REQUIRED)) % MIN_ALIGNMENT_REQUIRED;
    const size_t total_size = header_size + value_len + padding_len + footer_size;

    if (*output_len < total_size) {
        WRITE_LOGE(TAG, "TLV output buffer too small: have=%zu, need=%zu", *output_len, total_size);
        return ESP_ERR_SECURE_CERT_TLV_BUFFER_TOO_SMALL;
    }

    // Clear output buffer first
    memset(output_buf, 0, total_size);

    // Prepare header
    esp_secure_cert_tlv_header_t *header = (esp_secure_cert_tlv_header_t *)output_buf;
    header->magic = ESP_SECURE_CERT_TLV_MAGIC;
    header->flags = flags;
    header->type = (uint8_t)type;
    header->subtype = subtype;
    header->length = (uint16_t)value_len;

    size_t pos = header_size;

    // Copy value data if present
    if (value_len > 0 && value != NULL) {
        memcpy(output_buf + pos, value, value_len);
    }
    pos += value_len + padding_len;  // Skip padding (already zeroed)

    // Calculate and add CRC32
    uint32_t crc = esp_crc32_le(UINT32_MAX, output_buf, pos);
    esp_secure_cert_tlv_footer_t *footer = (esp_secure_cert_tlv_footer_t *)(output_buf + pos);
    footer->crc = crc;

    *output_len = total_size;
    WRITE_LOGD(TAG, "TLV prepared: type=%d, len=%zu, total=%zu, crc=%"PRIu32"", type, value_len, total_size, crc);

    return ESP_OK;
}

/* Get the next available write offset in the partition */
static esp_err_t esp_secure_cert_get_next_write_offset(size_t *offset)
{
    if (offset == NULL) {
        WRITE_LOGE(TAG, "Invalid offset parameter");
        return ESP_ERR_INVALID_ARG;
    }

    esp_secure_cert_partition_ctx_t *ctx = NULL;
    if (esp_secure_cert_map_partition(&ctx) != ESP_OK || ctx == NULL) {
        WRITE_LOGE(TAG, "Failed to get partition mapped address");
        return ESP_ERR_SECURE_CERT_PARTITION_ACCESS_FAILED;
    }

    const void *esp_secure_cert_addr = ctx->esp_secure_cert_mapped_addr;
    void *tlv_address = NULL;
    esp_err_t err = esp_secure_cert_find_tlv(esp_secure_cert_addr, ESP_SECURE_CERT_TLV_END, 0, &tlv_address);
    if (err != ESP_OK) {
        WRITE_LOGD(TAG, "No TLV_END found, starting from beginning");
        *offset = 0;
        return ESP_OK;
    }

    /* Calculate offset from base address */
    *offset = (size_t)((uint8_t *)tlv_address - (uint8_t *)esp_secure_cert_addr);
    return ESP_OK;
}

/* Validate TLV information parameters */
static esp_err_t esp_secure_cert_validate_tlv_info(const esp_secure_cert_tlv_info_t *tlv_info)
{
    if (tlv_info == NULL) {
        WRITE_LOGE(TAG, "TLV info structure is null");
        return ESP_ERR_INVALID_ARG;
    }

    if (tlv_info->data == NULL && tlv_info->length > 0) {
        WRITE_LOGE(TAG, "TLV data pointer null but length=%u", (unsigned int)tlv_info->length);
        return ESP_ERR_SECURE_CERT_TLV_INVALID_DATA;
    }

    if (tlv_info->type >= ESP_SECURE_CERT_TLV_MAX) {
        WRITE_LOGE(TAG, "Invalid TLV type: %d", tlv_info->type);
        return ESP_ERR_SECURE_CERT_TLV_INVALID_TYPE;
    }

    if (tlv_info->length > UINT16_MAX) {
        WRITE_LOGE(TAG, "TLV data length %u exceeds maximum", (unsigned int)tlv_info->length);
        return ESP_ERR_SECURE_CERT_TLV_INVALID_LENGTH;
    }

    return ESP_OK;
}

/* Internal append function - must be called with lock already held for flash mode */
static esp_err_t esp_secure_cert_append_tlv_internal(esp_secure_cert_tlv_info_t *tlv_info,
                                                      const esp_secure_cert_write_config_t *cfg)
{
    esp_err_t err;

    /* Calculate required buffer size with padding */
    const size_t required_buf_len = tlv_info->length + sizeof(esp_secure_cert_tlv_header_t) +
                                   sizeof(esp_secure_cert_tlv_footer_t) + MIN_ALIGNMENT_REQUIRED;

    /* Allocate TLV buffer */
    uint8_t *output_buf = heap_caps_calloc(1, required_buf_len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (output_buf == NULL) {
        return ESP_ERR_SECURE_CERT_WRITE_NO_MEMORY;
    }

    size_t output_len = required_buf_len;
    err = esp_secure_cert_prepare_tlv_compact(tlv_info->type, (const unsigned char *)tlv_info->data,
                                             tlv_info->length, output_buf, &output_len,
                                             tlv_info->flags, tlv_info->subtype);
    if (err != ESP_OK) {
        free(output_buf);
        return err;
    }

    if (cfg->mode == ESP_SECURE_CERT_WRITE_MODE_FLASH) {
        /* Get mapped partition address */
        esp_secure_cert_partition_ctx_t *ctx = NULL;
        if (esp_secure_cert_map_partition(&ctx) != ESP_OK || ctx == NULL) {
            free(output_buf);
            return ESP_ERR_SECURE_CERT_PARTITION_ACCESS_FAILED;
        }
        const void *esp_secure_cert_addr = ctx->esp_secure_cert_mapped_addr;

        /* Check for duplicate TLV */
        void *tlv_address = NULL;
        if (esp_secure_cert_find_tlv(esp_secure_cert_addr, tlv_info->type, tlv_info->subtype, &tlv_address) == ESP_OK) {
            WRITE_LOGW(TAG, "TLV exists: type=%d subtype=%d", tlv_info->type, tlv_info->subtype);
            free(output_buf);
            return ESP_ERR_SECURE_CERT_TLV_ALREADY_EXISTS;
        }

        /* Get next write offset */
        size_t write_offset;
        err = esp_secure_cert_get_next_write_offset(&write_offset);
        if (err != ESP_OK) {
            free(output_buf);
            return err;
        }

        /* Handle erase checking if enabled */
        if (cfg->flash.check_erase) {
            bool is_erased;
            err = esp_secure_cert_check_flash_erased(write_offset, output_len, &is_erased);
            if (err != ESP_OK) {
                free(output_buf);
                return err;
            }

            if (!is_erased) {
                if (cfg->flash.auto_erase) {
                    /* Use internal erase since caller already holds the lock */
                    err = esp_secure_cert_erase_partition_internal();
                    if (err != ESP_OK) {
                        free(output_buf);
                        return err;
                    }
                    write_offset = 0;
                } else {
                    free(output_buf);
                    return ESP_ERR_SECURE_CERT_FLASH_NOT_ERASED;
                }
            }
        }

        /* Write TLV to flash (includes verification) */
        err = esp_secure_cert_write_to_flash(write_offset, output_buf, output_len);
        if (err != ESP_OK) {
            free(output_buf);
            return err;
        }

        WRITE_LOGI(TAG, "TLV written: type=%d offset=%zu len=%zu", tlv_info->type, write_offset, output_len);

    } else if (cfg->mode == ESP_SECURE_CERT_WRITE_MODE_BUFFER) {
        if (cfg->buffer.buffer == NULL || cfg->buffer.buffer_size == 0) {
            free(output_buf);
            return ESP_ERR_SECURE_CERT_BUFFER_CONFIG_INVALID;
        }

        size_t current_offset = cfg->buffer.bytes_written ? *cfg->buffer.bytes_written : 0;

        err = esp_secure_cert_write_to_buffer(cfg->buffer.buffer, cfg->buffer.buffer_size,
                                             &current_offset, output_buf, output_len);
        if (err != ESP_OK) {
            free(output_buf);
            return err;
        }

        if (cfg->buffer.bytes_written != NULL) {
            *cfg->buffer.bytes_written = current_offset;
        }

        WRITE_LOGD(TAG, "TLV buffered: type=%d len=%zu", tlv_info->type, output_len);
    } else {
        free(output_buf);
        return ESP_ERR_SECURE_CERT_INVALID_WRITE_MODE;
    }

    free(output_buf);
    return ESP_OK;
}

esp_err_t esp_secure_cert_append_tlv(esp_secure_cert_tlv_info_t *tlv_info,
                                     const esp_secure_cert_write_config_t *write_config)
{
    esp_err_t err = esp_secure_cert_validate_tlv_info(tlv_info);
    if (err != ESP_OK) {
        return err;
    }

    /* Use default config if none provided */
    esp_secure_cert_write_config_t default_config;
    const esp_secure_cert_write_config_t *cfg = write_config;
    if (cfg == NULL) {
        esp_secure_cert_write_config_init(&default_config, ESP_SECURE_CERT_WRITE_MODE_FLASH);
        cfg = &default_config;
    }

    /* Acquire write lock for flash mode */
    if (cfg->mode == ESP_SECURE_CERT_WRITE_MODE_FLASH) {
        /* Unmap first to ensure we get fresh partition data after any prior erase */
        esp_secure_cert_unmap_partition();
        err = esp_secure_cert_acquire_write_lock();
        if (err != ESP_OK) {
            return err;
        }
    }

    err = esp_secure_cert_append_tlv_internal(tlv_info, cfg);

    /* Release lock for flash mode */
    if (cfg->mode == ESP_SECURE_CERT_WRITE_MODE_FLASH) {
        esp_secure_cert_release_write_lock();
        /* Unmap partition so subsequent operations get fresh data from flash */
        esp_secure_cert_unmap_partition();
    }

    return err;
}

esp_err_t esp_secure_cert_append_tlv_batch(esp_secure_cert_tlv_info_t *tlv_entries,
                                           size_t num_entries,
                                           const esp_secure_cert_write_config_t *write_config)
{
    if (tlv_entries == NULL || num_entries == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Validate all entries upfront before any writes */
    size_t total_size = 0;
    for (size_t i = 0; i < num_entries; i++) {
        esp_err_t err = esp_secure_cert_validate_tlv_info(&tlv_entries[i]);
        if (err != ESP_OK) {
            return err;
        }
        total_size += tlv_entries[i].length + sizeof(esp_secure_cert_tlv_header_t) +
                     sizeof(esp_secure_cert_tlv_footer_t) + MIN_ALIGNMENT_REQUIRED;
    }

    /* Use default config if none provided */
    esp_secure_cert_write_config_t default_config;
    const esp_secure_cert_write_config_t *cfg = write_config;
    if (cfg == NULL) {
        esp_secure_cert_write_config_init(&default_config, ESP_SECURE_CERT_WRITE_MODE_FLASH);
        cfg = &default_config;
    }

    /* Acquire write lock for flash mode - hold for entire batch */
    bool lock_held = false;
    if (cfg->mode == ESP_SECURE_CERT_WRITE_MODE_FLASH) {
        /* Unmap first to ensure we get fresh partition data after any prior erase */
        esp_secure_cert_unmap_partition();
        esp_err_t err = esp_secure_cert_acquire_write_lock();
        if (err != ESP_OK) {
            return err;
        }
        lock_held = true;
    }

    /* For flash mode with erase check, verify once for entire batch */
    if (cfg->mode == ESP_SECURE_CERT_WRITE_MODE_FLASH && cfg->flash.check_erase) {
        size_t write_offset;
        esp_err_t err = esp_secure_cert_get_next_write_offset(&write_offset);
        if (err != ESP_OK) {
            if (lock_held) {
                esp_secure_cert_release_write_lock();
            }
            return err;
        }

        bool is_erased;
        err = esp_secure_cert_check_flash_erased(write_offset, total_size, &is_erased);
        if (err != ESP_OK) {
            if (lock_held) {
                esp_secure_cert_release_write_lock();
            }
            return err;
        }

        if (!is_erased) {
            if (cfg->flash.auto_erase) {
                /* Use internal erase since we already hold the lock */
                err = esp_secure_cert_erase_partition_internal();
                if (err != ESP_OK) {
                    if (lock_held) {
                        esp_secure_cert_release_write_lock();
                    }
                    return err;
                }
            } else {
                if (lock_held) {
                    esp_secure_cert_release_write_lock();
                }
                return ESP_ERR_SECURE_CERT_FLASH_NOT_ERASED;
            }
        }
    }

    /* Create a modified config that skips per-entry erase checks since we did it once */
    esp_secure_cert_write_config_t batch_cfg = *cfg;
    if (batch_cfg.mode == ESP_SECURE_CERT_WRITE_MODE_FLASH) {
        batch_cfg.flash.check_erase = false;  /* Already checked above */
    }

    /* Write each TLV entry using internal function (lock already held) */
    for (size_t i = 0; i < num_entries; i++) {
        esp_err_t err = esp_secure_cert_append_tlv_internal(&tlv_entries[i], &batch_cfg);
        if (err != ESP_OK) {
            WRITE_LOGE(TAG, "Batch write failed at entry %zu", i);
            if (lock_held) {
                esp_secure_cert_release_write_lock();
            }
            return err;
        }
    }

    if (lock_held) {
        esp_secure_cert_release_write_lock();
        /* Unmap partition so subsequent operations get fresh data from flash */
        esp_secure_cert_unmap_partition();
    }

    WRITE_LOGI(TAG, "Batch write completed: %zu entries", num_entries);
    return ESP_OK;
}

#if SOC_HMAC_SUPPORTED

#if (MBEDTLS_MAJOR_VERSION < 4)
/* Encrypt data using HMAC-based encryption - mbedtls version */
static esp_err_t esp_secure_cert_encrypt_with_hmac(const esp_secure_cert_tlv_info_t *tlv_info,
                                                    unsigned char **encrypted_data, size_t *encrypted_len)
{
    if (tlv_info == NULL || encrypted_data == NULL || encrypted_len == NULL) {
        WRITE_LOGE(TAG, "Invalid HMAC encryption parameters");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t esp_ret = ESP_FAIL;
    int ret = -1;
    esp_efuse_block_t efuse_block = EFUSE_BLK_MAX;
    mbedtls_gcm_context gcm_ctx;
    mbedtls_gcm_init(&gcm_ctx);

    if (!esp_efuse_find_purpose(ESP_EFUSE_KEY_PURPOSE_HMAC_UP, &efuse_block)) {
        WRITE_LOGE(TAG, "HMAC_UP key block not found");
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_KEY_NOT_FOUND;
        goto cleanup;
    }

    WRITE_LOGI(TAG, "eFuse block %d found with key purpose set to HMAC_UP", efuse_block);

    uint8_t aes_gcm_key[HMAC_ENCRYPTION_AES_GCM_KEY_LEN];
    uint8_t iv[HMAC_ENCRYPTION_IV_LEN];

    esp_ret = esp_secure_cert_calculate_hmac_encryption_iv(iv);
    if (esp_ret != ESP_OK) {
        WRITE_LOGE(TAG, "HMAC IV generation failed");
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_IV_GENERATION_FAILED;
        goto cleanup;
    }

    esp_ret = esp_secure_cert_calculate_hmac_encryption_key(aes_gcm_key);
    if (esp_ret != ESP_OK) {
        WRITE_LOGE(TAG, "HMAC key generation failed");
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_ENCRYPTION_FAILED;
        goto cleanup;
    }

    ret = mbedtls_gcm_setkey(&gcm_ctx, MBEDTLS_CIPHER_ID_AES,
                             (unsigned char *)aes_gcm_key, HMAC_ENCRYPTION_AES_GCM_KEY_LEN * 8);
    if (ret != 0) {
        WRITE_LOGE(TAG, "mbedtls_gcm_setkey failed: -0x%04X", -ret);
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_ENCRYPTION_FAILED;
        goto cleanup;
    }

    /* Allocate buffer for encrypted data + tag */
    *encrypted_len = tlv_info->length + HMAC_ENCRYPTION_TAG_LEN;
    *encrypted_data = heap_caps_calloc(1, *encrypted_len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (*encrypted_data == NULL) {
        WRITE_LOGE(TAG, "Failed to allocate encryption buffer");
        esp_ret = ESP_ERR_SECURE_CERT_WRITE_NO_MEMORY;
        goto cleanup;
    }

    ret = mbedtls_gcm_crypt_and_tag(&gcm_ctx, MBEDTLS_GCM_ENCRYPT, tlv_info->length, iv,
                                    HMAC_ENCRYPTION_IV_LEN, NULL, 0, (const unsigned char *)tlv_info->data,
                                    *encrypted_data, HMAC_ENCRYPTION_TAG_LEN,
                                    *encrypted_data + tlv_info->length);

    if (ret != 0) {
        WRITE_LOGE(TAG, "mbedtls_gcm_crypt_and_tag failed: 0x%02X", ret);
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_ENCRYPTION_FAILED;
        goto cleanup;
    }

    esp_ret = ESP_OK;

cleanup:
    mbedtls_gcm_free(&gcm_ctx);
    mbedtls_platform_zeroize(aes_gcm_key, sizeof(aes_gcm_key));
    mbedtls_platform_zeroize(iv, sizeof(iv));
    if (esp_ret != ESP_OK && *encrypted_data != NULL) {
        free(*encrypted_data);
        *encrypted_data = NULL;
        *encrypted_len = 0;
    }
    return esp_ret;
}

#else /* MBEDTLS_MAJOR_VERSION >= 4: Use PSA Crypto API */

/* Encrypt data using HMAC-based encryption - PSA version */
static esp_err_t esp_secure_cert_encrypt_with_hmac(const esp_secure_cert_tlv_info_t *tlv_info,
                                                    unsigned char **encrypted_data, size_t *encrypted_len)
{
    if (tlv_info == NULL || encrypted_data == NULL || encrypted_len == NULL) {
        WRITE_LOGE(TAG, "Invalid HMAC encryption parameters");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t esp_ret = ESP_FAIL;
    psa_status_t status;
    psa_key_id_t key_id = 0;
    esp_efuse_block_t efuse_block = EFUSE_BLK_MAX;

    *encrypted_data = NULL;
    *encrypted_len = 0;

    uint8_t aes_gcm_key[HMAC_ENCRYPTION_AES_GCM_KEY_LEN];
    uint8_t iv[HMAC_ENCRYPTION_IV_LEN];
    memset(aes_gcm_key, 0, sizeof(aes_gcm_key));
    memset(iv, 0, sizeof(iv));

    if (!esp_efuse_find_purpose(ESP_EFUSE_KEY_PURPOSE_HMAC_UP, &efuse_block)) {
        WRITE_LOGE(TAG, "HMAC_UP key block not found");
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_KEY_NOT_FOUND;
        goto cleanup;
    }

    WRITE_LOGI(TAG, "eFuse block %d found with key purpose set to HMAC_UP", efuse_block);

    esp_ret = esp_secure_cert_calculate_hmac_encryption_iv(iv);
    if (esp_ret != ESP_OK) {
        WRITE_LOGE(TAG, "HMAC IV generation failed");
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_IV_GENERATION_FAILED;
        goto cleanup;
    }

    esp_ret = esp_secure_cert_calculate_hmac_encryption_key(aes_gcm_key);
    if (esp_ret != ESP_OK) {
        WRITE_LOGE(TAG, "HMAC key generation failed");
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_ENCRYPTION_FAILED;
        goto cleanup;
    }

    /* Import key into PSA */
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attributes, PSA_ALG_GCM);
    psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attributes, HMAC_ENCRYPTION_AES_GCM_KEY_LEN * 8);

    status = psa_import_key(&attributes, aes_gcm_key, HMAC_ENCRYPTION_AES_GCM_KEY_LEN, &key_id);
    if (status != PSA_SUCCESS) {
        WRITE_LOGE(TAG, "Failed to import key, returned %d", (int)status);
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_ENCRYPTION_FAILED;
        goto cleanup;
    }

    /* Allocate buffer for encrypted data + tag */
    size_t output_len = tlv_info->length + HMAC_ENCRYPTION_TAG_LEN;
    *encrypted_data = heap_caps_calloc(1, output_len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (*encrypted_data == NULL) {
        WRITE_LOGE(TAG, "Failed to allocate encryption buffer");
        esp_ret = ESP_ERR_SECURE_CERT_WRITE_NO_MEMORY;
        goto cleanup;
    }

    /* Perform AEAD encryption */
    size_t ciphertext_len = 0;
    status = psa_aead_encrypt(key_id, PSA_ALG_GCM,
                              iv, HMAC_ENCRYPTION_IV_LEN,
                              NULL, 0,  /* No additional data */
                              (const unsigned char *)tlv_info->data, tlv_info->length,
                              *encrypted_data, output_len,
                              &ciphertext_len);

    if (status != PSA_SUCCESS) {
        WRITE_LOGE(TAG, "psa_aead_encrypt failed: %d", (int)status);
        esp_ret = ESP_ERR_SECURE_CERT_HMAC_ENCRYPTION_FAILED;
        goto cleanup;
    }

    *encrypted_len = ciphertext_len;
    esp_ret = ESP_OK;

cleanup:
    if (key_id != 0) {
        psa_destroy_key(key_id);
    }
    mbedtls_platform_zeroize(aes_gcm_key, sizeof(aes_gcm_key));
    mbedtls_platform_zeroize(iv, sizeof(iv));
    if (esp_ret != ESP_OK && *encrypted_data != NULL) {
        free(*encrypted_data);
        *encrypted_data = NULL;
        *encrypted_len = 0;
    }
    return esp_ret;
}
#endif /* (MBEDTLS_MAJOR_VERSION < 4) */

esp_err_t esp_secure_cert_append_tlv_with_hmac_encryption(esp_secure_cert_tlv_info_t *tlv_info,
                                                          const esp_secure_cert_write_config_t *write_config)
{
    esp_err_t err = esp_secure_cert_validate_tlv_info(tlv_info);
    if (err != ESP_OK) {
        return err;
    }

    unsigned char *encrypted_data = NULL;
    size_t encrypted_len = 0;
    esp_secure_cert_tlv_info_t tlv_info_hmac_enc = {};

    WRITE_LOGI(TAG, "Encrypting data with HMAC-based encryption");

    err = esp_secure_cert_encrypt_with_hmac(tlv_info, &encrypted_data, &encrypted_len);
    if (err != ESP_OK) {
        WRITE_LOGE(TAG, "HMAC encryption failed");
        return err;
    }

    // Setup encrypted TLV info
    tlv_info_hmac_enc.type = tlv_info->type;
    tlv_info_hmac_enc.subtype = tlv_info->subtype;
    tlv_info_hmac_enc.data = (char *)encrypted_data;
    tlv_info_hmac_enc.length = encrypted_len;
    tlv_info_hmac_enc.flags = tlv_info->flags | ESP_SECURE_CERT_TLV_FLAG_HMAC_ENCRYPTION;

    err = esp_secure_cert_append_tlv(&tlv_info_hmac_enc, write_config);
    if (err != ESP_OK) {
        WRITE_LOGE(TAG, "Encrypted TLV write failed: %s", esp_err_to_name(err));
    }

    free(encrypted_data);
    return err;
}

esp_err_t esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(const uint8_t *salt, size_t salt_len,
                                                                 esp_secure_cert_tlv_subtype_t subtype,
                                                                 const esp_secure_cert_write_config_t *write_config)
{
    if (salt == NULL || salt_len == 0) {
        WRITE_LOGE(TAG, "Invalid salt parameters");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;
    esp_efuse_block_t efuse_block = EFUSE_BLK_MAX;

    /* Verify HMAC_UP key exists in eFuse */
    if (!esp_efuse_find_purpose(ESP_EFUSE_KEY_PURPOSE_HMAC_UP, &efuse_block)) {
        WRITE_LOGE(TAG, "HMAC_UP key block not found in eFuse");
        return ESP_ERR_SECURE_CERT_HMAC_KEY_NOT_FOUND;
    }

    WRITE_LOGI(TAG, "HMAC_UP key found in eFuse block %d", efuse_block);

    /* Use default config if none provided */
    esp_secure_cert_write_config_t default_config;
    const esp_secure_cert_write_config_t *cfg = write_config;
    if (cfg == NULL) {
        esp_secure_cert_write_config_init(&default_config, ESP_SECURE_CERT_WRITE_MODE_FLASH);
        cfg = &default_config;
    }

    /* Acquire write lock for flash mode - hold for both TLV writes atomically */
    bool lock_held = false;
    if (cfg->mode == ESP_SECURE_CERT_WRITE_MODE_FLASH) {
        /* Unmap first to ensure we get fresh partition data after any prior erase */
        esp_secure_cert_unmap_partition();
        err = esp_secure_cert_acquire_write_lock();
        if (err != ESP_OK) {
            return err;
        }
        lock_held = true;
    }

    /* Step 1: Write the salt TLV entry */
    esp_secure_cert_tlv_info_t salt_tlv = {
        .type = ESP_SECURE_CERT_HMAC_ECDSA_KEY_SALT,
        .subtype = subtype,
        .data = (char *)salt,
        .length = salt_len,
        .flags = 0
    };

    err = esp_secure_cert_append_tlv_internal(&salt_tlv, cfg);
    if (err != ESP_OK) {
        WRITE_LOGE(TAG, "Failed to write salt TLV: %s", esp_err_to_name(err));
        if (lock_held) {
            esp_secure_cert_release_write_lock();
        }
        return err;
    }

    WRITE_LOGI(TAG, "Salt TLV written successfully (len=%zu)", salt_len);

    /* Step 2: Write the private key marker TLV with derivation flag
     * This TLV has zero data length but indicates that the key should be
     * derived at read time using PBKDF2-HMAC-SHA256 from the salt */
    esp_secure_cert_tlv_info_t priv_key_marker = {
        .type = ESP_SECURE_CERT_PRIV_KEY_TLV,
        .subtype = subtype,
        .data = NULL,
        .length = 0,
        .flags = ESP_SECURE_CERT_TLV_FLAG_HMAC_ECDSA_KEY_DERIVATION
    };

    err = esp_secure_cert_append_tlv_internal(&priv_key_marker, cfg);
    if (err != ESP_OK) {
        WRITE_LOGE(TAG, "Failed to write private key marker TLV: %s", esp_err_to_name(err));
        if (lock_held) {
            esp_secure_cert_release_write_lock();
        }
        return err;
    }

    if (lock_held) {
        esp_secure_cert_release_write_lock();
        /* Unmap partition so subsequent operations get fresh data from flash */
        esp_secure_cert_unmap_partition();
    }

    WRITE_LOGI(TAG, "HMAC-ECDSA derivation configured: salt written, marker TLV set with derivation flag");
    return ESP_OK;
}

/* Maximum number of attempts to generate a valid ECDSA key */
#define ESP_SECURE_CERT_ECDSA_KEY_GEN_MAX_TRIES  (30)

/* HMAC key size (256 bits) */
#define ESP_SECURE_CERT_HMAC_KEY_SIZE            (32)

esp_err_t esp_secure_cert_derive_hmac_ecdsa_key(uint8_t *pub_key_buf, size_t *pub_key_len,
                                                  esp_secure_cert_tlv_subtype_t subtype,
                                                  const esp_secure_cert_write_config_t *write_config)
{
    esp_err_t err = ESP_FAIL;
    esp_efuse_block_t efuse_block = EFUSE_BLK_MAX;

    /* Check if HMAC_UP key already exists in eFuse */
    if (esp_efuse_find_purpose(ESP_EFUSE_KEY_PURPOSE_HMAC_UP, &efuse_block)) {
        WRITE_LOGE(TAG, "HMAC_UP key already exists in eFuse block %d", efuse_block);
        return ESP_ERR_SECURE_CERT_HMAC_KEY_ALREADY_EXISTS;
    }

    /* Allocate buffers for key generation */
    uint8_t *salt = heap_caps_calloc(1, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint8_t *hmac_key = heap_caps_calloc(1, ESP_SECURE_CERT_HMAC_KEY_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint8_t *derived_key = heap_caps_calloc(1, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    uint8_t *hw_derived_key = heap_caps_calloc(1, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (salt == NULL || hmac_key == NULL || derived_key == NULL || hw_derived_key == NULL) {
        WRITE_LOGE(TAG, "Failed to allocate memory for key generation");
        err = ESP_ERR_SECURE_CERT_WRITE_NO_MEMORY;
        goto cleanup;
    }

    /* Step 1: Generate random salt */
    esp_fill_random(salt, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE);
    WRITE_LOGI(TAG, "Generated random salt (%d bytes)", ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE);

    /* Step 2: Generate HMAC key and derive ECDSA key, validate it's valid */
    bool key_valid = false;
    for (int attempt = 0; attempt < ESP_SECURE_CERT_ECDSA_KEY_GEN_MAX_TRIES; attempt++) {
        /* Generate random HMAC key */
        esp_fill_random(hmac_key, ESP_SECURE_CERT_HMAC_KEY_SIZE);

        /* Derive ECDSA key using software PBKDF2 */
        err = esp_secure_cert_sw_pbkdf2_hmac_sha256(
            hmac_key, ESP_SECURE_CERT_HMAC_KEY_SIZE,
            salt, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE,
            ESP_SECURE_CERT_KEY_DERIVATION_ITERATION_COUNT,
            derived_key, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE
        );
        if (err != ESP_OK) {
            WRITE_LOGE(TAG, "Software PBKDF2 failed on attempt %d", attempt + 1);
            continue;
        }

        /* Validate the derived key is a valid ECDSA private key */
        err = esp_secure_cert_validate_ecdsa_key(derived_key, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE);
        if (err == ESP_OK) {
            key_valid = true;
            WRITE_LOGI(TAG, "Valid ECDSA key generated on attempt %d", attempt + 1);
            break;
        }

        WRITE_LOGD(TAG, "Key validation failed on attempt %d, retrying...", attempt + 1);
    }

    if (!key_valid) {
        WRITE_LOGE(TAG, "Failed to generate valid ECDSA key after %d attempts",
                   ESP_SECURE_CERT_ECDSA_KEY_GEN_MAX_TRIES);
        err = ESP_ERR_SECURE_CERT_ECDSA_KEY_GEN_FAILED;
        goto cleanup;
    }

    /* Step 3: Find an unused eFuse key block and burn the HMAC key */
    efuse_block = esp_efuse_find_unused_key_block();
    if (efuse_block == EFUSE_BLK_KEY_MAX) {
        WRITE_LOGE(TAG, "No unused eFuse key block available");
        err = ESP_ERR_SECURE_CERT_EFUSE_WRITE_FAILED;
        goto cleanup;
    }

    WRITE_LOGI(TAG, "Burning HMAC key to eFuse block %d", efuse_block);
    err = esp_efuse_write_key(efuse_block, ESP_EFUSE_KEY_PURPOSE_HMAC_UP,
                              hmac_key, ESP_SECURE_CERT_HMAC_KEY_SIZE);
    if (err != ESP_OK) {
        WRITE_LOGE(TAG, "Failed to write HMAC key to eFuse: %s", esp_err_to_name(err));
        err = ESP_ERR_SECURE_CERT_EFUSE_WRITE_FAILED;
        goto cleanup;
    }

    /* Clear the HMAC key from memory immediately after burning */
    mbedtls_platform_zeroize(hmac_key, ESP_SECURE_CERT_HMAC_KEY_SIZE);
    WRITE_LOGI(TAG, "HMAC key burned to eFuse block %d", efuse_block);

    /* Step 4: Verify hardware HMAC can reproduce the same key */
    int ret = esp_pbkdf2_hmac_sha256(
        efuse_block - (int)EFUSE_BLK_KEY0,
        salt, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE,
        ESP_SECURE_CERT_KEY_DERIVATION_ITERATION_COUNT,
        ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE,
        hw_derived_key
    );
    if (ret != 0) {
        WRITE_LOGE(TAG, "Hardware PBKDF2 derivation failed");
        err = ESP_ERR_SECURE_CERT_PBKDF2_FAILED;
        goto cleanup;
    }

    /* Compare software and hardware derived keys */
    if (memcmp(derived_key, hw_derived_key, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE) != 0) {
        WRITE_LOGE(TAG, "Hardware-derived key does not match software-derived key");
        err = ESP_ERR_SECURE_CERT_KEY_VERIFICATION_FAILED;
        goto cleanup;
    }
    WRITE_LOGI(TAG, "Hardware key derivation verified successfully");

    /* Step 5: Calculate and return public key if requested */
    if (pub_key_buf != NULL && pub_key_len != NULL) {
        err = esp_secure_cert_calc_public_key(derived_key, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE,
                                               pub_key_buf, pub_key_len);
        if (err != ESP_OK) {
            WRITE_LOGE(TAG, "Failed to calculate public key: %s", esp_err_to_name(err));
            goto cleanup;
        }
        WRITE_LOGI(TAG, "Public key calculated (%zu bytes)", *pub_key_len);
    }

    /* Step 6: Store salt in partition using the existing derivation API */
    err = esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(
        salt, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE,
        subtype, write_config
    );
    if (err != ESP_OK) {
        WRITE_LOGE(TAG, "Failed to write derivation TLVs: %s", esp_err_to_name(err));
        goto cleanup;
    }

    WRITE_LOGI(TAG, "HMAC-ECDSA key derivation setup complete");
    err = ESP_OK;

cleanup:
    /* Clear sensitive data using zeroize to prevent compiler optimization */
    if (salt) {
        mbedtls_platform_zeroize(salt, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE);
        free(salt);
    }
    if (hmac_key) {
        mbedtls_platform_zeroize(hmac_key, ESP_SECURE_CERT_HMAC_KEY_SIZE);
        free(hmac_key);
    }
    if (derived_key) {
        mbedtls_platform_zeroize(derived_key, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE);
        free(derived_key);
    }
    if (hw_derived_key) {
        mbedtls_platform_zeroize(hw_derived_key, ESP_SECURE_CERT_DERIVED_ECDSA_KEY_SIZE);
        free(hw_derived_key);
    }

    return err;
}
#endif
