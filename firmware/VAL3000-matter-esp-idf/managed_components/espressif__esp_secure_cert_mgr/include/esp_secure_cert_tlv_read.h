/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* Check IDF version for write support (requires >= 5.3) */
#if __has_include("esp_idf_version.h")
    #include "esp_idf_version.h"
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
        #define ESP_SECURE_CERT_WRITE_SUPPORT 1
    #endif
#endif

/* Include atomic support only when write support is available */
#ifdef ESP_SECURE_CERT_WRITE_SUPPORT
    #ifdef __cplusplus
        #include <atomic>
        using atomic_bool = std::atomic<bool>;
    #else
        #include <stdatomic.h>
    #endif // __cplusplus
#endif // ESP_SECURE_CERT_WRITE_SUPPORT

#include "esp_partition.h"
#if defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "spi_flash_mmap.h"
#endif

#include "esp_secure_cert_tlv_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * Context structure to hold the partition information
 * and memory mapped address for esp_secure_cert partition
 */
typedef struct esp_secure_cert_partition_ctx {
    const esp_partition_t *partition;           /* Pointer to the esp_secure_cert partition */
    const void *esp_secure_cert_mapped_addr;    /* Memory mapped address of the partition */
    spi_flash_mmap_handle_t handle;             /* Memory map handle */
#ifdef ESP_SECURE_CERT_WRITE_SUPPORT
    atomic_bool write_lock;                     /* Atomic lock for write operations (1 byte) */
#endif
} esp_secure_cert_partition_ctx_t;

/*
 * TLV config struct
 */
typedef struct tlv_config {
    esp_secure_cert_tlv_type_t type; /* TLV type */
    esp_secure_cert_tlv_subtype_t subtype; /* TLV subtype */
} esp_secure_cert_tlv_config_t;

/*
 * TLV info struct
 */
typedef struct tlv_info {
    esp_secure_cert_tlv_type_t type; /* Type of the TLV */
    esp_secure_cert_tlv_subtype_t subtype; /* Subtype of the TLV */
    char *data; /* Pointer to the buffer containting TLV data */
    uint32_t length; /* TLV data length */
    uint8_t flags;
} esp_secure_cert_tlv_info_t;

/*
 * TLV iterator struct
 */
typedef struct tlv_iterator {
    void *iterator; /* Opaque TLV iterator */
} esp_secure_cert_tlv_iterator_t;

/*
 *  Get the TLV information for given TLV configuration
 *
 *  @note
 *  TLV Algorithms:
 *  If the TLV data is stored with some additional encryption then it first needs to be decrypted and the decrypted data is
 *  stored in a dynamically allocated buffer. This API automatically decrypts any encryption applied to the TLV by supported algorithms.
 *  For this the API may look for TLV entries of other types which store necessary information, these TLV entries must be of the same subtype as of the subtype field in the config struct.
 *  Please see documentation regarding supported TLV storage algorithms in the TLV documentation.
 *  A call to the esp_secure_cert_free_tlv_info() should be made to free any memory allocated while populating the tlv information object.
 *  This API also validates the crc of the respective tlv before returning the offset.
 *
 *  If tlv type in the config struct is set to ESP_SECURE_CERT_TLV_END then the address returned shall be the end address of current tlv formatted data and the length returned shall be the total length of the valid TLV entries.
 * @input
 *     tlv_config           Pointer to a readable struct of type esp_secure_cert_tlv_config_t.
 *                          The contents of the struct must be already filled by the caller,
 *                          This information shall be used to find the appropriate TLV entry.
 *
 *     tlv_info             Pointer to a writable struct of type esp_secure_cert_tlv_info_t,
 *                          If TLV entry defined by tlv_config is found then the TLV information shall be populated in this struct.
 * @return
 *
 *      - ESP_OK    On success
 *      - ESP_FAIL/other relevant esp error code
 *                  On failure
 */
esp_err_t esp_secure_cert_get_tlv_info(esp_secure_cert_tlv_config_t *tlv_config, esp_secure_cert_tlv_info_t *tlv_info);

/*
 * Free the memory allocated while populating the tlv_info object
 * @note
 * Please note this does not free the tlv_info struct itself but only the memory allocated internally while populating this struct.
 */
esp_err_t esp_secure_cert_free_tlv_info(esp_secure_cert_tlv_info_t *tlv_info);

/*
 * Iterate to the next valid TLV entry
 * @note
 *       To obtain the first TLV entry, the tlv_iterator structure must be zero initialized
 * @input
 *      tlv_iterator Pointer to a readable struct of type esp_secure_cert_tlv_iterator_t
 *
 * @return
 *  ESP_OK      On success
 *              The iterator location shall be moved to point to the next TLV entry.
 *  ESP_FAIL/other relevant error codes
 *              On failure
 */
esp_err_t esp_secure_cert_iterate_to_next_tlv(esp_secure_cert_tlv_iterator_t *tlv_iterator);

/*
 * Get the TLV information from a valid iterator location
 *
 * @note
 * A call to the esp_secure_cert_free_tlv_info() should be made to free any memory allocated while populating the tlv information object.
 *
 * @input
 *     tlv_config           Pointer to a readable struct of type esp_secure_cert_tlv_iterator_t.
 *                          The iterator must be set to point to a valid TLV,
 *                          by a previous call to esp_secure_cert_iterate_to_next_tlv();.
 *
 *     tlv_info             Pointer to a writable struct of type esp_secure_cert_tlv_info_t
 *                          If TLV entry pointed by the iterator is valid then the TLV information shall be populated in this struct.
 * @return
 *  ESP_OK  On success
 *          The tlv_info object shall be populated with information of the TLV pointed by the iterator
 *  ESP_FAIL/other relevant error codes
 *          On failure
 */
esp_err_t esp_secure_cert_get_tlv_info_from_iterator(esp_secure_cert_tlv_iterator_t *tlv_iterator, esp_secure_cert_tlv_info_t *tlv_info);

/*
 * List TLV entries
 *
 * This API serially traverses through all of the available
 * TLV entries in the esp_secure_cert partition and logs
 * brief information about each TLV entry.
 */
void esp_secure_cert_list_tlv_entries(void);

/**
 * @brief Initialize the esp_secure_cert partition context.
 * This function maps the entire esp_secure_cert partition and
 * populates the context structure with partition information.
 *
 * @param[out] ctx Output parameter that will point to the partition context.
 *                 Pass the address of a pointer variable.
 *
 * @return
 *      - ESP_OK    Successfully mapped partition or already mapped
 *      - ESP_FAIL  Failed to find or map the partition
 *
 * @note If the partition is already mapped, this function returns immediately
 *       without remapping. This allows safe repeated calls.
 */
esp_err_t esp_secure_cert_map_partition(esp_secure_cert_partition_ctx_t **ctx);

/*
 * Unmap the esp_secure_cert partition to free memory.
 *
 * This API is useful for memory-constrained systems where you want to
 * temporarily free the memory used by the mapped partition when secure
 * cert operations are not actively needed.
 *
 * @note
 * After calling this function, any subsequent calls to esp_secure_cert APIs
 * will automatically remap the partition as needed.
 */
void esp_secure_cert_unmap_partition(void);

#ifdef __cplusplus
}
#endif
