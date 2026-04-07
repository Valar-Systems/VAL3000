# ESP Secure Certificate Manager

The *esp_secure_cert_mgr* provides a simplified interface to access the PKI credentials of a device pre-provisioned with the
Espressif Provisioning Service. It provides the set of APIs that are required to access the contents of
the `esp_secure_cert` partition.
A demo example has also been provided with the `esp_secure_cert_mgr`, more details can be found out
in the [example README](https://github.com/espressif/esp_secure_cert_mgr/blob/main/examples/esp_secure_cert_app/README.md)

## Usage Guidelines

### 1) Include `esp_secure_cert_mgr` in your project
There are two ways to include `esp_secure_cert_mgr` in your project:

i) Add `esp_secure_cert_mgr` to your project with help of IDF component manager:
* The component is hosted at https://components.espressif.com/component/espressif/esp_secure_cert_mgr. Please use the same link to obtain the latest available version of the component along with the instructions on how to add it to your project.
* Additional details about using a component through IDF component manager can be found [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html#using-with-a-project)

ii) Add `esp_secure_cert_mgr` as an extra component in your project.

* Download `esp_secure_cert_mgr` with:
```
    git clone https://github.com/espressif/esp_secure_cert_mgr.git
```
* Include  `esp_secure_cert_mgr` in `ESP-IDF` with setting `EXTRA_COMPONENT_DIRS` in CMakeLists.txt/Makefile of your project.For reference see [Optional Project Variables](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#optional-project-variables)

### 2) Use the public API provided by `esp_secure_cert_mgr` in your project
* **Read APIs**: The file [esp_secure_cert_read.h](https://github.com/espressif/esp_secure_cert_mgr/blob/main/include/esp_secure_cert_read.h) contains the public APIs for reading from the `esp_secure_cert` partition.
* **Write APIs**: The file [esp_secure_cert_write.h](https://github.com/espressif/esp_secure_cert_mgr/blob/main/include/esp_secure_cert_write.h) contains the public APIs for writing to the `esp_secure_cert` partition (see [Write API Documentation](#write-api-documentation) below).

## What is Pre-Provisioning?

With the Espressif Pre-Provisioning Service, the ESP modules are pre-provisioned with an encrypted RSA private key and respective X509 public certificate before they are shipped out to you. The PKI credentials can then be registered with the cloud service to establish a secure TLS channel for communication. With the pre-provisioning taking place in the factory, it provides a hassle-free PKI infrastructure to the Makers. You may use this repository to set up your test modules to validate that your firmware works with the pre-provisioned modules that you ordered through Espressif's pre-provisioning service.

## ESP Secure Cert Partition

When a device is pre-provisioned that means the PKI credentials are generated for the device. The PKI credentials are then stored in a partition named
*esp_secure_cert*.

The `esp_secure_cert` partition can be generated on host with help of [configure_esp_secure_cert.py](https://github.com/espressif/esp_secure_cert_mgr/blob/main/tools/configure_esp_secure_cert.py) utility, more details about the utility can be found in the [tools/README](https://github.com/espressif/esp_secure_cert_mgr/tree/main/tools#readme).

For esp devices that support DS peripheral, the pre-provisioning is done by leveraging the security benefit of the DS peripheral. In that case, all of the data which is present in the *esp_secure_cert* partition is completely secure.

When the device is pre-provisioned with help of the DS peripheral then by default the partition primarily contains the following data:
1) Device certificate: It is the public key/ certificate for the device's private key. It is used in TLS authentication.
2) CA certificate: This is the certificate of the CA which is used to sign the device cert.
3) Ciphertext: This is the encrypted private key of the device. The ciphertext is encrypted using the DS peripheral, thus it is completely safe to store on the flash.

As listed above, the data only contains the public certificates and the encrypted private key and hence it is completely secure in itself. There is no need to further encrypt this data with any additional security algorithm.

## Trying Out the `esp_secure_cert` Tool
The `esp_secure_cert` tool provides a convenient way to generate the `esp_secure_cert` partition for your ESP device, including support for advanced use cases such as Digital Signature (DS) peripheral and custom partition layouts. You can use the tool to create the partition from a CSV configuration file, and test the process using `QEMU` (an emulator) before flashing to real hardware. For detailed instructions on how to generate and test the `esp_secure_cert` partition using QEMU, please refer to the [tools/README.md](https://github.com/espressif/esp_secure_cert_mgr/blob/main/tools/README.md#test-esp_secure_cert-partition-using-qemu).

### Partition Format

The *esp_secure_cert* partition uses TLV format by default. Please take a look at the [format document](https://github.com/espressif/esp_secure_cert_mgr/tree/main/docs/format.md) for more details.

## Write API Documentation

The `esp_secure_cert_mgr` component provides APIs for writing TLV entries to the `esp_secure_cert` partition at runtime. This is useful for:
- Storing custom user data alongside certificates
- Runtime provisioning of credentials
- Generating partition images in memory (for host-side tools)

> **⚠️ Warning**: Writing to the `esp_secure_cert` partition will modify device credentials. Use with caution, especially on production devices.

### Key APIs

Include the header file:
```c
#include "esp_secure_cert_write.h"
```

#### 1. Erase Partition
```c
esp_err_t esp_secure_cert_erase_partition(void);
```
Completely erases the `esp_secure_cert` partition. **This is irreversible.**

#### 2. Write a Single TLV Entry
```c
esp_err_t esp_secure_cert_append_tlv(esp_secure_cert_tlv_info_t *tlv_info,
                                     const esp_secure_cert_write_config_t *write_config);
```

#### 3. Write Multiple TLV Entries (Batch)
```c
esp_err_t esp_secure_cert_append_tlv_batch(esp_secure_cert_tlv_info_t *tlv_entries,
                                           size_t num_entries,
                                           const esp_secure_cert_write_config_t *write_config);
```

#### 4. Write with HMAC Encryption (SOC_HMAC_SUPPORTED only)
```c
esp_err_t esp_secure_cert_append_tlv_with_hmac_encryption(esp_secure_cert_tlv_info_t *tlv_info,
                                                          const esp_secure_cert_write_config_t *write_config);
```
Encrypts data using HMAC-based AES-GCM before writing. Requires an HMAC key in eFuse with `HMAC_UP` purpose.

#### 5. Write HMAC-based ECDSA Key Derivation (SOC_HMAC_SUPPORTED only)
```c
esp_err_t esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(const uint8_t *salt, size_t salt_len,
                                                                 esp_secure_cert_tlv_subtype_t subtype,
                                                                 const esp_secure_cert_write_config_t *write_config);
```
Sets up HMAC-based ECDSA private key derivation. Instead of storing the actual private key, stores a salt value. At read time, the private key is derived using PBKDF2-HMAC-SHA256 with 2048 iterations. Requires an HMAC key in eFuse with `HMAC_UP` purpose.

### Write Modes

The write configuration supports two modes:

| Mode | Description | Use Case |
|------|-------------|----------|
| `ESP_SECURE_CERT_WRITE_MODE_FLASH` | Write directly to flash partition | Runtime provisioning on device |
| `ESP_SECURE_CERT_WRITE_MODE_BUFFER` | Write to memory buffer | Host-side partition generation |

### Basic Usage Example

```c
#include "esp_secure_cert_write.h"

// Erase partition first (required for clean writes)
esp_secure_cert_erase_partition();

// Prepare TLV info
esp_secure_cert_tlv_info_t tlv_info = {
    .type = ESP_SECURE_CERT_USER_DATA_1,
    .subtype = ESP_SECURE_CERT_SUBTYPE_0,
    .data = "My custom data",
    .length = strlen("My custom data") + 1,
    .flags = 0
};

// Write using default (legacy) mode
esp_err_t err = esp_secure_cert_append_tlv(&tlv_info, NULL);
```

### Advanced Usage: Flash Mode with Erase Checking

```c
// Initialize write config for flash mode
esp_secure_cert_write_config_t config;
esp_secure_cert_write_config_init(&config, ESP_SECURE_CERT_WRITE_MODE_FLASH);
config.flash.check_erase = true;   // Verify flash is erased before writing
config.flash.auto_erase = false;   // Don't auto-erase (safer)

esp_err_t err = esp_secure_cert_append_tlv(&tlv_info, &config);
```

### Advanced Usage: Buffer Mode (Host Generation)

```c
// Allocate buffer for partition image
uint8_t *buffer = malloc(8192);
size_t bytes_written = 0;

// Initialize write config for buffer mode
esp_secure_cert_write_config_t config;
esp_secure_cert_write_config_init(&config, ESP_SECURE_CERT_WRITE_MODE_BUFFER);
config.buffer.buffer = buffer;
config.buffer.buffer_size = 8192;
config.buffer.bytes_written = &bytes_written;

// Write TLV entries to buffer
esp_secure_cert_append_tlv(&tlv_info, &config);

// 'buffer' now contains the partition image (bytes_written bytes)
```

### Advanced Usage: HMAC-based ECDSA Key Derivation

This method stores only a salt value instead of the actual private key. The private key is derived on-demand using PBKDF2-HMAC-SHA256. This provides better security as the raw key never exists in flash.

```c
#include "esp_secure_cert_write.h"
#include "esp_random.h"

// Generate random salt (typically 32 bytes for P-256)
uint8_t salt[32];
esp_fill_random(salt, sizeof(salt));

// Erase partition first
esp_secure_cert_erase_partition();

// Write HMAC-ECDSA derivation configuration
// This writes:
// 1. Salt TLV (ESP_SECURE_CERT_HMAC_ECDSA_KEY_SALT)
// 2. Private key marker TLV with derivation flag
esp_err_t err = esp_secure_cert_append_tlv_with_hmac_ecdsa_derivation(
    salt, sizeof(salt),
    ESP_SECURE_CERT_SUBTYPE_0,
    NULL  // Use default flash write mode
);

// Later, when reading the private key:
// esp_secure_cert_get_priv_key() will automatically derive the key
// using PBKDF2-HMAC-SHA256(salt, hmac_key, 2048 iterations)
// Output: 121-byte DER-encoded ECDSA private key (SECP256R1)
```

**Requirements:**
- HMAC key must be burned in eFuse with `ESP_EFUSE_KEY_PURPOSE_HMAC_UP` purpose
- Hardware HMAC peripheral (ESP32-C3, ESP32-S2, ESP32-S3, ESP32-C6, ESP32-H2)

**Key derivation parameters:**
- Algorithm: PBKDF2-HMAC-SHA256
- Iterations: 2048 (fixed)
- Output: 32-byte raw ECDSA key (converted to 121-byte DER format)
- Curve: SECP256R1 (P-256)

### Error Codes

Write-specific error codes are defined in `esp_secure_cert_write_errors.h`:

| Error Code | Description |
|------------|-------------|
| `ESP_ERR_SECURE_CERT_TLV_ALREADY_EXISTS` | TLV entry with same type/subtype exists |
| `ESP_ERR_SECURE_CERT_PARTITION_NOT_FOUND` | esp_secure_cert partition not found |
| `ESP_ERR_SECURE_CERT_FLASH_NOT_ERASED` | Flash area not erased (when check enabled) |
| `ESP_ERR_SECURE_CERT_BUFFER_OVERFLOW` | Buffer too small for data |
| `ESP_ERR_SECURE_CERT_WRITE_NO_MEMORY` | Memory allocation failed |
| `ESP_ERR_SECURE_CERT_HMAC_KEY_NOT_FOUND` | HMAC_UP key not found in eFuse |

See `esp_secure_cert_write_errors.h` for the complete list of error codes.

### TLV Types for Custom Data

Use these types for storing custom application data:

| Type | Value | Description |
|------|-------|-------------|
| `ESP_SECURE_CERT_USER_DATA_1` | 51 | User custom data slot 1 |
| `ESP_SECURE_CERT_USER_DATA_2` | 52 | User custom data slot 2 |
| `ESP_SECURE_CERT_USER_DATA_3` | 53 | User custom data slot 3 |
| `ESP_SECURE_CERT_USER_DATA_4` | 54 | User custom data slot 4 |
| `ESP_SECURE_CERT_USER_DATA_5` | 55 | User custom data slot 5 |

Each type supports subtypes 0-254 for multiple entries of the same type.
