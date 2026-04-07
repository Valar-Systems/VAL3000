# ESP Secure Certificate Application

The sample app demonstrates the use of APIs from *esp_secure_cert_mgr* to:
- **Read** the contents of the *esp_secure_cert* partition (certificates, keys)
- **Write** custom data to the partition (optional demo)
- **Verify** the validity of the contents from the partition

## Requirements
* The device must be pre-provisioned and have an *esp_secure_cert* partition (for read operations)
* For write demo: An erased or writable *esp_secure_cert* partition

## How to use the example
Before project configuration and build, be sure to set the correct chip target using `idf.py set-target <chip_name>`.
### Configure the project

* The *esp_secure_cert* partition needs to be generated and flashed first with help of [configure_esp_secure_cert.py](https://github.com/espressif/esp_secure_cert_mgr/blob/main/tools/configure_esp_secure_cert.py) script. See [tools/README.md](https://github.com/espressif/esp_secure_cert_mgr/blob/main/tools/README.md) for more details.

* Please ensure that appropriate type of esp_secure_cert partition has been set in your projects `partitions.csv` file. Please refer the "esp_secure_cert partition" section in the [component README](https://github.com/espressif/esp_secure_cert_mgr#readme) for more details.

### Enable Write Demo (Optional)

To enable the write functionality demonstration:

```bash
idf.py menuconfig
```

Navigate to: **Example Configuration** → **Enable write functionality demo**

When enabled, the app will:
1. Read and display existing partition data (certificates, keys)
2. Back up the original partition contents to RAM
3. Erase the partition, write a sample TLV entry, read back and verify
4. Restore the original partition contents from backup

Pre-provisioned data is preserved across demo runs.

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view the serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

### Example Output (Read Mode)
```
I (331) esp_secure_cert_app: Device Cert:
Length: 1233
-----BEGIN CERTIFICATE-----
.
.
-----END CERTIFICATE-----

I (441) esp_secure_cert_app: CA Cert:
Length: 1285
-----BEGIN CERTIFICATE-----
.
.
-----END CERTIFICATE-----

I (561) esp_secure_cert_app: Successfuly obtained ciphertext, ciphertext length is 1200
I (571) esp_secure_cert_app: Successfuly obtained initialization vector, iv length is 16
I (571) esp_secure_cert_app: RSA length is 2048
I (581) esp_secure_cert_app: Efuse key id 1
I (581) esp_secure_cert_app: Successfully obtained the ds context
I (831) esp_secure_cert_app: Ciphertext validated succcessfully
```

### Example Output (Write Demo Enabled)
```
I (331) esp_secure_cert_app: Starting ESP Secure Cert App
W (331) esp_secure_cert_app: Write demo enabled - this will modify the esp_secure_cert partition!
I (341) esp_secure_cert_app: === Write Demo: Basic TLV write/read ===
I (351) esp_secure_cert_app: Partition erased
I (361) esp_secure_cert_app: Wrote user data TLV
I (371) esp_secure_cert_app: Read back: "Hello from esp_secure_cert write demo!" (len=40)
I (381) esp_secure_cert_app: Write demo completed successfully
```

## Additional configurations for `pre_prov` partition
Few of the modules which were pre-provisioned initially had the name of the pre-provisioning partition as `pre_prov`. For the modules which have pre-provisioning partition of name `esp_secure_cert` this part can be ignored.

* For modules with `pre_prov` partition of type *cust_flash*, please update the line refering to `esp_secure_cert` partition in the partitions.csv with following: 
```
pre_prov,         0x3F,          ,    0xD000,     0x6000,
```
* No change is necessary for `pre_prov` partition of type *nvs*.
