# esp_secure_cert Configuration Tool
> WARNING: This tool is to be used only for development purpose. It does not enable any kind security feature for the protection of the sensitive data in the `esp_secure_cert` partition.

The script [configure_esp_secure_cert.py](https://github.com/espressif/esp_secure_cert_mgr/blob/main/tools/configure_esp_secure_cert.py) is used for configuring the ESP platform with PKI credentials into the esp_secure_cert partition which shall reside on its flash storage.
It also configures the DS peripheral on the ESP32-S2/ESP32-S3/ESP32-C3 SoC. The steps in the script are based on technical details of certain operations in the Digital Signature calculation, which can be found in the Digital Signature Section of [ESP32-S2 TRM](https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf).

The script generates a partition named `esp_secure_cert` on host machine, that contains the parameters required by the DS peripheral. The `configure_esp_secure_cert.py` utility automatically flashes the partition to the ESP platform which has been connected.

# Installation

```
pip install esp-secure-cert-tool
```

# Configuration

1) Generate root ca and key pair:
```
openssl req -newkey rsa:2048 -nodes -keyout prvtkey.pem -x509 -days 3650 -out cacert.pem -subj "/CN=Test CA"
```

2) Generate client private key:
```
openssl genrsa -out client.key
```

3) Generate device cert:
```
openssl req -out client.csr -key client.key -new
openssl x509 -req -days 365 -in client.csr -CA cacert.pem -CAkey prvtkey.pem  -sha256 -CAcreateserial -out client.crt
```

# Generate `esp_secure_cert` partition

## 1. Generating `esp_secure_cert` partition using CSV file

You can create an `esp_secure_cert` partition using a CSV file that lists all certificates, keys, and custom data you want to include. The CSV format makes it easy to specify what goes into the partition, including support for advanced features like DS/ECDSA peripherals and custom TLV entries.

For details on the CSV format and examples, see [docs/esp_secure_cert_tools/configure_esp_secure_cert_csv.md](../docs/esp_secure_cert_tools/configure_esp_secure_cert_csv.md).

> **NOTE:** Before creating the `esp_secure_cert` partition on actual hardware, it is recommended to first test your configuration and process using `QEMU` (the ESP32 emulator). This allows you to validate your CSV, partition generation, and flashing workflow in a safe environment before applying changes to real hardware.

## 2. Creating `esp_secure_cert` partition without using CSV:

Following commands can be used to configure the DS peripheral and generate the `esp_secure_cert` partition.
The script can generate `cust_flash` as well as `nvs` type of `esp_secure_cert` partition. Please refer [upper level README](../README.md) for more details about type of partitions.

* When configuring the DS peripheral, by default the configuration script does not enable the read protection for the efuse key block in which the DS key is programmed. This is done for allowing flexibility while using the script for development purpose.

* Please remove the `--configure_ds` argument from these commands if use of the DS peripheral is disabled in the menu config.
> **WARNING**: This is not recommended for production purpose as the private key shall be stored as plaintext.

### Generate `esp_secure_cert` partition of type `cust_flash_tlv`:

This command shall generate a binary partition containing the PKI credentials stored in the TLV format and flash it at the default offset of `0xD000`.

```
configure_esp_secure_cert.py -p /* Serial port */ --keep_ds_data_on_host --efuse_key_id 1 --ca-cert cacert.pem --device-cert client.crt --private-key client.key --target_chip /* target chip */ --secure_cert_type cust_flash_tlv --configure_ds
```

* When [Flash Encryption](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/security/flash-encryption.html) is enabled for the device, the option ``--skip_flash`` (explained below) can be used to prevent the flashing opereation and only save the `esp_secure_cert.bin` on the host machine. It can then be flashed on the target using below command:

	```esptool.py -p /* Serial Port*/ write_flash 0xD000 esp_secure_cert.bin --encrypt```

	More details regarding [esptool.py](https://docs.espressif.com/projects/esptool/en/latest/esp32/esptool/index.html#esptool-py) utility can be found [here](https://docs.espressif.com/projects/esptool/en/latest/esp32/esptool/index.html).

    Note: This is only applicable for [Development mode](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/security/flash-encryption.html#flash-enc-development-mode) of Flash Encryption.

### Legacy partition formats:

1. Generate `esp_secure_cert` partition of type `cust_flash`:

```
configure_esp_secure_cert.py -p /* Serial port */ --keep_ds_data_on_host --efuse_key_id 1 --ca-cert cacert.pem --device-cert client.crt --private-key client.key --target_chip /* target chip */ --secure_cert_type cust_flash --configure_ds
```

2. Generate `esp_secure_cert` partition of type `nvs`:
```
configure_esp_secure_cert.py -p /* Serial port */ --keep_ds_data_on_host --efuse_key_id 1 --ca-cert cacert.pem --device-cert client.crt --private-key client.key --target_chip /* target chip */ --secure_cert_type nvs --configure_ds
```

# Test `esp_secure_cert` partition using QEMU

QEMU is a free, open-source emulator that lets you run and test software for different hardware platforms on your computer, without needing the actual device. Currently, QEMU supports ESP32, ESP32C3, and ESP32S3. For more information, refer to https://github.com/espressif/esp-toolchain-docs/blob/main/README.md

1. Set the target:
```
cd examples/esp_secure_cert_app
idf.py set-target <target>
```

2. Modify the sdkconfig file according to requirement and build the application:
```
idf.py build
```

3. Create the `qemu_flash.bin` and `qemu_efuse.bin`, which emulate the application and efuse:
```
cd build
esptool.py --chip esp32c3 merge_bin --fill-flash-size 4MB -o flash_image.bin @flash_args
```

- For efuse, create efuse, create `efuse.hex` and copy the following data into it:
```
000000000000000000000000000000000000000000000000000000000000
00000000000000000c000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
000000000000000000000000000000000000000000000000000000000000
00000000
```
- Convert this `.hex` into `.bin` using `xxd -p`
```
xxd -r -p efuse.hex qemu_efuse.bin
```


4. Start the `qemu` as server (Download mode), which listen on TCP port `socket://localhost:5555`:
```
qemu-system-riscv32 -nographic \
    -machine esp32c3 \
    -drive file=flash_image.bin,if=mtd,format=raw \
    -global driver=esp32c3.gpio,property=strap_mode,value=0x02 \
    -drive file=qemu_efuse.bin,if=none,format=raw,id=efuse \
    -global driver=nvram.esp32c3.efuse,property=drive,value=efuse \
    -serial tcp::5555,server,nowait
```

5. Create the `esp_secure_cert.bin` file:
```
../../tools/configure_esp_secure_cert.py --esp_secure_cert_csv esp_secure_cert_config_examples.csv --port socket://localhost:5555 --target_chip esp32c3 --skip_flash
```
- If required then `BURN` the efuse.

6. Flash the `esp_secure_cert.bin` on device with TCP port `socket://localhost:5555`
```
esptool.py --before no_reset --no-stub --after no_reset --chip esp32c3 -p socket://localhost:5555 write_flash 0xD000 esp_secure_cert_data/esp_secure_cert.bin
```

7. Exit from the `Download Mode` and start with `Boot Mode`:
```
qemu-system-riscv32 -nographic \
    -machine esp32c3 \
    -drive file=flash_image.bin,if=mtd,format=raw \
    -global driver=esp32c3.gpio,property=strap_mode,value=0x08 \
    -drive file=qemu_efuse.bin,if=none,format=raw,id=efuse \
    -global driver=nvram.esp32c3.efuse,property=drive,value=efuse \
```

## Additional options for the utility
The following options can be provided additional to the main arguments given above.

### Set the `esp_secure_cert` partition offset
By default the `esp_secure_cert` partition shall be flashed at an offset of `0xD000` by the utility.
In order to flash the `esp_secure_cert` partition at a different offset, the following argument can be provided additionally to the configure_esp_secure_cert.py script.

```
configure_esp_secure_cert.py  --sec_cert_part_offset /* offset value in hex e.g. 0xD000 */
```
### Skip automatic flashing of `esp_secure_cert` partition

By default the `esp_secure_cert` partition shall be flashed automatically at the offset value provided to `sec_cert_part_offset` argument.
In order to stop the flashing process the following argument can be provided additionally to the configure_esp_secure_cert.py script

```
configure_esp_secure_cert.py --skip_flash
```

### Help section
The additional options supported by the utility can be found at `configure_esp_secure_cert.py --help`.

# Parsing the `esp_secure_cert.bin` file

You can parse the generated `esp_secure_cert.bin` file using the `configure_esp_secure_cert.py` tool. This allows you to extract and inspect the contents of the partition, including certificates, private keys, and any custom data stored within it.

- The tool can also generate a CSV file from the parsed `.bin`, which can be useful for backup, migration, or auditing purposes.
- All data fields present in the partition (such as certificates, keys, and user data) will be included in the output CSV.
- For more details on the CSV format and how to use it, refer to [docs/esp_secure_cert_tools/configure_esp_secure_cert_parser.md](https://github.com/espressif/esp_secure_cert_mgr/blob/main/docs/esp_secure_cert_tools/configure_esp_secure_cert_parser.md).
