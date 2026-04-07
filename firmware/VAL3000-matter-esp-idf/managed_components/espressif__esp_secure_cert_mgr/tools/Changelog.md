# This file contains the list of changes across different versions

## 2.5.1
- Fixed the issue, where `configure_esp_secure_cert.py` tool unable to generate DS context using already burned efuse key on device
when port is provided.
- Added support for `ECDSA_KEY_P256` efuse purpose.

## 2.5.0
### Changed
- Minimum Python version requirement updated from 3.7 to 3.8
- Added support for Python 3.11, 3.12, and 3.13
- Replaced deprecated cryptography `default_backend()` parameter usage
- Fixed private API usage (`rsa._modinv`) with standard `pow()` function for modular inverse
- Updated GitHub Actions workflow to use latest Ubuntu and action versions

### Removed
- Dropped support for Python 3.7

## 2.4.0
* Added feature in the `configure_esp_secure_cert.py` tool to secure-sign the esp-secure-cert partition binary.
* It signs the binary while generating new partition as well as to existing partition.

## 2.3.7
* Fixed the `configure_esp_secure_cert.py` tool for custom data as file.
* Unable to add the TLV entry, regression caused due TLV parser MR.

## 2.3.6
* Fixed Python 3.7 compatibility by replacing union type operator (|) with typing.Union

## 2.3.5
* Fixed package name to comply with PEP 625 (changed from `esp-secure-cert-tool` to `esp_secure_cert_tool`)

## 2.3.4
* Fixed regression caused due to `tlv_parser` import 

## 2.3.3
* Unpinned the esptool version to avoid dependency issues 

## 2.3.2
* The `configure_esp_secure_cert.py` tool can generate the `esp_secure_cert.bin` file using an `esp_secure_cert.csv` file as input.
* The tool can also parse an existing `esp_secure_cert.bin` file to extract the partition contents and generate a corresponding CSV file, as well as output the partition data in an organized manner.
