#!/usr/bin/env python
# SPDX-FileCopyrightText: 2020-2022 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0
import argparse
import os
import subprocess
import sys
import csv
from esp_secure_cert import nvs_format, custflash_format
from esp_secure_cert import tlv_format
from esp_secure_cert.tlv_format_construct import (
    EspSecureCert
)
from esp_secure_cert.efuse_helper import (
    log_efuse_summary,
)

# Check python version is proper or not to avoid script failure
assert sys.version_info >= (3, 8, 0), 'Python version too low. Python 3.8 or higher is required.'

esp_secure_cert_data_dir = 'esp_secure_cert_data'
# hmac_key_file is generated when HMAC_KEY is calculated,
# it is used when burning HMAC_KEY to efuse
hmac_key_file = os.path.join(esp_secure_cert_data_dir, 'hmac_key.bin')
ecdsa_key_file = os.path.join(esp_secure_cert_data_dir, 'ecdsa_key.bin')
# csv and bin filenames are default filenames
# for nvs partition files created with this script
csv_filename = os.path.join(esp_secure_cert_data_dir, 'esp_secure_cert.csv')
bin_filename = os.path.join(esp_secure_cert_data_dir, 'esp_secure_cert.bin')
# Targets supported by the script
supported_targets = {'esp32', 'esp32s2', 'esp32c3', 'esp32s3',
                     'esp32c6', 'esp32h2', 'esp32p4', 'esp32c5'}

def cleanup(args):
    if args.keep_ds_data is False:
        if os.path.exists(hmac_key_file):
            os.remove(hmac_key_file)
        if os.path.exists(csv_filename):
            os.remove(csv_filename)


def main():
    parser = argparse.ArgumentParser(description='''
    The python utility helps to configure and provision
    the device with PKI credentials, to generate the esp_secure_cert partition.
    The utility also configures the DS peripheral on the SoC if available.
    ''')

    parser.add_argument(
        '--private-key',
        dest='privkey',
        default=None,
        metavar='relative/path/to/client-priv-key',
        help='relative path to client private key')

    parser.add_argument(
        '--pwd', '--password',
        dest='priv_key_pass',
        metavar='[password]',
        help='the password associated with the private key')

    parser.add_argument(
        '--device-cert',
        dest='device_cert',
        default=None,
        metavar='relative/path/to/device-cert',
        help='relative path to device/client certificate '
             '(which contains the public part of the client private key) ')

    parser.add_argument(
        '--ca-cert',
        dest='ca_cert',
        default=None,
        metavar='relative/path/to/ca-cert',
        help='relative path to ca certificate which '
             'has been used to sign the client certificate')

    parser.add_argument(
        '--target_chip',
        dest='target_chip', type=str,
        choices=supported_targets,
        default='esp32c3',
        metavar='target chip',
        help='The target chip e.g. esp32s2, s3, c3')

    parser.add_argument(
        '--summary',
        dest='summary', action='store_true',
        help='Provide this option to print efuse summary of the chip')

    parser.add_argument(
        '--secure_cert_type',
        dest='sec_cert_type', type=str,
        choices={'cust_flash_tlv', 'cust_flash', 'nvs'},
        default='cust_flash_tlv',
        metavar='type of secure_cert partition',
        help='The type of esp_secure_cert partition. '
             'Can be \"cust_flash_tlv\" or \"cust_flash\" or \"nvs\". '
             'Please note that \"cust_flash\" and \"nvs\" are legacy formats.')

    parser.add_argument(
        '--configure_ds',
        dest='configure_ds', action='store_true',
        help='Provide this option to configure the DS peripheral.')

    parser.add_argument(
        '--skip_flash',
        dest='skip_flash', action='store_true',
        help='Provide this option to skip flashing the'
             ' esp_secure_cert partition at the value'
             ' provided to sec_cert_part_offset option')

    parser.add_argument(
        '--efuse_key_id',
        dest='efuse_key_id', type=int, choices=range(0, 6),
        metavar='[key_id] ',
        default=None,
        help='Provide the efuse key_id which '
             'contains/will contain HMAC_KEY, default is 1')

    parser.add_argument(
        '--efuse_key_file',
        help='eFuse key file which contains the key that shall be burned in the eFuse (for HMAC key only, not for ECDSA key).',
        metavar='[/path/to/efuse key file]')

    parser.add_argument(
        '--port', '-p',
        dest='port',
        metavar='[port]',
        required=False,
        help='UART com port to which the ESP device is connected')

    parser.add_argument(
        '--keep_ds_data_on_host', '-keep_ds_data',
        dest='keep_ds_data', action='store_true',
        help='Keep encrypted private key data and key '
             'on host machine for testing purpose')

    parser.add_argument(
        '--sec_cert_part_offset',
        dest='sec_cert_part_offset',
        default='0xD000',
        help='The flash offset of esp_secure_cert partition'
             ' Hex value must be given e.g. 0xD000')

    parser.add_argument(
        '--priv_key_algo',
        help='Signing algorithm used by the private key '
             ', e.g. RSA 2048, ECDSA 256',
        nargs=2, required=False,
        metavar='[sign algorithm, key size]')

    parser.add_argument(
        '--esp_secure_cert_csv',
        dest='esp_secure_cert_csv',
        metavar='[/path/to/esp_secure_cert_config.csv]',
        help='CSV file containing ESP Secure Cert contents (TLV entries, custom data). ')

    parser.add_argument(
        '--parse_bin',
        dest='parse_bin',
        metavar='[/path/to/esp_secure_cert.bin]',
        help='Parse an esp_secure_cert.bin file and generate CSV with extracted certificates/keys')

    parser.add_argument(
        '--secure-sign',
        dest='secure_sign',
        action='store_true',
        help='Enable secure boot')

    parser.add_argument(
        '--signing-scheme',
        dest='signing_scheme',
        choices=['rsa3072', 'ecdsa192', 'ecdsa256', 'ecdsa384'],
        default='rsa3072',
        help='Secure boot scheme to use')

    parser.add_argument(
        '--signing-key-file',
        dest='signing_key_file',
        default="secure_boot_key.bin",
        nargs='+',
        help='Signing key file(s) to use. You can provide multiple files separated by space.')

    parser.add_argument(
        '--esp-secure-cert-file',
        dest='bin_filename',
        default=None,
        help='Bin filename to use')

    args = parser.parse_args()

    idf_target = args.target_chip
    if idf_target not in supported_targets:
        if idf_target is not None:
            print('ERROR: The script does not support '
                  'the target {}'.format(idf_target))
        sys.exit(-1)
    idf_target = str(idf_target)

    if args.summary is not False:
        log_efuse_summary(idf_target, args.port)
        sys.exit(0)

    if args.parse_bin:
        EspSecureCert.parse_esp_secure_cert_bin(args.parse_bin)
        return

    if (args.privkey is not None and os.path.exists(args.privkey) is False):
        print('ERROR: The provided private key file does not exist')
        sys.exit(-1)

    if (args.device_cert is not None and os.path.exists(args.device_cert) is False):
        print('ERROR: The provided client cert file does not exist')
        sys.exit(-1)

    if (args.ca_cert is not None and os.path.exists(args.ca_cert) is False):
        print('ERROR: The provided ca cert file does not exist')
        sys.exit(-1)

    if (os.path.exists(esp_secure_cert_data_dir) is False):
        os.makedirs(esp_secure_cert_data_dir)

    c = None
    iv = None
    key_size = None

    if args.sec_cert_type == 'cust_flash_tlv':
        # Create instance of EspSecureCert
        esp_secure_cert = EspSecureCert()

        # Create entry for CA certificate (if provided)

        if args.ca_cert is not None:
            entry_ca = {
                'tlv_type': tlv_format.tlv_type_t.ESP_SECURE_CERT_CA_CERT_TLV,
                'tlv_subtype': 0,
                'data_value': os.path.abspath(args.ca_cert),
                'data_type': 'file',
            }
            esp_secure_cert.add_entry(entry_ca)

        # Create entry for device certificate
        if args.device_cert is not None:
            entry_dev = {
                'tlv_type': tlv_format.tlv_type_t.ESP_SECURE_CERT_DEV_CERT_TLV,
                'tlv_subtype': 0,
                'data_value': os.path.abspath(args.device_cert),
                'data_type': 'file',
            }
            esp_secure_cert.add_entry(entry_dev)

        # Create entry for private key
        priv_key_type = 'plaintext'
        if args.configure_ds is not False:
            if args.priv_key_algo[0] == 'RSA':
                priv_key_type = 'rsa_ds'
            elif args.priv_key_algo[0] == 'ECDSA':
                priv_key_type = 'ecdsa_peripheral'

        if args.privkey is not None:
            entry_priv = {
                'tlv_type': tlv_format.tlv_type_t.ESP_SECURE_CERT_PRIV_KEY_TLV,
                'tlv_subtype': 0,
                'data_value': os.path.abspath(args.privkey),
                'data_type': 'file',
                'priv_key_type': priv_key_type,
                'algorithm': args.priv_key_algo[0].upper() if args.priv_key_algo else '',
                'key_size': int(args.priv_key_algo[1]) if args.priv_key_algo and len(args.priv_key_algo) > 1 else 0,
                'efuse_id': args.efuse_key_id if hasattr(args, 'efuse_key_id') else 0,
                'efuse_key_file': args.efuse_key_file if hasattr(args, 'efuse_key_file') else None, # For HMAC key only, not for ECDSA key
            }
            esp_secure_cert.add_entry(entry_priv)

        if args.bin_filename is not None and args.secure_sign:
            if args.signing_scheme is None:
                raise ValueError("Signing scheme is not set")

            signed_bin_filename = esp_secure_cert.add_signature_block_using_existing_key(os.path.abspath(args.bin_filename), args.signing_key_file, args.signing_scheme)

            if not args.skip_flash:
                esp_secure_cert.flash_esp_secure_cert_partition(args.target_chip, args.port, args.sec_cert_part_offset, signed_bin_filename)
            else:
                print(f'To flash manually: esptool.py --chip {args.target_chip} -p {args.port} write_flash {args.sec_cert_part_offset} {signed_bin_filename}')

            return

        if args.esp_secure_cert_csv is not None:
            esp_secure_cert.parse_esp_secure_cert_csv(args.esp_secure_cert_csv)

        bin_filename = esp_secure_cert.generate_esp_secure_cert(args.target_chip, args.port)

        if args.secure_sign:
            # Set the secure boot scheme
            bin_filename = esp_secure_cert.add_signature_block_using_existing_key(bin_filename, args.signing_key_file, args.signing_scheme)

        if not args.skip_flash:
            esp_secure_cert.flash_esp_secure_cert_partition(args.target_chip, args.port, args.sec_cert_part_offset, bin_filename)
        else:
            if args.port:
                print(f'To flash manually: esptool.py --chip {args.target_chip} -p {args.port} write_flash {args.sec_cert_part_offset} {bin_filename}')
            else:
                print(f'To flash manually: esptool.py --chip {args.target_chip} -p <PORT> write_flash {args.sec_cert_part_offset} {bin_filename}')

        esp_secure_cert.esp_secure_cert_cleanup()

        return

    elif args.sec_cert_type == 'cust_flash':
        if args.configure_ds is not False:
            custflash_format.generate_partition_ds(c, iv, args.efuse_key_id,
                                                   key_size, args.device_cert,
                                                   args.ca_cert, idf_target,
                                                   bin_filename)
        else:
            custflash_format.generate_partition_no_ds(args.device_cert,
                                                      args.ca_cert,
                                                      args.privkey,
                                                      args.priv_key_pass,
                                                      idf_target, bin_filename)
    elif args.sec_cert_type == 'nvs':
        # Generate csv file for the DS data and generate an NVS partition.
        if args.configure_ds is not False:
            nvs_format.generate_csv_file_ds(c, iv, args.efuse_key_id,
                                            key_size, args.device_cert,
                                            args.ca_cert, csv_filename)
        else:
            nvs_format.generate_csv_file_no_ds(args.device_cert, args.ca_cert,
                                               args.privkey,
                                               args.priv_key_pass,
                                               csv_filename)
        nvs_format.generate_partition(csv_filename, bin_filename)

    if args.skip_flash is False:
        flash_esp_secure_cert_partition(idf_target,
                                        args.port,
                                        args.sec_cert_part_offset,
                                        bin_filename)

    cleanup(args)

if __name__ == '__main__':
    main()
