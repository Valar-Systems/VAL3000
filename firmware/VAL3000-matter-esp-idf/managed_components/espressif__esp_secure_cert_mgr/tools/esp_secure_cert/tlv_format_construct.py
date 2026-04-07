#!/usr/bin/env python3
"""
Enhanced TLV Format Generator using Construct Library
Provides improved readability, endianness handling, and custom TLV support
"""

import enum
import csv
import os
import sys
import io
import zlib
import struct
import base64
import shutil
import subprocess
from typing import Dict, List, Tuple, Any, Union
from construct import (Struct, Int32ul, Int8ul, Int16ul, Int32sl, Bytes, this, GreedyBytes)
import tempfile
try:
    from espsecure import (
        _get_sbv2_pub_key,
        _sha256_digest,
        _sha384_digest,
        _get_sbv2_rsa_primitives,
        _microecc_format,
        sign_data,
        generate_signature_block_using_private_key,
    )
except ImportError:
    raise ImportError("espsecure module not available")


from esp_secure_cert.esp_secure_cert_helper import (
    load_private_key,
    load_certificate,
    get_efuse_key_file,
)

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa, ec
from cryptography.utils import int_to_bytes
from esp_secure_cert import configure_ds, tlv_format, tlv_parser
from esp_secure_cert.tlv_format import tlv_type_t, tlv_priv_key_type_t


class TlvSubtype(enum.IntEnum):
    SUBTYPE_0 = 0
    SUBTYPE_1 = 1
    SUBTYPE_2 = 2
    SUBTYPE_3 = 3
    SUBTYPE_4 = 4
    SUBTYPE_5 = 5
    SUBTYPE_MAX = 254
    SUBTYPE_INVALID = 255


# Constants
MIN_ALIGNMENT_REQUIRED = 16
TLV_MAGIC = 0xBA5EBA11
PARTITION_SIZE = 0x2000  # 8KB
MAGIC_END = 0xFFFF

SECURE_VERIFICATION_VERSION_1 = 0x01

ECDSA_SIG_BLOCK_MAGIC = 0xE7

RSA3072_ALGORITHM = 0
ECDSA192_ALGORITHM = 1
ECDSA256_ALGORITHM = 2
ECDSA384_ALGORITHM = 3

esp_secure_cert_data_dir = 'esp_secure_cert_data'
# hmac_key_file is generated when HMAC_KEY is calculated,
# it is used when burning HMAC_KEY to efuse
hmac_key_file = os.path.join(esp_secure_cert_data_dir, 'hmac_key.bin')
ecdsa_key_file = os.path.join(esp_secure_cert_data_dir, 'ecdsa_key.bin')
# csv and bin filenames are default filenames
# for nvs partition files created with this script
csv_filename = os.path.join(esp_secure_cert_data_dir, 'esp_secure_cert.csv')

# Construct structures with proper endianness
TlvHeader = Struct(
    "magic" / Int32ul,  # Little-endian 32-bit unsigned int
    "flags" / Int8ul,   # 8-bit flags
    "reserved" / Bytes(3),  # 3 reserved bytes
    "type" / Int8ul,     # TLV type
    "subtype" / Int8ul,  # TLV subtype
    "length" / Int16ul,  # Little-endian 16-bit length
)

TlvFooter = Struct(
    "crc" / Int32ul,    # Little-endian 32-bit CRC
)

EspSecCertSigRSA = Struct(
    "version" / Int8ul,           # Signature block version
    "reserved1" / Bytes(3),       # Reserved for future use
    "offset" / Int32ul,           # Offset for hash calculation
    "length" / Int32ul,           # Length of hashed data
    "algorithm" / Int8ul,         # Algorithm: 0=RSA3072, 1=ECDSA192, 2=ECDSA256, 3=ECDSA384
    "reserved2" / Bytes(3),       # Reserved for future use
    "rsa_signature_block" / GreedyBytes,    # RSA signature
)

EspSecCertSigECDSA = Struct(
    "version" / Int8ul,           # Signature block version
    "reserved1" / Bytes(3),       # Reserved for future use
    "offset" / Int32ul,           # Offset for hash calculation
    "length" / Int32ul,           # Length of hashed data
    "algorithm" / Int8ul,         # Algorithm: 0=RSA3072, 1=ECDSA192, 2=ECDSA256, 3=ECDSA384
    "reserved2" / Bytes(3),       # Reserved for future use
    "public_key" / Bytes(lambda ctx: 49 if ctx.algorithm == 1 else (65 if ctx.algorithm == 2 else 97)),  # curve_id(1) + key_size
    "signature" / Bytes(lambda ctx: 48 if ctx.algorithm == 1 else (64 if ctx.algorithm == 2 else 96)),  # Signature size
)

TlvEntry = Struct(
    "header" / TlvHeader,
    "data" / Bytes(this.header.length),
    "padding" / Bytes(lambda ctx: _calculate_padding(ctx.header.length)),
    "footer" / TlvFooter,
)


def _calculate_padding(data_length: int) -> int:
    """Calculate padding needed for 16-byte alignment"""
    return (MIN_ALIGNMENT_REQUIRED - (data_length % MIN_ALIGNMENT_REQUIRED)) % MIN_ALIGNMENT_REQUIRED


def _get_flag_byte(key_type: tlv_priv_key_type_t) -> int:
    """Generate flag byte based on private key type"""
    flags = 0
    if key_type == tlv_priv_key_type_t.ESP_SECURE_CERT_HMAC_ENCRYPTED_KEY:
        flags |= (1 << 7)  # bit 7
    elif key_type == tlv_priv_key_type_t.ESP_SECURE_CERT_HMAC_DERIVED_ECDSA_KEY:
        flags |= (1 << 6)  # bit 6
    elif key_type == tlv_priv_key_type_t.ESP_SECURE_CERT_ECDSA_PERIPHERAL_KEY:
        flags |= (1 << 3)  # bit 3
    return flags


class TlvPartitionBuilder:
    """Builder class for creating TLV partitions with custom data support"""

    def __init__(self):
        self.entries: List[Dict] = []
        self.partition_data = bytearray(b'\xff' * PARTITION_SIZE)
        self.current_offset = 0

    def add_certificate(self, tlv_type: tlv_type_t, cert_path: Union[str, bytes], subtype: int = 0) -> None:
        """Add certificate to partition"""
        cert_data = load_certificate(cert_path)

        # Add null terminator for PEM certificates
        if cert_data["encoding"] == serialization.Encoding.PEM.value:
            cert_path = cert_data["bytes"] + b'\0'
        else:
            cert_path = cert_data["bytes"]
        self.add_tlv_entry(tlv_type, subtype, cert_path, 0)

    def add_private_key(self, key_path: str, key_pass: Any = None,
                        key_type: tlv_priv_key_type_t = tlv_priv_key_type_t.ESP_SECURE_CERT_DEFAULT_FORMAT_KEY,
                        subtype: int = 0) -> None:
        """Add private key to partition"""
        if key_path and os.path.exists(key_path):
            key_data = load_private_key(key_path, key_pass)

            if key_data["encoding"] == serialization.Encoding.PEM.value:
                key_bytes = key_data["bytes"] + b'\0'
            else:
                key_bytes = key_data["bytes"]
        else:
            key_bytes = b''  # Empty for peripheral keys

        flags = _get_flag_byte(key_type)
        self.add_tlv_entry(tlv_type_t.ESP_SECURE_CERT_PRIV_KEY_TLV, subtype, key_bytes, flags)

    def add_ds_data(self, ciphertext: bytes, iv: bytes, rsa_key_len: int, subtype: int = 0) -> None:
        """Add DS data for RSA hardware acceleration"""
        # Create DS data structure: [key_len_param][iv][ciphertext]
        key_len_param = Int32sl.build(rsa_key_len // 32 - 1)  # Signed little-endian
        ds_data = key_len_param + iv + ciphertext

        self.add_tlv_entry(tlv_type_t.ESP_SECURE_CERT_DS_DATA_TLV, subtype, ds_data, 0)

    def add_ds_context(self, efuse_key_id: int, rsa_key_len: int, subtype: int = 0) -> None:
        """Add DS context for hardware acceleration"""
        # DS context structure must match exactly what the legacy code generated
        # From legacy code in tlv_format.py:
        # ds_context = struct.pack('<I', 0)                          # reserved:4 (32-bit little-endian)
        # ds_context = ds_context + struct.pack('<B', efuse_key_id)   # efuse_id:1 (8-bit)
        # ds_context = ds_context + struct.pack('<B', 0)             # padding:1 (8-bit)
        # ds_context = ds_context + struct.pack('<H', rsa_key_len)   # key_len:2 (16-bit little-endian)
        ds_context = Struct(
            "reserved" / Int32ul,      # 4 bytes reserved (32-bit unsigned little-endian)
            "efuse_key_id" / Int8ul,   # 1 byte efuse key ID (8-bit unsigned)
            "padding" / Int8ul,        # 1 byte padding (8-bit unsigned)
            "rsa_key_len" / Int16ul,   # 2 bytes RSA key length (16-bit unsigned little-endian)
        ).build({
            "reserved": 0,
            "efuse_key_id": efuse_key_id,
            "padding": 0,
            "rsa_key_len": rsa_key_len,
        })

        self.add_tlv_entry(tlv_type_t.ESP_SECURE_CERT_DS_CONTEXT_TLV, subtype, ds_context, 0)

    def add_security_config(self, efuse_key_id: int, subtype: int = 0) -> None:
        """Add security configuration"""
        # Security config: [priv_key_efuse_id:1][reserved:39]
        efuse_block_id = efuse_key_id + 4  # Convert to block ID
        sec_cfg = bytes([efuse_block_id]) + b'\x00' * 39

        self.add_tlv_entry(tlv_type_t.ESP_SECURE_CERT_SEC_CFG_TLV, subtype, sec_cfg, 0)

    def add_signature_block(self, algorithm: int, public_key: bytes, signature: bytes, subtype: int = 0, partition_length: int = None) -> None:
        """
        Add optimized signature block with custom ESP Secure Cert structure
        Structure matches firmware: version + reserved1(3) + offset + length + algorithm + reserved2(3) + key_data + signature

        For RSA: key_data is ets_rsa_pubkey_t structure
        For ECDSA: key_data is X||Y coordinates, signature is r||s

        Structure matches esp_secure_cert_signature_t in esp_secure_cert_signature_verify.c:
        - uint8_t version
        - uint8_t reserved1[3]
        - uint32_t offset
        - uint32_t length
        - uint8_t algorithm
        - uint8_t reserved2[3]
        - public_key + signature
        """
        # Calculate offset (always 0 for partition start) and length
        offset = 0
        # Use provided partition_length or current offset (for backward compatibility)
        length = partition_length if partition_length is not None else self.current_offset

        if algorithm == RSA3072_ALGORITHM:
            sig_block_data = EspSecCertSigRSA.build({
                "version": SECURE_VERIFICATION_VERSION_1,
                "reserved1": b'\x00' * 3,
                "offset": offset,
                "length": length,
                "algorithm": algorithm,
                "reserved2": b'\x00' * 3,
                "rsa_signature_block": signature,
            })
        else:
            sig_block_data = EspSecCertSigECDSA.build({
                "version": SECURE_VERIFICATION_VERSION_1,
                "reserved1": b'\x00' * 3,
                "offset": offset,
                "length": length,
                "algorithm": algorithm,
                "reserved2": b'\x00' * 3,
                "public_key": public_key,
                "signature": signature,
            })
        self.add_tlv_entry(tlv_type_t.ESP_SECURE_CERT_SIGNATURE_BLOCK_TLV, subtype, sig_block_data, 0)

    def add_tlv_entry(self, tlv_type: Union[int, tlv_type_t], subtype: int,
                      data: bytes, flags: int) -> None:
        """Internal method to add TLV entry to partition"""
        if isinstance(tlv_type, tlv_type_t):
            tlv_type = tlv_type.value

        # Build TLV entry using construct
        tlv_data = {
            "header": {
                "magic": TLV_MAGIC,
                "flags": flags,
                "reserved": b'\x00\x00\x00',
                "type": tlv_type,
                "subtype": subtype,
                "length": len(data),
            },
            "data": data,
            "footer": {
                "crc": 0,  # Will be calculated below
            }
        }

        # Build the TLV without footer first to calculate CRC
        # TlvHeader.build() serializes this to binary
        header_and_data = TlvHeader.build(tlv_data["header"]) + data
        padding_len = _calculate_padding(len(data))
        padding = b'\x00' * padding_len
        crc_data = header_and_data + padding

        # Calculate CRC32
        crc = zlib.crc32(crc_data, 0xffffffff) & 0xffffffff
        tlv_data["footer"]["crc"] = crc

        # Build complete TLV entry
        complete_tlv = crc_data + TlvFooter.build(tlv_data["footer"])

        # Add to partition
        if self.current_offset + len(complete_tlv) > PARTITION_SIZE:
            raise ValueError(f"Partition size exceeded: {self.current_offset + len(complete_tlv)} > {PARTITION_SIZE}")

        self.partition_data[self.current_offset:self.current_offset + len(complete_tlv)] = complete_tlv
        self.current_offset += len(complete_tlv)

        self.entries.append({
            "type": tlv_type,
            "subtype": subtype,
            "length": len(data),
            "flags": flags,
            "offset": self.current_offset - len(complete_tlv)
        })

        print(f"Added TLV entry: type={tlv_type}, subtype={subtype}, length={len(complete_tlv)}")

    def build_partition(self, output_file: str) -> None:
        """Write partition to file with proper TLV termination"""

        with open(output_file, 'wb') as f:
            f.write(self.partition_data[:self.current_offset])

        print(f"Total TLV entries: {len(self.entries)}")
        print(f"Total partition size used: {self.current_offset} / {PARTITION_SIZE} bytes")


class EspSecureCert:

    def __init__(self, version: str = "2", hash_type: str = "sha256"):
        # Initialize the list to store TLV entries
        self.secure_cert_entries = []
        self.version = "2"
        self.hash_type = hash_type
        self.keyfile_path = None
        self.signature_block_no = 0
        self.secure_boot_scheme = "rsa3072"  # Default secure boot scheme
        # Create the directory esp_secure_cert_data if it does not exist

        self.builder = TlvPartitionBuilder()

        if not os.path.exists(esp_secure_cert_data_dir):
            os.makedirs(esp_secure_cert_data_dir)

        self.bin_filename = os.path.join(esp_secure_cert_data_dir, "esp_secure_cert.bin")
        self.signed_bin_filename = os.path.join(esp_secure_cert_data_dir, "esp_secure_cert_signed.bin")

    def __del__(self):
        """Destructor - cleanup when object is destroyed"""
        self.esp_secure_cert_cleanup()

    def _is_private_key_entry(self, tlv_type: Union[int, tlv_type_t]) -> bool:
        """Check if entry is a private key"""
        return tlv_type == tlv_type_t.ESP_SECURE_CERT_PRIV_KEY_TLV.value

    def _is_certificate_entry(self, tlv_type: Union[int, tlv_type_t]) -> bool:
        """Check if entry is a certificate"""
        return tlv_type in [tlv_type_t.ESP_SECURE_CERT_CA_CERT_TLV.value, tlv_type_t.ESP_SECURE_CERT_DEV_CERT_TLV.value]

    def _validate_entry(self, entry: dict) -> bool:
        """Validate entry"""

        if entry.get('tlv_type') is None:
            raise ValueError("ERROR: Missing required 'tlv_type' in CSV")
        if entry.get('tlv_subtype') is None:
            raise ValueError("ERROR: Missing required 'tlv_subtype' in CSV")
        if not entry.get('data_value'):
            raise ValueError("ERROR: Missing required 'data_value' in CSV")
        if not entry.get('data_type'):
            raise ValueError("ERROR: Missing required 'data_type' in CSV")

        if not self._check_for_duplicate_tlv_entries(entry):
            raise ValueError("ERROR: Validation failed for duplicate entries")

        if self._is_private_key_entry(entry.get('tlv_type')):
            self._private_key_validation(entry)

        if (self._is_certificate_entry(entry.get('tlv_type')) or self._is_private_key_entry(entry.get('tlv_type'))) and entry.get('data_type') != 'file':
            entry['data_value'] = self._parse_data_from_any_format(entry['data_value'], entry['data_type'], True)
            entry['data_type'] = 'file'

        return True

    def _private_key_validation(self, entry: dict) -> bool:
        """Validate private key"""
        if entry.get('ds_enabled'):
            if entry.get('efuse_id') is None:
                raise ValueError("ERROR: Missing required 'efuse_id' in CSV")
        if entry.get('algorithm') is None:
            raise ValueError("ERROR: Missing required 'algorithm' in CSV")
        if entry.get('key_size') is None:
            raise ValueError("ERROR: Missing required 'key_size' in CSV")
        return True

    def _parse_data_from_any_format(self, data_value, data_type, output_file=False):
        """
        Process data content based on type:
        - 'file': return file path
        - 'content': write content to temp file and return path
        - 'string': return data as bytes (or temp file for certs/keys)
        - 'hex': return hex decoded bytes (or temp file for certs/keys)
        - 'base64': return base64 decoded bytes (or temp file for certs/keys)
        Returns: (processed_data, is_file_path)
        """
        # Check if this is a certificate or private key that needs file-based processing
        output_data = None

        if data_type == 'file':
            if not os.path.exists(data_value):
                raise FileNotFoundError(f"File not found: {data_value}")
            return data_value

        elif data_type == 'string':
            output_data = data_value.encode('utf-8')

        elif data_type == 'hex':
            output_data = bytes.fromhex(data_value.replace(' ', ''))

        elif data_type == 'base64':
            output_data = base64.b64decode(data_value)

        else:
            raise ValueError(f"Unsupported data_type: {data_type}")

        if output_file:
            temp_file = os.path.join(esp_secure_cert_data_dir, f'temp_key_{hash(str(data_value)) % 10000}.key')
            with open(temp_file, 'wb') as f:
                f.write(output_data)
            return temp_file
        return output_data

    def _check_for_duplicate_tlv_entries(self, entry):
        """Check for duplicate TLV entries (by tlv_type and tlv_subtype)"""

        # Check for duplicate subtypes within the same type
        # Check if the entry (by tlv_type and tlv_subtype) is already present in entries
        for existing_entry in self.secure_cert_entries:
            if (existing_entry.get('tlv_type') == entry.get('tlv_type') and
                    existing_entry.get('tlv_subtype') == entry.get('tlv_subtype')):
                print(f"WARNING: Duplicate entry found for type {entry.get('tlv_type')}, subtype {entry.get('tlv_subtype')}")
                print(f"  - Existing entry: {existing_entry}")
                print(f"  - New entry: {entry}")
                return False
        return True

    def _strip_csv_row(self, row: dict) -> dict:
        """Optimized version using dictionary comprehension"""
        return {key: value.strip() if isinstance(value, str) else value
                for key, value in row.items()}

    def parse_esp_secure_cert_csv(self, csv_file):
        """Parse ESP Secure Cert CSV configuration file"""

        if not os.path.exists(csv_file):
            raise FileNotFoundError(f"CSV file not found: {csv_file}")

        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)

            for line_num, row in enumerate(reader, 1):
                # Strip all values once at the beginning (optimized)
                row = self._strip_csv_row(row)

                # Skip empty lines and comments
                if not row or (row.get('tlv_type', '').startswith('#')):
                    continue

                try:

                    if hasattr(tlv_format.tlv_type_t, row['tlv_type']):
                        tlv_type = getattr(tlv_format.tlv_type_t, row['tlv_type'])
                    else:
                        tlv_type = int(row['tlv_type'])

                    entry = {
                        'tlv_type': tlv_type,
                        'tlv_subtype': int(row['tlv_subtype']),
                        'data_value': row['data_value'],
                        'data_type': row['data_type'],
                        'priv_key_type': row['priv_key_type'].lower(),
                        'algorithm': row['algorithm'].upper(),
                        'key_size': int(row['key_size']) if row['key_size'] else 0,
                        'efuse_id': int(row['efuse_id']) if row['efuse_id'] else None,
                        'efuse_key_file': row['efuse_key'] if row['efuse_key'] else None,
                        'ds_enabled': row['priv_key_type'].lower() in ['rsa_ds', 'ecdsa_peripheral'],
                    }

                    self.add_entry(entry)

                except Exception as e:
                    print(f"Error parsing line {line_num}: {row}, error: {e}")
                    continue
        return

    def add_entry(self, entry):
        """
        Add an entry to entries after checking for duplicate (type, subtype).
        Returns True if added, False if duplicate found.
        """

        if entry['tlv_type'] == tlv_format.tlv_type_t.ESP_SECURE_CERT_PRIV_KEY_TLV:
            # Determine if DS (Digital Signature) is enabled for this private key
            entry['ds_enabled'] = entry['priv_key_type'] in ('rsa_ds', 'ecdsa_peripheral')
            entry['efuse_key_file'] = entry.get('efuse_key_file', '')
            entry['key_size'] = entry.get('key_size', 0)
            entry['algorithm'] = entry.get('algorithm', '')
            entry['private_key_pass'] = None

        # Add the processed entry to the list
        self._validate_entry(entry)
        self.secure_cert_entries.append(entry)

    def generate_esp_secure_cert(self, target_chip, port=None):
        """
        Process ESP Secure Cert CSV and generate partition.

        Args:
            target_chip (str): Target ESP chip (e.g., esp32c3, esp32s3)
            port (str, optional): Serial port for device communication.
                                If None, DS configuration will be done locally without device interaction.
        """

        try:
            # Group entries by type for better logging
            entries_by_type = {}
            for entry in self.secure_cert_entries:
                key_type = entry.get('tlv_type')
                if key_type not in entries_by_type:
                    entries_by_type[key_type] = []
                entries_by_type[key_type].append(entry)

            print("\n=== Entry Summary by Type ===")
            for tlv_type, type_entries in entries_by_type.items():
                print(f"Type {tlv_type}: {len(type_entries)} entries")
                for entry in type_entries:
                    print(f"  - Subtype {entry['tlv_subtype']}: {entry['data_type']} format")

            # Initialize DS variables for each configuration
            ds_tlv_entries = []  # Store results for each DS config

            # Handle DS configuration if enabled (only for private key entries)
            ds_enabled_entries = [entry for entry in self.secure_cert_entries if entry.get('ds_enabled', False)]
            if ds_enabled_entries:
                # Display operation mode information
                if port:
                    print(f"\n=== DS Configuration Mode: Device Connected (Port: {port}) ===")
                    print("INFO: DS keys will be burned to device eFuse blocks")
                else:
                    print("\n=== DS Configuration Mode: Local Only (No Port) ===")
                    print("WARNING: DS keys will be generated locally but NOT burned to device")
                    print("WARNING: You will need to manually burn the keys to device eFuse later")

                for entry in ds_enabled_entries:
                    ds_tlv_entry = {
                        'subtype': entry['tlv_subtype'],
                        'algorithm': entry['algorithm'],
                        'key_size': entry['key_size'],
                        'efuse_id': entry['efuse_id'],
                        'c': None,
                        'iv': None,
                        'rsa_key_len': entry['key_size']
                    }

                    if entry['algorithm'] == 'RSA':
                        efuse_key_file = get_efuse_key_file(entry['efuse_key_file'])
                        print(f"\nProcessing RSA DS configuration (subtype {entry['tlv_subtype']}):")
                        print(f"  - Key size: {entry['key_size']} bits")
                        print(f"  - eFuse key ID: {entry['efuse_id']}")
                        if efuse_key_file:
                            print(f"  - Using provided eFuse key file: {efuse_key_file}")
                        else:
                            print(f"  - Will generate new HMAC key file: {hmac_key_file}")

                        hmac_key = configure_ds.configure_efuse_for_rsa(
                            target_chip, port, efuse_key_file,
                            str(entry['key_size']), entry['data_value'],
                            None, entry['efuse_id']
                        )
                        c, iv, rsa_key_len = configure_ds.calculate_rsa_ds_params(
                            entry['data_value'], None, hmac_key, target_chip
                        )
                        ds_tlv_entry['c'] = c
                        ds_tlv_entry['iv'] = iv
                        ds_tlv_entry['rsa_key_len'] = rsa_key_len
                        print("  - RSA DS parameters calculated successfully")

                    elif entry['algorithm'] == 'ECDSA':
                        configure_ds.configure_efuse_for_ecdsa(
                            target_chip, port, ecdsa_key_file,
                            esp_secure_cert_data_dir, str(entry['key_size']),
                            entry['data_value'], None, entry['efuse_id']
                        )
                        print("  - ECDSA DS configuration completed successfully")

                    ds_tlv_entries.append(ds_tlv_entry)

            # Auto-add DS-related TLVs for each DS configuration
            for ds_tlv_entry in ds_tlv_entries:
                if ds_tlv_entry['algorithm'] == 'RSA':
                    self.builder.add_ds_data(ds_tlv_entry['c'], ds_tlv_entry['iv'], ds_tlv_entry['rsa_key_len'], ds_tlv_entry['subtype'])
                    self.builder.add_ds_context(ds_tlv_entry['efuse_id'], ds_tlv_entry['rsa_key_len'], ds_tlv_entry['subtype'])
                elif ds_tlv_entry['algorithm'] == 'ECDSA':
                    self.builder.add_security_config(ds_tlv_entry['efuse_id'], ds_tlv_entry['subtype'])

            if not ds_tlv_entries:
                print("No DS configuration found")

            print("\n=== Processing TLV Entries ===")
            processed_count = 0
            for entry in self.secure_cert_entries:
                tlv_type = entry.get('tlv_type')
                tlv_subtype = entry.get('tlv_subtype')
                data_value = entry.get('data_value')
                data_type = entry.get('data_type')
                priv_key_type = entry['priv_key_type'] if 'priv_key_type' in entry else None
                configure_ds_enabled = entry['ds_enabled'] if 'ds_enabled' in entry else None

                print(f"Processing: Type {tlv_type}, Subtype {tlv_subtype}, Data Type: {data_type}")

                # Process data based on type
                try:
                    processed_data = self._parse_data_from_any_format(data_value, data_type, False)

                    # Handle certificates
                    if self._is_certificate_entry(tlv_type):  # CA_CERT or DEV_CERT
                        cert_type_name = "CA Certificate" if tlv_type == tlv_format.tlv_type_t.ESP_SECURE_CERT_CA_CERT_TLV else "Device Certificate"
                        print(f"  Adding {cert_type_name} (subtype {tlv_subtype})")
                        self.builder.add_certificate(tlv_type_t(tlv_type), processed_data, tlv_subtype)
                        processed_count += 1
                    # Handle private key
                    elif self._is_private_key_entry(tlv_type):  # PRIV_KEY
                        if priv_key_type == 'plaintext':
                            print(f"  Adding private key as plaintext for subtype {tlv_subtype}")
                            self.builder.add_private_key(processed_data, None, tlv_format.tlv_priv_key_type_t.ESP_SECURE_CERT_DEFAULT_FORMAT_KEY, tlv_subtype)
                            processed_count += 1
                        elif priv_key_type == 'rsa_ds' and configure_ds_enabled:
                            print(f"  Skipping private key TLV for subtype {tlv_subtype} (using hardware RSA DS)")
                            processed_count += 1
                            pass  # RSA DS private key is handled by DS data and context
                        elif priv_key_type == 'ecdsa_peripheral' and configure_ds_enabled:
                            print(f"  Adding private key for hardware ECDSA DS for subtype {tlv_subtype}")
                            self.builder.add_private_key(processed_data, None, tlv_format.tlv_priv_key_type_t.ESP_SECURE_CERT_ECDSA_PERIPHERAL_KEY, tlv_subtype)
                            processed_count += 1
                    else:  # Direct data processing
                        if data_type == 'file':
                            with open(processed_data, 'rb') as f:
                                processed_data = f.read()

                            print(f"  Adding direct data (subtype {tlv_subtype})")
                            self.builder.add_tlv_entry(tlv_type, tlv_subtype, processed_data, 0)
                            processed_count += 1

                        else:
                            # If processed_data is a .bin file path, read its contents as bytes
                            if isinstance(processed_data, str) and processed_data.lower().endswith('.bin') and os.path.isfile(processed_data):
                                with open(processed_data, 'rb') as bin_f:
                                    bin_data = bin_f.read()
                                self.builder.add_tlv_entry(tlv_type, tlv_subtype, bin_data, 0)
                                print(f"  Added binary data from {processed_data} (subtype {tlv_subtype})")
                            else:
                                self.builder.add_tlv_entry(tlv_type, tlv_subtype, processed_data, 0)
                                print(f"  Skipping unknown TLV type {tlv_type} (subtype {tlv_subtype})")
                            processed_count += 1

                except Exception as e:
                    print(f"Error processing entry {tlv_type}: {e}")
                    continue

            print(f"\nSuccessfully processed {processed_count} out of {len(self.secure_cert_entries)} entries")

            # Build partition
            self.builder.build_partition(self.bin_filename)
            print(f'\nPartition generated: {self.bin_filename}')

            # Display summary
            print("\n=== ESP Secure Cert Generation Summary ===")
            print(f"Target chip: {target_chip}")
            print(f"Operation mode: {'Device Connected' if port else 'Local Only'}")
            print(f"Partition file: {self.bin_filename}")
            print(f"Total TLV entries: {len(self.secure_cert_entries)}")

            if ds_enabled_entries:
                print(f"DS peripheral entries: {len(ds_enabled_entries)}")
                if not port:
                    print("IMPORTANT: DS keys were generated locally. You must burn them to device eFuse manually:")
                    for entry in ds_enabled_entries:
                        efuse_key_file = get_efuse_key_file(entry['efuse_key_file'])
                        key_file = efuse_key_file if efuse_key_file else (ecdsa_key_file if entry['algorithm'] == 'ECDSA' else hmac_key_file)
                        if entry['algorithm'] == 'RSA':
                            print(f"  espefuse.py --chip {target_chip} -p <PORT> burn_key BLOCK_KEY{entry['efuse_id']} {key_file} HMAC_DOWN_DIGITAL_SIGNATURE")
                        elif entry['algorithm'] == 'ECDSA':
                            print(f"  espefuse.py --chip {target_chip} -p <PORT> burn_key BLOCK_KEY{entry['efuse_id']} <ecdsa_key.pem> ECDSA_KEY")

            print("=" * 50)

            return self.bin_filename

        except Exception as e:
            print(f'ERROR: Failed to process ESP Secure Cert CSV: {e}')
            import traceback
            traceback.print_exc()
            sys.exit(-1)

    # Flash esp_secure_cert partition after its generation
    # @info
    # The partition shall be flashed at the offset provided
    # for the --sec_cert_part_offset option.
    # The port is required for flashing the esp_secure_cert partition.
    # The flash_filename is the filename of the esp_secure_cert partition.
    def flash_esp_secure_cert_partition(self, idf_target, port, sec_cert_part_offset, flash_filename):
        """
        Flash the esp_secure_cert partition to the device.

        Args:
            idf_target (str): Target ESP chip
            port (str): Serial port for device communication
            sec_cert_part_offset (str): Flash offset for the partition
            flash_filename (str): Path to the partition binary file
        """
        print('\n=== Flashing ESP Secure Cert Partition ===')
        print(f'Partition file: {flash_filename}')
        print(f'Flash offset: {sec_cert_part_offset}')
        print('Note: You can skip this step by providing --skip_flash argument')

        # Check if the flash_filename exists
        if not os.path.exists(flash_filename):
            print(f"ERROR: The provided flash_filename {flash_filename} does not exist")
            sys.exit(-1)

        # Check if the port is provided
        if not port:
            print("WARNING: Port is not provided, skipping flash operation")
            print("INFO: Partition has been generated successfully but not flashed to device")
            print(f"To flash manually: esptool.py --chip {idf_target} -p <PORT> write_flash {sec_cert_part_offset} {flash_filename}")
            return

        flash_command = f"esptool.py --chip {idf_target} " + \
            f"-p {port} write_flash " + \
            f" {sec_cert_part_offset} {flash_filename}"
        try:
            flash_command_output = subprocess.check_output(
                flash_command,
                shell=True
            )
            print(flash_command_output.decode('utf-8'))
        except subprocess.CalledProcessError as e:
            print(e.output.decode("utf-8"))
            print('ERROR: Failed to execute the flash command')
            sys.exit(-1)

    def esp_secure_cert_cleanup(self):
        # Remove the directory esp_secure_cert_data
        if os.path.exists(esp_secure_cert_data_dir):
            for filename in os.listdir(esp_secure_cert_data_dir):
                if filename.startswith("temp_"):
                    file_path = os.path.join(esp_secure_cert_data_dir, filename)
                    if os.path.isfile(file_path):
                        os.remove(file_path)

    @staticmethod
    def parse_esp_secure_cert_bin(bin_file_path):
        """
        Parse an esp_secure_cert.bin file and generate CSV from parsed data.
        - CA/Device cert: extract, save as file, add row to CSV
        - Custom data: add row to CSV, no file
        - Private key:
          - plaintext: extract, save as file, add row to CSV
          - rsa_ds: add DS_DATA and DS_CONTEXT rows/files instead of priv key
          - ecdsa_peripheral: add SEC_CFG row/file instead of priv key
        All files are created in esp_secure_cert_parsed_data and referenced in the CSV.
        Adds efuse_id, key_size, algorithm, priv_key_type if possible.
        """
        output_dir = 'esp_secure_cert_parsed_data'
        content_dir = os.path.join(output_dir, "contents")
        if os.path.exists(output_dir):
            shutil.rmtree(output_dir)
        os.makedirs(output_dir)
        os.makedirs(content_dir)
        # Check if the file exists and is a .bin file
        if not os.path.isfile(bin_file_path):
            print(f"ERROR: The provided file '{bin_file_path}' does not exist or is not a file.")
            sys.exit(-1)
        if not bin_file_path.lower().endswith('.bin'):
            print(f"ERROR: The provided file '{bin_file_path}' is not a .bin file.")
            sys.exit(-1)
        # Check if the file is not empty and has a valid TLV header
        try:
            with open(bin_file_path, 'rb') as f:
                data = f.read()
            if len(data) == 0:
                print(f"ERROR: The provided .bin file '{bin_file_path}' is empty.")
                sys.exit(-1)
            # Check for valid TLV magic at the start
            if len(data) < 4:
                print(f"ERROR: The provided .bin file '{bin_file_path}' is too small to be a valid TLV partition.")
                sys.exit(-1)
            magic = int.from_bytes(data[0:4], 'little')

            if magic != TLV_MAGIC:
                print(f"ERROR: The provided .bin file '{bin_file_path}' does not have a valid TLV_MAGIC header (found 0x{magic:08X}).")
                sys.exit(-1)
        except Exception as e:
            print(f"ERROR: Failed to read or validate the .bin file '{bin_file_path}': {e}")
            sys.exit(-1)

        # Parse TLV entries
        tlv_entries = EspSecureCert._parse_tlv_entries_from_bin(bin_file_path)

        # Check if any TLV entries were actually parsed
        if not tlv_entries or len(tlv_entries) == 0:
            print(f"ERROR: No valid TLV entries found in the provided .bin file '{bin_file_path}'.")
            print("This file may not be a valid ESP Secure Cert partition or may be corrupted.")
            sys.exit(-1)

        # Group by type/subtype for lookup
        entries_by_type = {}
        for entry in tlv_entries:
            t = entry['header']['type']
            s = entry['header']['subtype']
            if t not in entries_by_type:
                entries_by_type[t] = {}
            entries_by_type[t][s] = entry
        # Prepare CSV rows
        csv_rows = []

        # Iterate and extract
        for entry in tlv_entries:
            t = entry['header']['type']
            s = entry['header']['subtype']
            data = entry['data']
            # CA cert
            if t == tlv_format.tlv_type_t.ESP_SECURE_CERT_CA_CERT_TLV:
                fname = f"cacert_{t}_{s}.pem"
                fpath = os.path.join(content_dir, fname)
                if data.startswith(b'-----BEGIN CERTIFICATE-----'):
                    with open(fpath, 'w') as f:
                        f.write(data.rstrip(b'\x00').decode('utf-8'))
                else:
                    fname = f"cacert_{t}_{s}.der"
                    fpath = os.path.join(content_dir, fname)
                    with open(fpath, 'wb') as f:
                        f.write(data)
                csv_rows.append({
                    'tlv_type': 'ESP_SECURE_CERT_CA_CERT_TLV',
                    'tlv_subtype': s,
                    'data_value': f"contents/{fname}",  # Reference file in contents subdirectory
                    'data_type': 'file',
                    'priv_key_type': '',
                    'algorithm': '',
                    'key_size': '',
                    'efuse_id': '',
                    'efuse_key': ''
                })
            # Device cert
            elif t == tlv_format.tlv_type_t.ESP_SECURE_CERT_DEV_CERT_TLV:
                fname = f"devcert_{t}_{s}.pem"
                fpath = os.path.join(content_dir, fname)
                if data.startswith(b'-----BEGIN CERTIFICATE-----'):
                    with open(fpath, 'w') as f:
                        f.write(data.rstrip(b'\x00').decode('utf-8'))
                else:
                    fname = f"devcert_{t}_{s}.der"
                    fpath = os.path.join(content_dir, fname)
                    with open(fpath, 'wb') as f:
                        f.write(data)
                csv_rows.append({
                    'tlv_type': 'ESP_SECURE_CERT_DEV_CERT_TLV',
                    'tlv_subtype': s,
                    'data_value': f"contents/{fname}",  # Reference file in contents subdirectory
                    'data_type': 'file',
                    'priv_key_type': '',
                    'algorithm': '',
                    'key_size': '',
                    'efuse_id': '',
                    'efuse_key': ''
                })
            # Private key
            elif t == tlv_format.tlv_type_t.ESP_SECURE_CERT_PRIV_KEY_TLV:
                # Plaintext
                fname = f"privkey_{t}_{s}.pem"
                fpath = os.path.join(content_dir, fname)
                algo = ''
                if data.startswith(b'-----BEGIN'):
                    key_bytes = data.rstrip(b'\x00')
                    pem_header = key_bytes.split(b'\n', 1)[0]
                    # Try to infer algorithm from PEM header
                    if b'RSA' in pem_header:
                        algo = 'RSA'
                    elif b'EC' in pem_header or b'ECDSA' in pem_header:
                        algo = 'ECDSA'
                    # PKCS#8: -----BEGIN PRIVATE KEY-----
                    if b'PRIVATE KEY-----' in pem_header and b'RSA' not in pem_header:
                        # Re-serialize as PKCS#8 using cryptography
                        from cryptography.hazmat.primitives import serialization
                        key = serialization.load_pem_private_key(key_bytes, password=None)
                        pem = key.private_bytes(
                            encoding=serialization.Encoding.PEM,
                            format=serialization.PrivateFormat.PKCS8,
                            encryption_algorithm=serialization.NoEncryption()
                        )
                        with open(fpath, 'wb') as f:
                            f.write(pem)
                    else:
                        # PKCS#1 or other: write as-is
                        with open(fpath, 'w') as f:
                            f.write(key_bytes.decode('utf-8'))
                else:
                    fname = f"privkey_{t}_{s}.der"
                    fpath = os.path.join(content_dir, fname)
                    with open(fpath, 'wb') as f:
                        f.write(data)
                csv_rows.append({
                    'tlv_type': 'ESP_SECURE_CERT_PRIV_KEY_TLV',
                    'tlv_subtype': s,
                    'data_value': f"contents/{fname}",  # Reference file in contents subdirectory
                    'data_type': 'file',
                    'priv_key_type': 'plaintext',
                    'algorithm': algo,
                    'key_size': '',
                    'efuse_id': '',
                    'efuse_key': ''
                })
            # DS_DATA (if not already handled)
            elif t == tlv_format.tlv_type_t.ESP_SECURE_CERT_DS_DATA_TLV:
                if not any(r['tlv_type'] == 'ESP_SECURE_CERT_DS_DATA_TLV' and r['tlv_subtype'] == s for r in csv_rows):
                    fname = f"ds_data_{t}_{s}.bin"
                    fpath = os.path.join(content_dir, fname)
                    with open(fpath, 'wb') as f:
                        f.write(data)
                    key_size = ''
                    if len(data) >= 4:
                        key_size_val = struct.unpack('<i', data[:4])[0]
                        key_size = str((key_size_val + 1) * 32)
                    csv_rows.append({
                        'tlv_type': 'ESP_SECURE_CERT_DS_DATA_TLV',
                        'tlv_subtype': s,
                        'data_value': f"contents/{fname}",  # Reference file in contents subdirectory
                        'data_type': 'file',
                        'priv_key_type': 'rsa_ds',
                        'algorithm': 'RSA',
                        'key_size': key_size,
                        'efuse_id': '',
                        'efuse_key': ''
                    })
            # DS_CONTEXT (if not already handled)
            elif t == tlv_format.tlv_type_t.ESP_SECURE_CERT_DS_CONTEXT_TLV:
                if not any(r['tlv_type'] == 'ESP_SECURE_CERT_DS_CONTEXT_TLV' and r['tlv_subtype'] == s for r in csv_rows):
                    fname = f"ds_context_{t}_{s}.bin"
                    fpath = os.path.join(content_dir, fname)
                    with open(fpath, 'wb') as f:
                        f.write(data)
                    efuse_id = ''
                    ctx_key_size = ''
                    if len(data) >= 8:
                        efuse_id = str(data[4])
                        ctx_key_size = str(int.from_bytes(data[6:8], 'little'))
                    csv_rows.append({
                        'tlv_type': 'ESP_SECURE_CERT_DS_CONTEXT_TLV',
                        'tlv_subtype': s,
                        'data_value': f"contents/{fname}",  # Reference file in contents subdirectory
                        'data_type': 'file',
                        'priv_key_type': 'rsa_ds',
                        'algorithm': 'RSA',
                        'key_size': ctx_key_size,
                        'efuse_id': efuse_id,
                        'efuse_key': ''
                    })
            # # SEC_CFG (if not already handled)
            elif t == tlv_format.tlv_type_t.ESP_SECURE_CERT_SEC_CFG_TLV:
                if not any(r['tlv_type'] == 'ESP_SECURE_CERT_SEC_CFG_TLV' and r['tlv_subtype'] == s for r in csv_rows):
                    fname = f"sec_cfg_{t}_{s}.bin"
                    fpath = os.path.join(content_dir, fname)
                    with open(fpath, 'wb') as f:
                        f.write(data)
                    efuse_id = ''
                    if len(data) >= 1:
                        efuse_id = str(data[0] - 4)
                    csv_rows.append({
                        'tlv_type': 'ESP_SECURE_CERT_SEC_CFG_TLV',
                        'tlv_subtype': s,
                        'data_value': f"contents/{fname}",  # Reference file in contents subdirectory
                        'data_type': 'file',
                        'priv_key_type': 'ecdsa_peripheral',
                        'algorithm': 'ECDSA',
                        'key_size': '256',
                        'efuse_id': efuse_id,
                        'efuse_key': ''
                    })
            # Custom user data
            elif t >= tlv_format.tlv_type_t.ESP_SECURE_CERT_USER_DATA_1_TLV:
                try:
                    text = data.decode('utf-8')
                    if all(c.isprintable() or c.isspace() for c in text):
                        val = text
                        dtype = 'string'
                    else:
                        val = data.hex().upper()
                        dtype = 'hex'
                except Exception:
                    val = data.hex().upper()
                    dtype = 'hex'
                csv_rows.append({
                    'tlv_type': tlv_format.tlv_type_t(t).name,
                    'tlv_subtype': s,
                    'data_value': val,
                    'data_type': dtype,
                    'priv_key_type': '',
                    'algorithm': '',
                    'key_size': '',
                    'efuse_id': '',
                    'efuse_key': ''
                })

        # Check if any CSV rows were created (i.e., any valid entries were processed)
        if not csv_rows or len(csv_rows) == 0:
            print(f"ERROR: No valid TLV entries could be processed from the .bin file '{bin_file_path}'.")
            print("The file may contain only unknown or invalid TLV types.")
            sys.exit(-1)

        # Write CSV
        csv_file_path = os.path.join(output_dir, 'esp_secure_cert_parsed.csv')
        try:
            with open(csv_file_path, 'w', newline='') as csvfile:
                fieldnames = ['tlv_type', 'tlv_subtype', 'data_value', 'data_type', 'priv_key_type', 'algorithm', 'key_size', 'efuse_id', 'efuse_key']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(csv_rows)
        except Exception as e:
            print(f"ERROR: Could not write CSV file '{csv_file_path}': {e}")
            sys.exit(-1)

        # Write TLV partition full output to tlv_entries_raw.txt using tlv_parser.py's process_tlv_entries
        try:

            # Add the tools directory to sys.path if needed
            tools_dir = os.path.join(os.path.dirname(__file__), '..')
            tools_dir = os.path.abspath(tools_dir)
            if tools_dir not in sys.path:
                sys.path.insert(0, tools_dir)

            # Redirect stdout to capture all prints from process_tlv_entries
            summary_output = io.StringIO()
            sys_stdout = sys.stdout
            sys.stdout = summary_output
            tlv_parser.process_tlv_entries(bin_file_path)
            sys.stdout = sys_stdout
            summary_text = summary_output.getvalue()
            summary_output.close()
            # Normalize line endings and remove null bytes
            summary_text = summary_text.replace('\r\n', '\n').replace('\r', '\n').replace('\x00', '\n')
            with open(os.path.join(output_dir, 'tlv_entries_raw.txt'), 'w') as f:
                f.write(summary_text)
        except Exception as e:
            print(f"WARNING: Could not write tlv_entries_raw.txt: {e}")

        print("\nSuccessfully parsed binary file!")
        print("Generated files:")
        print(f"  - CSV file: {csv_file_path}")
        print(f"      - {len(csv_rows)} valid entries parsed")
        print("      - Entries:")
        for row in csv_rows:
            tlv_type = row.get('tlv_type', 'N/A')
            tlv_subtype = row.get('tlv_subtype', 'N/A')
            data_value = row.get('data_value', '')
            tlv_length = len(data_value) if data_value else 0
            if tlv_length > 0:
                print(f"        TLV type: {tlv_type}, subtype: {tlv_subtype}, length: {tlv_length}")
            else:
                print(f"        TLV type: {tlv_type}, subtype: {tlv_subtype}")
        print(f"  - tlv_entries_raw.txt: {os.path.join(output_dir, 'tlv_entries_raw.txt')}")
        print(f"  - All files in: {output_dir}")

    @staticmethod
    def _parse_tlv_entries_from_bin(bin_file_path):
        """
        Parse TLV entries from binary file using the same logic as tlv_parser.py
        Returns a list of dicts with header, data, and offset.
        """
        with open(bin_file_path, 'rb') as f:
            data = f.read()
        if len(data) == 0:
            raise ValueError("Binary file is empty")
        offset = 0
        tlv_entries = []
        while offset < len(data):
            try:
                tlv_header, header_size = tlv_parser.parse_tlv_header(data, offset)
                if tlv_header['magic'] == MAGIC_END:
                    break
                if tlv_header['magic'] != TLV_MAGIC:
                    break
                tlv_data_offset = offset + header_size
                padding_length = _calculate_padding(tlv_header['length'])
                tlv_data = data[tlv_data_offset:tlv_data_offset + tlv_header['length']]
                entry = {
                    'header': tlv_header,
                    'data': tlv_data,
                    'offset': offset
                }
                tlv_entries.append(entry)
                total_tlv_size = header_size + tlv_header['length'] + padding_length + 4  # +4 for CRC footer
                offset += total_tlv_size
            except Exception as e:
                print(f"Warning: Error parsing TLV at offset 0x{offset:04X}: {e}")
                break
        return tlv_entries

    # =============== Secure Verification related functions ===============

    def generate_entries_hash(self, tlv_entries_data) -> bytes:
        """Generate hash of TLV entries"""
        print(f"====entries data length: {len(tlv_entries_data)}")
        if self.hash_type == "sha256":
            return _sha256_digest(tlv_entries_data)
        elif self.hash_type == "sha384":
            return _sha384_digest(tlv_entries_data)
        else:
            raise ValueError(f"Unsupported hash type: {self.hash_type}")

    def _extract_public_key_and_algorithm(self, private_key_path: str) -> Tuple[int, bytes]:
        """
        Extract public key and detect algorithm using espsecure public APIs only.
        Uses generate_signature_block_using_private_key to create a signature block
        and extracts the public key from it.

        Args:
            private_key_path: Path to the private key file

        Returns:
            Tuple of (algorithm, public_key_bytes):
            - algorithm: 0=RSA3072, 1=ECDSA192, 2=ECDSA256, 3=ECDSA384
            - public_key_bytes: Serialized public key in format expected by signature block
        """
        with open(private_key_path, 'rb') as keyfile:
            public_key = _get_sbv2_pub_key(keyfile)

            if isinstance(public_key, rsa.RSAPublicKey):

                primitives = _get_sbv2_rsa_primitives(public_key)
                binary_key = struct.pack(
                    "<384sI384sI",
                    int_to_bytes(primitives.n)[::-1],  # little-endian
                    primitives.e,
                    int_to_bytes(primitives.rinv)[::-1],  # little-endian
                    primitives.m & 0xFFFFFFFF,
                )
                algorithm = 0  # RSA3072
            else:
                numbers = public_key.public_numbers()
                if isinstance(public_key.curve, ec.SECP192R1):
                    curve_len = 192
                    curve_id = 1  # CURVE_ID_P192
                    algorithm = 1  # ECDSA192
                elif isinstance(public_key.curve, ec.SECP256R1):
                    curve_len = 256
                    curve_id = 2  # CURVE_ID_P256 (was incorrectly 1)
                    algorithm = 2  # ECDSA256
                elif isinstance(public_key.curve, ec.SECP384R1):
                    curve_len = 384
                    curve_id = 3  # CURVE_ID_P384 (was incorrectly 2)
                    algorithm = 3  # ECDSA384
                else:
                    raise ValueError(f"Unsupported curve: {public_key.curve}")
                pubkey_point = _microecc_format(numbers.x, numbers.y, curve_len)

                # Use correct format based on curve size
                if curve_id == 3:  # P384
                    binary_key = struct.pack("<B96s", curve_id, pubkey_point)
                else:  # P192 or P256
                    binary_key = struct.pack("<B64s", curve_id, pubkey_point)
            return algorithm, binary_key

    def _sign_data_with_espsecure(self, private_key_path: str, bin_data: bytes, algorithm: int) -> bytes:
        """Generate signature using espsecure.sign_data API with --skip-padding
        Args:
            private_key_path: Path to the private key file
            bin_data: The partition data (bytes) to sign - espsecure.sign_data will hash it internally
            algorithm: Algorithm type (0=RSA3072, 1=ECDSA192, 2=ECDSA256, 3=ECDSA384)
        Returns: signature bytes in little-endian format (as expected by firmware)
        """
        try:
            # Create temporary files for input/output
            with tempfile.NamedTemporaryFile(mode='wb', suffix='.bin', delete=False) as temp_input:
                temp_input_path = temp_input.name
                temp_input.write(bin_data)

            with tempfile.NamedTemporaryFile(mode='rb', suffix='_signed.bin', delete=False) as temp_output:
                temp_output_path = temp_output.name

            try:
                # Use sign_data API with skip_padding=True
                print("Calling sign_data API with skip_padding=True")
                print(f"  Input file: {temp_input_path} ({len(bin_data)} bytes)")
                print(f"  Output file: {temp_output_path}")
                print(f"  Private key: {private_key_path}")

                with open(private_key_path, 'rb') as keyfile:
                    with open(temp_input_path, 'rb') as datafile:
                        try:
                            sign_data(
                                version="2",  # Must be string, not int
                                keyfile=[keyfile],
                                output=temp_output_path,
                                append_signatures=False,
                                hsm=False,
                                hsm_config=None,
                                pub_key=[],
                                signature=[],
                                datafile=datafile,
                                skip_padding=True  # Skip sector padding
                            )
                        except Exception as e:
                            print(f"sign_data API call failed: {e}")
                            import traceback
                            traceback.print_exc()
                            raise

                # Read the signed output
                if not os.path.exists(temp_output_path):
                    raise ValueError(f"Output file was not created: {temp_output_path}")

                with open(temp_output_path, 'rb') as f:
                    signed_data = f.read()

                if len(signed_data) == 0:
                    raise ValueError(f"Output file is empty: {temp_output_path}")

                data_length = len(bin_data)

                sig_block = signed_data[data_length:]

                print(f"Original data length: {data_length} bytes")
                print(f"Signed data length: {len(signed_data)} bytes")
                print(f"Signature block offset: {data_length}")
                print(f"Signature block length: {len(sig_block)} bytes")

                # Validate signature block
                if len(sig_block) < 2:
                    raise ValueError("Signature block too short")

                magic, version = struct.unpack("BB", sig_block[:2])
                if magic != ECDSA_SIG_BLOCK_MAGIC:
                    raise ValueError(f"Invalid signature block magic byte: 0x{magic:02X}, expected 0x{ECDSA_SIG_BLOCK_MAGIC:02X}")

                # Extract signature based on algorithm
                if algorithm == ECDSA192_ALGORITHM:
                    # ECDSA SHA256 signature block format: <BBBx32sB64s64s1031x
                    # signature_rs is at offset 101-165 (64 bytes), but P192 only uses first 48 bytes
                    # Signature is in microecc format: little-endian r||s concatenated
                    signature = sig_block[101:101+48]  # 48 bytes: 24 bytes r + 24 bytes s
                    print(f"ECDSA192 signature extracted: {len(signature)} bytes")

                elif algorithm == ECDSA256_ALGORITHM:
                    # ECDSA SHA256 signature block format: <BBBx32sB64s64s1031x
                    # signature_rs is at offset 101-165 (64 bytes)
                    # Signature is in microecc format: little-endian r||s concatenated
                    signature = sig_block[101:165]  # 64 bytes: 32 bytes r + 32 bytes s
                    print(f"ECDSA256 signature extracted: {len(signature)} bytes")

                elif algorithm == ECDSA384_ALGORITHM:
                    # ECDSA SHA384 signature block format: <BBBx48sB96s96s951x
                    # signature_rs is at offset 149-245 (96 bytes)
                    # Signature is in microecc format: little-endian r||s concatenated
                    signature = sig_block[149:245]  # 96 bytes: 48 bytes r + 48 bytes s
                    print(f"ECDSA384 signature extracted: {len(signature)} bytes")
                else:
                    raise ValueError(f"Unsupported algorithm: {algorithm}")

                # Verify CRC32
                calc_crc = zlib.crc32(sig_block[:1196]) & 0xFFFFFFFF
                blk_crc = struct.unpack("<I", sig_block[1196:1200])[0]
                if calc_crc != blk_crc:
                    raise ValueError("Signature block CRC32 mismatch")

                return signature

            finally:
                # Clean up temporary files
                if os.path.exists(temp_input_path):
                    os.unlink(temp_input_path)
                if os.path.exists(temp_output_path):
                    os.unlink(temp_output_path)

        except Exception as e:
            print(f"Error generating signature with espsecure.sign_data: {e}")
            raise

    def add_signature_block_using_existing_key(self, data_file_path: str, signing_key_file: Union[List[str], str], signing_scheme: str):
        """Add signature block to the partition using existing key

        Args:
            data_file_path: Path to the partition data file
            signing_key_file: List of paths to signing key files, or a single path string
        """

        if not os.path.exists(data_file_path):
            raise ValueError("Data file does not exist")

        # Normalize signing_key_file to a list
        if isinstance(signing_key_file, str):
            signing_key_file = [signing_key_file]
        elif not isinstance(signing_key_file, (list, tuple)):
            raise ValueError("signing_key_file must be a string, list, or tuple")

        self.secure_boot_scheme = signing_scheme

        # Validate all key files exist
        for key_path in signing_key_file:
            if not os.path.exists(key_path):
                raise FileNotFoundError(f"Signing key file not found: {key_path}")

        # Read the current partition data
        with open(data_file_path, 'rb') as f:
            bin_data = f.read()

        # Calculate hash using espsecure
        data_hash = _sha256_digest(bin_data)
        print(f"Calculated partition hash: {data_hash.hex()}")

        # Store original partition length for signature block length field
        original_partition_length = len(bin_data)

        # Reset builder for signature blocks only
        self.builder.partition_data = bytearray(b'\xff' * PARTITION_SIZE)
        self.builder.current_offset = 0

        try:
            for keyfile_path in signing_key_file:
                print(f"\n=== Processing signing key: {keyfile_path} ===")

                # Extract public key and detect algorithm using espsecure
                algorithm, public_key = self._extract_public_key_and_algorithm(keyfile_path)

                if algorithm == RSA3072_ALGORITHM:
                    with open(keyfile_path, 'rb') as keyfile:
                        signature_block = generate_signature_block_using_private_key(
                            keyfiles=[keyfile],
                            contents=bin_data)
                    signature = signature_block
                else:  # ECDSA192, ECDSA256, ECDSA384
                    signature = self._sign_data_with_espsecure(keyfile_path, bin_data, algorithm)

                self.builder.add_signature_block(algorithm, public_key, signature, self.signature_block_no, original_partition_length)
                self.signature_block_no += 1

        except Exception as e:
            print(f"Error generating signature block: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(-1)

        # Write final partition: original data + signature blocks appended directly
        with open(self.signed_bin_filename, 'wb') as f:
            f.write(bin_data)
            # Append signature blocks directly after partition data
            f.write(self.builder.partition_data[:self.builder.current_offset])

        print(f"\n=== Signed partition created: {self.signed_bin_filename} ===")
        return self.signed_bin_filename
