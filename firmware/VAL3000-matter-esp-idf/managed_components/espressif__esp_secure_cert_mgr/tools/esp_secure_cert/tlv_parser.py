#!/usr/bin/env python3
"""
ESP Secure Certificate TLV Parser

This script parses and displays the contents of an esp_secure_cert partition
in TLV (Type-Length-Value) format. It's helpful for debugging partitions
generated through pre-provisioning or other tools.

Usage: python tlv_parser.py <esp_secure_cert.bin>
"""

import struct
import zlib
import sys
import os
from typing import Dict, Tuple, Optional

# Import TLV type enum from tlv_format module
from esp_secure_cert.tlv_format import tlv_type_t

# Constants
MIN_ALIGNMENT_REQUIRED = 16
ESP_SECURE_CERT_TLV_MAGIC = 0xBA5EBA11
MAGIC_END = 0xFFFF  # Magic number indicating the end of TLV entries

# Private Key Type flags
PRIV_KEY_TYPES = {
    0: "DEFAULT_FORMAT_KEY (Plaintext)",
    1: "RSA_DS_PERIPHERAL_KEY (Hardware DS)",
    2: "ECDSA_PERIPHERAL_KEY (Hardware ECDSA)"
}

def crc32_le(data: bytes, seed: int = 0xFFFFFFFF) -> int:
    """Calculate CRC32 with little-endian byte order"""
    return zlib.crc32(data, seed) & 0xFFFFFFFF

def get_padding_length(length: int) -> int:
    """Calculate the padding length required for alignment"""
    return (MIN_ALIGNMENT_REQUIRED - (length % MIN_ALIGNMENT_REQUIRED)) % MIN_ALIGNMENT_REQUIRED

def get_tlv_total_length(header: Dict) -> int:
    """Calculate total TLV length including header, data, padding, and footer"""
    padding_length = get_padding_length(header['length'])
    total_length = struct.calcsize('I6BH') + header['length'] + padding_length + struct.calcsize('I')
    return total_length

def verify_tlv_integrity(tlv_header: Dict, tlv_data: bytes, tlv_footer: Dict) -> Tuple[bool, int]:
    """Verify TLV integrity using CRC32"""
    padding_length = get_padding_length(tlv_header['length'])
    crc_data_len = struct.calcsize('I6BH') + tlv_header['length'] + padding_length
    calculated_crc = crc32_le(tlv_data[:crc_data_len])
    return calculated_crc == tlv_footer['crc'], calculated_crc

def parse_tlv_header(data: bytes, offset: int) -> Tuple[Dict, int]:
    """Parse TLV header from binary data"""
    tlv_header_format = 'I6BH'  # 1x uint32 + 6x uint8 + 1x uint16
    tlv_header_size = struct.calcsize(tlv_header_format)

    if offset + tlv_header_size > len(data):
        raise IndexError(f"Not enough data to read TLV header at offset {offset}")

    tlv_header_data = struct.unpack_from(tlv_header_format, data, offset)

    tlv_header = {
        'magic': tlv_header_data[0],
        'flags': tlv_header_data[1],
        'reserved': tlv_header_data[2:5],  # 3 reserved bytes
        'type': tlv_header_data[5],
        'subtype': tlv_header_data[6],
        'length': tlv_header_data[7]
    }

    return tlv_header, tlv_header_size

def parse_tlv_footer(data: bytes, offset: int) -> Tuple[Dict, int]:
    """Parse TLV footer from binary data"""
    tlv_footer_format = 'I'  # 1x uint32
    tlv_footer_size = struct.calcsize(tlv_footer_format)

    if offset + tlv_footer_size > len(data):
        raise IndexError(f"Not enough data to read TLV footer at offset {offset}")

    tlv_footer_data = struct.unpack_from(tlv_footer_format, data, offset)

    tlv_footer = {
        'crc': tlv_footer_data[0]
    }

    return tlv_footer, tlv_footer_size

def format_certificate_data(cert_data: bytes) -> str:
    """Format certificate data for display"""
    try:
        cert_text = cert_data.decode('utf-8', errors='ignore')
        if '-----BEGIN CERTIFICATE-----' in cert_text:
            # Show complete certificate without truncation
            return cert_text.strip()
        else:
            return f"Binary certificate data: {cert_data.hex()}"
    except:
        return f"Binary certificate data: {cert_data.hex()}"

def format_private_key_data(key_data: bytes, flags: int) -> str:
    """Format private key data for display"""
    key_type = PRIV_KEY_TYPES.get(flags, f"Unknown type ({flags})")

    if flags == 0:  # Plaintext key
        try:
            key_text = key_data.decode('utf-8', errors='ignore')
            if '-----BEGIN' in key_text and '-----END' in key_text:
                # Show complete private key without truncation
                return f"{key_type}\n{key_text.strip()}"
            return f"{key_type}\nBinary key data: {key_data.hex()}"
        except:
            return f"{key_type}\nBinary key data: {key_data.hex()}"

def format_ds_context_data(ds_data: bytes) -> str:
    """Format DS context data for display"""
    if len(ds_data) >= 8:
        efuse_key_id = struct.unpack('<I', ds_data[0:4])[0]
        rsa_length = struct.unpack('<I', ds_data[4:8])[0]
        return f"eFuse Key ID: {efuse_key_id}, RSA Length: {rsa_length} bits"
    else:
        return f"Invalid DS context data: {ds_data.hex()}"

def format_security_config_data(sec_data: bytes) -> str:
    """Format security config data for ECDSA peripheral"""
    if len(sec_data) >= 4:
        efuse_key_id = struct.unpack('<I', sec_data[0:4])[0]
        return f"eFuse Key ID: {efuse_key_id}"
    else:
        return f"Invalid security config data: {sec_data.hex()}"

def format_custom_data(custom_data: bytes, max_display: int = None) -> str:
    """Format custom data for display"""
    try:
        # Try to decode as text
        text_data = custom_data.decode('utf-8', errors='ignore')
        if all(c.isprintable() or c.isspace() for c in text_data[:100]):
            # Show complete text data
            return f"Text data: {text_data}"
        else:
            # Show complete hex data
            return f"Hex data: {custom_data.hex()}"
    except:
        # Show complete hex data
        return f"Hex data: {custom_data.hex()}"

def format_tlv_data(tlv_type: int, tlv_flags: int, tlv_data: bytes) -> str:
    """Format TLV data based on type"""
    if tlv_type == tlv_type_t.ESP_SECURE_CERT_CA_CERT_TLV or tlv_type == tlv_type_t.ESP_SECURE_CERT_DEV_CERT_TLV:
        return format_certificate_data(tlv_data)
    elif tlv_type == tlv_type_t.ESP_SECURE_CERT_PRIV_KEY_TLV:
        return format_private_key_data(tlv_data, tlv_flags)
    elif tlv_type == tlv_type_t.ESP_SECURE_CERT_DS_DATA_TLV:
        return f"Encrypted DS data: {tlv_data.hex()}"
    elif tlv_type == tlv_type_t.ESP_SECURE_CERT_DS_CONTEXT_TLV:
        return format_ds_context_data(tlv_data)
    elif tlv_type == tlv_type_t.ESP_SECURE_CERT_SEC_CFG_TLV:
        return format_security_config_data(tlv_data)
    elif tlv_type >= tlv_type_t.ESP_SECURE_CERT_USER_DATA_1_TLV and tlv_type <= tlv_type_t.ESP_SECURE_CERT_USER_DATA_5_TLV:
        return format_custom_data(tlv_data)
    else:
        return f"Raw data: {tlv_data.hex()}"

def print_partition_summary(tlv_entries: list):
    """Print a summary of the partition contents"""
    print("\n" + "="*80)
    print("PARTITION SUMMARY")
    print("="*80)

    found_types = {}
    total_size = 0

    for entry in tlv_entries:
        tlv_type = entry['header']['type']
        try:
            type_name = tlv_type_t(tlv_type).name
        except ValueError:
            type_name = f"UNKNOWN_{tlv_type}"

        if type_name not in found_types:
            found_types[type_name] = []
        found_types[type_name].append(entry)
        total_size += get_tlv_total_length(entry['header'])

    print(f"Total TLV entries: {len(tlv_entries)}")
    print(f"Total partition size: {total_size} bytes")
    print("\nFound TLV types:")

    for type_name, entries in found_types.items():
        if len(entries) == 1:
            entry = entries[0]
            print(f"  - {type_name}: {entry['header']['length']} bytes")
        else:
            total_bytes = sum(e['header']['length'] for e in entries)
            print(f"  - {type_name}: {len(entries)} entries, {total_bytes} bytes total")

    # Determine key configuration
    print("\nKey Configuration:")
    has_ds_data = any(e['header']['type'] == tlv_type_t.ESP_SECURE_CERT_DS_DATA_TLV for e in tlv_entries)
    has_ds_context = any(e['header']['type'] == tlv_type_t.ESP_SECURE_CERT_DS_CONTEXT_TLV for e in tlv_entries)
    has_sec_cfg = any(e['header']['type'] == tlv_type_t.ESP_SECURE_CERT_SEC_CFG_TLV for e in tlv_entries)
    has_priv_key = any(e['header']['type'] == tlv_type_t.ESP_SECURE_CERT_PRIV_KEY_TLV for e in tlv_entries)

    if has_ds_data and has_ds_context:
        print("  - RSA with DS Peripheral (Hardware)")
    elif has_sec_cfg:
        print("  - ECDSA with Peripheral (Hardware)")
    elif has_priv_key:
        print("  - Software-based (Plaintext)")
    else:
        print("  - Unknown or incomplete configuration")

def process_tlv_entries(file_path: str):
    """Read and process TLV entries from a binary file"""
    if not os.path.exists(file_path):
        print(f"ERROR: File not found: {file_path}")
        sys.exit(1)

    with open(file_path, 'rb') as f:
        data = f.read()

    if len(data) == 0:
        print("ERROR: File is empty")
        sys.exit(1)

    print(f"ESP Secure Certificate TLV Parser")
    print(f"File: {file_path}")
    print(f"File size: {len(data)} bytes")
    print("="*80)

    offset = 0
    tlv_count = 0
    tlv_entries = []

    while offset < len(data):
        try:
            # Parse TLV header
            tlv_header, header_size = parse_tlv_header(data, offset)

            # Check if we have reached the end of the TLV entries
            if tlv_header['magic'] == MAGIC_END:
                print(f"\n✓ End of TLV entries (magic 0x{MAGIC_END:04X}) at offset 0x{offset:04X}")
                break

            # Check if the magic number is correct for a valid TLV entry
            if tlv_header['magic'] != ESP_SECURE_CERT_TLV_MAGIC:
                print(f"\nEnd of TLV partition at offset 0x{offset:04X}")
                print(f"\n✗ Invalid magic number 0x{tlv_header['magic']:08X}")
                print(f"   Expected: 0x{ESP_SECURE_CERT_TLV_MAGIC:08X}")
                break

            # Calculate offsets and sizes
            tlv_data_offset = offset + header_size
            padding_length = get_padding_length(tlv_header['length'])
            tlv_data_size = tlv_header['length'] + padding_length
            tlv_footer_offset = tlv_data_offset + tlv_data_size

            # Extract data
            tlv_data = data[tlv_data_offset:tlv_data_offset + tlv_header['length']]

            # Parse TLV footer
            tlv_footer, footer_size = parse_tlv_footer(data, tlv_footer_offset)

            # Verify TLV integrity
            is_valid, calculated_crc = verify_tlv_integrity(tlv_header, data[offset:], tlv_footer)

            # Store entry info
            entry = {
                'header': tlv_header,
                'data': tlv_data,
                'footer': tlv_footer,
                'is_valid': is_valid,
                'calculated_crc': calculated_crc,
                'offset': offset
            }
            tlv_entries.append(entry)

            # Print TLV entry details
            tlv_type = tlv_header['type']
            try:
                type_name = tlv_type_t(tlv_type).name
            except ValueError:
                type_name = f"UNKNOWN_{tlv_type}"

            print(f"\nTLV Entry #{tlv_count + 1}: {type_name}")
            print("-" * 60)
            print(f"Offset:     0x{offset:04X}")
            print(f"Type:       {tlv_type} ({type_name})")
            print(f"Subtype:    {tlv_header['subtype']}")
            print(f"Flags:      0x{tlv_header['flags']:02X}")
            print(f"Length:     {tlv_header['length']} bytes")
            print(f"Padding:    {padding_length} bytes")
            print(f"Total size: {get_tlv_total_length(tlv_header)} bytes")
            print(f"CRC32:      0x{tlv_footer['crc']:08X} ({'✓ VALID' if is_valid else '✗ INVALID'})")

            if not is_valid:
                print(f"Calculated: 0x{calculated_crc:08X}")

            # Format and display data
            if tlv_header['length'] > 0:
                print("Data:")
                formatted_data = format_tlv_data(tlv_type, tlv_header['flags'], tlv_data)
                for line in formatted_data.split('\n'):
                    print(f"    {line}")

            # Move to the next TLV entry
            total_tlv_size = get_tlv_total_length(tlv_header)
            offset += total_tlv_size
            tlv_count += 1

        except Exception as e:
            print(f"\n✗ Error parsing TLV at offset 0x{offset:04X}: {e}")
            break

    if tlv_count > 0:
        print_partition_summary(tlv_entries)
    else:
        print("\n✗ No valid TLV entries found in the file")
