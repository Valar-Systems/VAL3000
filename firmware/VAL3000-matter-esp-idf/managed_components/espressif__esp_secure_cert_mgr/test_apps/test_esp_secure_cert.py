# SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

"""
Test suite for ESP Secure Cert Manager.

This module contains pytest-based tests for validating the esp_secure_cert_mgr
component across different flash storage formats (TLV, NVS, custom flash) and
ESP32 targets. Tests are designed to run on both QEMU emulator and real
hardware.

Test Categories:
    - TLV Format Tests: Tests for the modern TLV (Type-Length-Value) format
    - Legacy Format Tests: Tests for legacy NVS and custom flash formats
    - QEMU Tests: Automated tests running on QEMU emulator
    - Hardware Tests: Tests running on physical ESP32 devices
"""

import pytest
import os
import glob
import hashlib
from typing import Any


# Flash memory offsets for partition table and secure cert partition
PARTITION_TABLE_OFFSET = 0xC000
SECURE_CERT_OFFSET = 0xD000


def write_bin_to_flash_image(dut: Any, bin_path: str, offset: int) -> None:
    """
    Write binary data to QEMU flash image at specified offset.

    This function modifies the flash_image.bin file used by QEMU to inject
    test data (certificates, keys, partition tables) at specific flash offsets.

    Args:
        dut: Device under test (pytest-embedded fixture)
        bin_path: Path to the binary file to write
        offset: Flash memory offset where data should be written

    Raises:
        AssertionError: If flash image is not found or is too small
    """
    flash_image_bin = os.path.join(dut.app.binary_path, 'flash_image.bin')
    assert os.path.exists(flash_image_bin), (
        f"Flash image not found: {flash_image_bin}"
    )

    # Read the binary data to be written
    with open(bin_path, 'rb') as f:
        data_to_write = f.read()

    # Read the current flash image
    with open(flash_image_bin, 'rb') as f:
        flash_data = bytearray(f.read())

    # Ensure flash image is large enough
    required_size = offset + len(data_to_write)
    if len(flash_data) < required_size:
        assert False, (
            f"Flash image is too small: {len(flash_data)} < {required_size}"
        )

    # Write the data at the specified offset
    flash_data[offset:offset + len(data_to_write)] = data_to_write

    # Write back to the flash image file
    with open(flash_image_bin, 'wb') as f:
        f.write(flash_data)


def read_bin_from_flash_image(dut: Any, offset: int, size: int) -> bytes:
    """
    Read binary data from QEMU flash image at specified offset.

    This function reads data from the flash_image.bin file used by QEMU
    to verify that data was written correctly.

    Args:
        dut: Device under test (pytest-embedded fixture)
        offset: Flash memory offset to read from
        size: Number of bytes to read

    Returns:
        bytes: The binary data read from flash image

    Raises:
        AssertionError: If flash image is not found
    """
    flash_image_bin = os.path.join(dut.app.binary_path, 'flash_image.bin')
    assert os.path.exists(flash_image_bin), (
        f"Flash image not found: {flash_image_bin}"
    )

    with open(flash_image_bin, 'rb') as f:
        f.seek(offset)
        return f.read(size)


def setup_flash_image_for_qemu(
    dut: Any, format: str = 'cust_flash_tlv', partition_table: bool = True
) -> None:
    """
    Prepare QEMU flash image with test data for specified format.

    This function sets up the QEMU flash image by:
    1. Writing the partition table to the correct offset
    2. Writing the secure cert data to the esp_secure_cert partition
    3. Verifying data integrity by reading back and comparing

    Args:
        dut: Device under test (pytest-embedded fixture)
        format: Flash format to use. Options:
            - 'cust_flash_tlv': TLV format in custom flash partition (default)
            - 'cust_flash': Legacy format in custom flash partition
            - 'nvs': Legacy format in NVS partition
            - 'nvs_legacy': Legacy NVS format
            - 'cust_flash_legacy': Legacy custom flash format
        partition_table: Whether to write partition table to flash
    Raises:
        pytest.fail: If setup fails or data verification fails
    """

    # Search for the binaries in the qemu_test directory
    secure_cert_bin = glob.glob(
        os.path.join('qemu_test', format, f'{format}.bin'),
        recursive=True
    )[0]
    if partition_table:
        partition_table_bin = os.path.join(
            'qemu_test', format, 'partition-table.bin'
        )
        assert os.path.exists(partition_table_bin), (
            f"Partition table not found: {partition_table_bin}"
        )

    assert os.path.exists(secure_cert_bin), (
        f"No {format}.bin found in qemu_test directory"
    )

    try:
        # Get the existing flash_image.bin from the build directory
        flash_image_bin = os.path.join(dut.app.binary_path, 'flash_image.bin')
        assert os.path.exists(flash_image_bin), (
            f"Flash image not found: {flash_image_bin}"
        )

    except Exception as e:
        pytest.fail(f"Unexpected error: {e}")

    try:
        # Write the partition table and secure cert data to the flash image
        if partition_table:
            write_bin_to_flash_image(
                dut, partition_table_bin, PARTITION_TABLE_OFFSET
            )

        write_bin_to_flash_image(dut, secure_cert_bin, SECURE_CERT_OFFSET)

        flash_img_size = os.path.getsize(flash_image_bin)
        expected_size = 2 * 1024 * 1024  # 2MB (set in the sdkconfig)
        print(f"flash_img_size {flash_img_size}")
        print(f"expected_size {expected_size}")
        assert flash_img_size == expected_size, (
            f"Flash image size {flash_img_size} is incorrect, "
            f"expected {expected_size}"
        )

        # Read the original partition table and secure cert data from the files
        # and check that they match the original data
        if partition_table:
            with open(partition_table_bin, 'rb') as f:
                original_partition = f.read()

            partition_size = os.path.getsize(partition_table_bin)
            partition_readback = read_bin_from_flash_image(
                dut, PARTITION_TABLE_OFFSET, partition_size
            )
            assert partition_readback == original_partition, (
                "Partition table data mismatch"
            )

        with open(secure_cert_bin, 'rb') as f:
            original_secure_cert = f.read()
        secure_cert_size = os.path.getsize(secure_cert_bin)
        secure_cert_readback = read_bin_from_flash_image(
            dut, SECURE_CERT_OFFSET, secure_cert_size
        )

        assert secure_cert_readback == original_secure_cert, (
            "esp_secure_cert data mismatch"
        )
    except Exception as e:
        pytest.fail(f"Unexpected error: {e}")


def verify_certificates_and_keys(
    dut: Any, format: str = 'cust_flash_tlv'
) -> None:
    """
    Verify certificates and keys read from esp_secure_cert partition.

    This function:
    1. Reads expected CA cert, device cert, and private key from input_data
    2. Calculates SHA256 hashes of expected data
    3. Extracts SHA256 hashes from device firmware logs
    4. Compares firmware hashes with expected hashes

    Args:
        dut: Device under test (pytest-embedded fixture)
        format: Flash format being tested (used to locate input data)

    Raises:
        pytest.fail: If verification fails or hashes don't match
    """
    print(f"Verifying certificates and keys for format: {format}")
    try:
        # Get the input data from the input_data directory
        input_data_dir = os.path.join(
            'qemu_test', format, 'input_data'
        )
        if os.path.isdir(input_data_dir):
            ca_cert_file = glob.glob(
                os.path.join(input_data_dir, 'ca_cert.pem')
            )[0]
            dev_cert_file = glob.glob(
                os.path.join(input_data_dir, 'device_cert.pem')
            )[0]
            priv_key = glob.glob(
                os.path.join(input_data_dir, 'priv_key.pem')
            )[0]
        else:
            pytest.fail(f"Input data directory not found: {input_data_dir}")

        # Read the CA Cert, Device Cert, and Private Key from the input_data
        # directory
        with open(ca_cert_file, 'rb') as f:
            ca_cert_data = f.read()
        ca_cert_sha256 = hashlib.sha256(ca_cert_data).hexdigest()

        with open(dev_cert_file, 'rb') as f:
            dev_cert_data = f.read()
        dev_cert_sha256 = hashlib.sha256(dev_cert_data).hexdigest()

        with open(priv_key, 'rb') as f:
            priv_key_data = f.read()
        priv_key_sha256 = hashlib.sha256(priv_key_data).hexdigest()
        print(f"length of priv_key_data: {len(priv_key_data)}")
        print(f"priv_key_sha256: {priv_key_sha256}")

        # Extract Device Cert SHA256 from the firmware log
        try:
            result = dut.expect(
                r'SHA256 of Device Cert: ([0-9a-fA-F]{64})', timeout=10
            )

            fw_device_cert_sha256 = result.group(1).decode('utf-8').lower()

            assert fw_device_cert_sha256 == dev_cert_sha256, (
                f"Device Cert SHA256 mismatch: firmware log="
                f"{fw_device_cert_sha256}, file={dev_cert_sha256}"
            )

        except Exception as e:
            pytest.fail(f"Could not extract Device Cert SHA256 from logs: {e}")

        # Extract CA Cert SHA256 from the firmware log
        try:
            result = dut.expect(
                r'SHA256 of CA Cert: ([0-9a-fA-F]{64})', timeout=10
            )
            fw_ca_cert_sha256 = result.group(1).decode('utf-8').lower()

            assert fw_ca_cert_sha256 == ca_cert_sha256, (
                f"CA Cert SHA256 mismatch: firmware log="
                f"{fw_ca_cert_sha256}, file={ca_cert_sha256}"
            )

        except Exception as e:
            pytest.fail(f"Could not extract CA Cert SHA256 from logs: {e}")

        # Extract Private Key SHA256 from the firmware log
        try:
            result = dut.expect(
                r'SHA256 of Private Key: ([0-9a-fA-F]{64})', timeout=10
            )
            fw_priv_key_sha256 = result.group(1).decode('utf-8').lower()

            assert fw_priv_key_sha256 == priv_key_sha256, (
                f"Private Key SHA256 mismatch: firmware log="
                f"{fw_priv_key_sha256}, file={priv_key_sha256}"
            )
        except Exception as e:
            pytest.fail(f"Could not extract Private Key SHA256 from logs: {e}")
    except Exception as e:
        pytest.fail(f"Unexpected error: {e}")


@pytest.mark.qemu
@pytest.mark.parametrize('config', ['legacy'], indirect=True)
@pytest.mark.parametrize('target', ['esp32c3'])
def test_esp_secure_cert_nvs_legacy_qemu(dut: Any) -> None:
    """
    Test legacy NVS format on QEMU emulator.

    This test validates the legacy NVS storage format for certificates and
    keys. Tests are run on QEMU to ensure CI/CD pipeline compatibility
    without hardware.

    Args:
        dut: Device under test fixture (QEMU emulator instance)
    """
    setup_flash_image_for_qemu(dut, 'nvs_legacy')
    verify_certificates_and_keys(dut, 'nvs_legacy')
    dut.expect(r'Tests finished, rc=0', timeout=10)


@pytest.mark.qemu
@pytest.mark.parametrize('config', ['legacy'], indirect=True)
@pytest.mark.parametrize('target', ['esp32c3'])
def test_esp_secure_cert_cust_flash_legacy_qemu(dut: Any) -> None:
    """
    Test legacy custom flash format on QEMU emulator.

    This test validates the legacy custom flash storage format for certificates
    and keys. Tests are run on QEMU for automated CI/CD testing.

    Args:
        dut: Device under test fixture (QEMU emulator instance)
    """
    setup_flash_image_for_qemu(dut, 'cust_flash_legacy')
    verify_certificates_and_keys(dut, 'cust_flash_legacy')
    dut.expect(r'Tests finished, rc=0', timeout=10)


@pytest.mark.qemu
@pytest.mark.parametrize('config', ['legacy'], indirect=True)
@pytest.mark.parametrize('target', ['esp32c3'])
def test_esp_secure_cert_cust_flash_qemu(dut: Any) -> None:
    """
    Test custom flash format on QEMU emulator.

    This test validates the custom flash storage format for certificates and
    keys. Tests are run on QEMU for CI/CD pipeline integration.

    Args:
        dut: Device under test fixture (QEMU emulator instance)
    """
    setup_flash_image_for_qemu(dut, "cust_flash")
    verify_certificates_and_keys(dut, "cust_flash")
    dut.expect(r'Tests finished, rc=0', timeout=10)


@pytest.mark.qemu
@pytest.mark.parametrize('config', ['legacy'], indirect=True)
@pytest.mark.parametrize('target', ['esp32c3'])
def test_esp_secure_cert_nvs_qemu(dut: Any) -> None:
    """
    Test NVS format on QEMU emulator.

    This test validates the NVS storage format for certificates and keys.
    Tests are run on QEMU for automated testing in CI/CD pipelines.

    Args:
        dut: Device under test fixture (QEMU emulator instance)
    """
    setup_flash_image_for_qemu(dut, "nvs")
    verify_certificates_and_keys(dut, "nvs")
    dut.expect(r'Tests finished, rc=0', timeout=10)


@pytest.mark.qemu
@pytest.mark.parametrize('config', ['tlv'], indirect=True)
@pytest.mark.parametrize('target', ['esp32', 'esp32c3', 'esp32s3'])
def test_esp_secure_cert_tlv_qemu(dut: Any) -> None:
    """
    Test TLV format on QEMU emulator.

    This test validates the modern TLV (Type-Length-Value) storage format for
    certificates and keys. This is the recommended format for new projects.
    Tests run on multiple ESP32 targets (esp32, esp32c3, esp32s3) using QEMU.

    Args:
        dut: Device under test fixture (QEMU emulator instance)
    """
    setup_flash_image_for_qemu(dut, partition_table=False)
    verify_certificates_and_keys(dut)
    dut.expect(r'Tests finished, rc=0', timeout=10)


@pytest.mark.parametrize('config', ['tlv'], indirect=True)
@pytest.mark.parametrize('target', ['esp32', 'esp32c3', 'esp32s3'])
def test_esp_secure_cert_tlv(dut: Any) -> None:
    """
    Test TLV format on real hardware.

    This test validates the TLV storage format on physical ESP32 devices.
    It verifies certificates and keys can be read correctly from the
    esp_secure_cert partition on actual hardware across multiple targets.

    Note: This test requires physical hardware and will be skipped in
    QEMU-only test runs (use -m "not qemu" to run hardware tests).

    Args:
        dut: Device under test fixture (real ESP32 device)
    """
    setup_flash_image_for_qemu(dut, partition_table=False)
    verify_certificates_and_keys(dut)
    dut.expect(r'Tests finished, rc=0', timeout=10)


@pytest.mark.qemu
@pytest.mark.parametrize('config', ['crypto'], indirect=True)
@pytest.mark.parametrize('target', ['esp32', 'esp32c3', 'esp32s3'])
def test_esp_secure_cert_crypto(dut: Any) -> None:
    """
    Test cryptographic operations on real hardware.

    This test validates the cryptographic operations on physical ESP32
    devices. It verifies the cryptographic operations can be performed
    correctly on actual hardware.

    Args:
        dut: Device under test fixture (real ESP32 device)
    """
    setup_flash_image_for_qemu(dut, partition_table=False)
    dut.expect(r'Tests finished, rc=0', timeout=10)


@pytest.mark.qemu
@pytest.mark.parametrize('config', ['basics'], indirect=True)
@pytest.mark.parametrize('target', ['esp32', 'esp32c3', 'esp32s3'])
def test_esp_secure_cert_basics(dut: Any) -> None:
    """
    Test basic operations on real hardware.

    This test validates the basic operations on physical ESP32 devices.
    It verifies the basic operations can be performed correctly on actual
    hardware.
    """
    setup_flash_image_for_qemu(dut, partition_table=False)
    dut.expect(r'Tests finished, rc=0', timeout=10)


# Secure Verification Test
@pytest.mark.qemu
@pytest.mark.parametrize('config', ['secure_verification'], indirect=True)
@pytest.mark.parametrize(
    'target,qemu_extra_args',
    [
        (
            'esp32c3',
            (
                '-drive file={},if=none,format=raw,id=efuse '
                '-global driver=nvram.esp32c3.efuse,'
                'property=drive,value=efuse '
                '-global driver=timer.esp32c3.timg,'
                'property=wdt_disable,value=true'
            ).format(
                os.path.join(
                    os.path.dirname(__file__),
                    "qemu_test",
                    "secure_verification",
                    "secure_key_0_esp32c3_efuse.bin"
                )
            ),
        ),
        (
            'esp32s3',
            (
                '-drive file={},if=none,format=raw,id=efuse '
                '-global driver=nvram.esp32s3.efuse,'
                'property=drive,value=efuse '
                '-global driver=timer.esp32s3.timg,'
                'property=wdt_disable,value=true'
            ).format(
                os.path.join(
                    os.path.dirname(__file__),
                    "qemu_test",
                    "secure_verification",
                    "secure_key_0_esp32s3_efuse.bin"
                )
            ),
        ),
    ],
    indirect=['qemu_extra_args'],
)
def test_esp_secure_cert_secure_verification_qemu(dut: Any) -> None:
    """
    Test secure verification operations on QEMU emulator.

    This test validates the secure verification operations on QEMU emulator.
    It verifies the secure verification operations can be performed correctly
    on QEMU emulator.

    Args:
        dut: Device under test fixture (QEMU emulator instance)
    """
    setup_flash_image_for_qemu(
        dut, 'secure_verification', partition_table=False
    )
    dut.expect(r'Tests finished, rc=0', timeout=10)


@pytest.mark.qemu
@pytest.mark.parametrize(
    'config',
    ['sec_verification_flash_encryption'],
    indirect=True
)
@pytest.mark.parametrize(
    'target,qemu_extra_args',
    [
        (
            'esp32c3',
            (
                '-drive file={},if=none,format=raw,id=efuse '
                '-global driver=nvram.esp32c3.efuse,'
                'property=drive,value=efuse '
                '-global driver=timer.esp32c3.timg,'
                'property=wdt_disable,value=true'
            ).format(
                os.path.join(
                    os.path.dirname(__file__),
                    "qemu_test",
                    "secure_verification_flash_encrypt",
                    "secure_key_0_esp32c3_efuse.bin"
                )
            ),
        ),
        (
            'esp32s3',
            (
                '-drive file={},if=none,format=raw,id=efuse '
                '-global driver=nvram.esp32s3.efuse,'
                'property=drive,value=efuse '
                '-global driver=timer.esp32s3.timg,'
                'property=wdt_disable,value=true'
            ).format(
                os.path.join(
                    os.path.dirname(__file__),
                    "qemu_test",
                    "secure_verification_flash_encrypt",
                    "secure_key_0_esp32s3_efuse.bin"
                )
            ),
        ),
    ],
    indirect=['qemu_extra_args'],
)
def test_esp_secure_cert_secure_verification_flash_encryption_qemu(dut):
    setup_flash_image_for_qemu(
        dut, 'secure_verification', partition_table=False
    )
    dut.expect(r'Tests finished, rc=0', timeout=10)


# Secure Verification Test for corrupted secure cert partition
@pytest.mark.qemu
@pytest.mark.parametrize(
    'config',
    ['secure_verification_corrupt_partition'],
    indirect=True
)
@pytest.mark.parametrize(
    'target,qemu_extra_args',
    [
        (
            'esp32c3',
            (
                '-drive file={},if=none,format=raw,id=efuse '
                '-global driver=nvram.esp32c3.efuse,'
                'property=drive,value=efuse '
                '-global driver=timer.esp32c3.timg,'
                'property=wdt_disable,value=true'
            ).format(
                os.path.join(
                    os.path.dirname(__file__),
                    "qemu_test",
                    "secure_verification",
                    "secure_key_0_esp32c3_efuse.bin"
                )
            ),
        ),
        (
            'esp32s3',
            (
                '-drive file={},if=none,format=raw,id=efuse '
                '-global driver=nvram.esp32s3.efuse,'
                'property=drive,value=efuse '
                '-global driver=timer.esp32s3.timg,'
                'property=wdt_disable,value=true'
            ).format(
                os.path.join(
                    os.path.dirname(__file__),
                    "qemu_test",
                    "secure_verification",
                    "secure_key_0_esp32s3_efuse.bin"
                )
            ),
        ),
    ],
    indirect=['qemu_extra_args'],
)
def test_esp_secure_cert_secure_verification_corrupt_partition_qemu(dut):
    setup_flash_image_for_qemu(
        dut, 'secure_verification_corrupt', partition_table=False
    )
    dut.expect(r'Tests finished, rc=0', timeout=10)


# Write functionality tests
@pytest.mark.qemu
@pytest.mark.parametrize('config', ['write'], indirect=True)
@pytest.mark.parametrize('target', ['esp32', 'esp32c3', 'esp32s3'])
def test_esp_secure_cert_write_qemu(dut: Any) -> None:
    """
    Test write operations on QEMU emulator.

    Tests TLV write, batch write, buffer mode, and error handling.

    Args:
        dut: Device under test fixture (QEMU emulator instance)
    """
    setup_flash_image_for_qemu(dut, partition_table=False)
    dut.expect(r'Tests finished, rc=0', timeout=60)
