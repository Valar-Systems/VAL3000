import pytest
import os
import glob

SECURE_CERT_OFFSET = 0xD000


def write_bin_to_flash_image(dut, bin_path, offset):
    """Write binary data to flash image at specified offset using file
    operations"""
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
        flash_data.extend(b'\x00' * (required_size - len(flash_data)))

    # Write the data at the specified offset
    flash_data[offset:offset + len(data_to_write)] = data_to_write

    # Write back to the flash image file
    with open(flash_image_bin, 'wb') as f:
        f.write(flash_data)


def read_bin_from_flash_image(dut, offset, size):
    """Read binary data from flash image at specified offset using file
    operations"""
    flash_image_bin = os.path.join(dut.app.binary_path, 'flash_image.bin')
    assert os.path.exists(flash_image_bin), (
        f"Flash image not found: {flash_image_bin}"
    )

    with open(flash_image_bin, 'rb') as f:
        f.seek(offset)
        return f.read(size)


def setup_flash_image_for_qemu(dut):

    # Search for the binaries in the qemu_test directory
    secure_cert_bin = glob.glob(
        os.path.join('tests', 'esp_secure_cert.bin'),
        recursive=True
    )[0]

    assert os.path.exists(secure_cert_bin), (
        "No esp_secure_cert.bin found in tests directory"
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
        write_bin_to_flash_image(
            dut, secure_cert_bin, SECURE_CERT_OFFSET
        )

        flash_img_size = os.path.getsize(flash_image_bin)
        expected_size = 2 * 1024 * 1024  # 2MB (set in the sdkconfig)
        assert flash_img_size == expected_size, (
            f"Flash image size {flash_img_size} is incorrect, "
            f"expected {expected_size}"
        )

        with open(secure_cert_bin, 'rb') as f:
            original_secure_cert = f.read()

        secure_cert_size = os.path.getsize(secure_cert_bin)

        secure_cert_readback = read_bin_from_flash_image(
            dut, SECURE_CERT_OFFSET, secure_cert_size
        )

        # Check that the partition table and secure cert data match the
        # original data
        assert secure_cert_readback == original_secure_cert, (
            "esp_secure_cert data mismatch"
        )
    except Exception as e:
        pytest.fail(f"Unexpected error: {e}")


@pytest.mark.qemu
@pytest.mark.parametrize('target', ['esp32c3', 'esp32'], indirect=True)
@pytest.mark.parametrize('config', ['default'], indirect=True)
def test_esp_secure_cert_sanity_qemu(dut):
    """Simple sanity check to verify the application is working"""
    setup_flash_image_for_qemu(dut)
    dut.expect(
        r'Successfully obtained and verified the contents of '
        r'esp_secure_cert partition',
        timeout=10
    )


@pytest.mark.parametrize('target', ['esp32c3', 'esp32'], indirect=True)
@pytest.mark.parametrize('config', ['default'], indirect=True)
def test_esp_secure_cert_sanity(dut):
    """Simple sanity check to verify the application is working"""
    dut.expect(
        r'Successfully obtained and verified the contents of '
        r'esp_secure_cert partition',
        timeout=10
    )
