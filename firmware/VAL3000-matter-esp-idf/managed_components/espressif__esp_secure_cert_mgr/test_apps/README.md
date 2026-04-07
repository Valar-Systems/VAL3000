# ESP Secure Cert Manager Test Applications

This directory contains **test applications** specifically designed for testing the `esp_secure_cert_mgr` component. These applications are **NOT for production use** - they are intended solely for **testing and validation purposes**.

## Purpose

The test applications in this directory serve to:

- **Test TLV format support** of the `esp_secure_cert_mgr` component
- **Test legacy format support** (cust_flash, nvs) 
- **Validate component functionality** through automated testing
- **Provide test infrastructure** for CI/CD pipelines
- **Enable QEMU-based testing** for different flash formats

## Testing Overview

The test application is designed to run on **both QEMU emulator and real hardware**:

- **QEMU Testing**: Used for CI/CD pipelines and development testing without physical hardware
- **Real Hardware Testing**: Used for final validation on actual ESP32 devices

## Running Tests

### Prerequisites

1. **Install dependencies**:
   ```bash
   cd test_apps
   pip install -r qemu_test/requirements.txt
   ```

2. **Build the test application**:
   ```bash
   idf.py build
   ```

### QEMU Testing

For running tests on QEMU emulator (recommended for development and CI):

```bash
pytest --target $IDF_TARGET \
       --app-path $ROOT_PATH/test_apps \
       --build-dir $BUILD_DIR \
       --embedded-services idf,qemu \
       -s -m qemu
```

**Example with specific values**:
```bash
pytest --target esp32c3 \
       --app-path /path/to/test_apps \
       --build-dir build_esp32c3_tlv \
       --embedded-services idf,qemu \
       -s -m qemu
```

**What this does**:
- Uses QEMU emulator instead of real hardware
- Automatically sets up flash image with test data
- Runs tests marked with `@pytest.mark.qemu`

### Real Hardware Testing

For running tests on actual ESP32 hardware:

```bash
pytest --target $IDF_TARGET \
       --app-path $ROOT_PATH/test_apps \
       --build-dir $BUILD_DIR \
       --embedded-services esp,idf \
       -s \
       -m "not qemu"
```

**Example with specific values**:
```bash
pytest --target esp32c3 \
       --app-path /path/to/test_apps \
       --build-dir build_esp32c3_tlv \
       --embedded-services esp,idf \
       -s \
       -m "not qemu"
```

**What this does**:
- Runs on real ESP32 hardware
- Requires physical device connection
- Uses ESP-IDF's embedded services for hardware interaction

### Test Markers

The test suite uses pytest markers to categorize tests:

- **`@pytest.mark.qemu`**: Tests designed to run on QEMU emulator
- **`@pytest.mark.parametrize('target', [...])`**: Tests that run on multiple ESP32 targets
- **`@pytest.mark.parametrize('config', [...])`**: Tests that run with different configurations

## Directory Structure

```
test_apps/
├── main/                           # Main test application source
│   ├── app_main.c                  # Test application entry point
│   └── CMakeLists.txt             # Component build configuration
├── qemu_test/                      # QEMU testing infrastructure
│   ├── cust_flash_tlv/            # TLV format test data
│   │   ├── cust_flash_tlv.bin     # Pre-built TLV flash image
│   │   ├── partition-table.bin    # Custom partition table
│   │   └── input_data/            # Source certificates and keys
│   ├── cust_flash/                # Legacy cust_flash format test data
│   ├── nvs/                       # Legacy NVS format test data
│   └── nvs_legacy/                # Legacy NVS format test data
├── test_esp_secure_cert.py        # Test script
├── sdkconfig.ci.tlv                  # TLV format configuration
├── sdkconfig.ci.legacy               # Legacy format configuration
├── sdkconfig.defaults             # Default configuration
├── partitions.csv                  # Custom partition table
├── CMakeLists.txt                 # Project build configuration
└── README.md                      # This file
```
\
