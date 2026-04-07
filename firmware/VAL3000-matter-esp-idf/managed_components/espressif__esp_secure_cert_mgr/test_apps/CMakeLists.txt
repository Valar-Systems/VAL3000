# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_esp_secure_cert)

# Flash the custom partition named `esp_secure_cert` ONLY
# DO NOT flash the partition table as it will corrupt the boot process
set(partition esp_secure_cert)
idf_build_get_property(project_dir PROJECT_DIR)

# Set the secure cert image file
set(secure_cert_image ${project_dir}/qemu_test/cust_flash_tlv/cust_flash_tlv.bin)

# Get partition offset for esp_secure_cert from the existing partition table
partition_table_get_partition_info(secure_cert_offset "--partition-name ${partition}" "offset")

# Flash the secure cert data to the esp_secure_cert partition
if(EXISTS ${secure_cert_image})
    esptool_py_flash_target_image(flash "${partition}" "${secure_cert_offset}" "${secure_cert_image}")
    message(STATUS "Flashed ${secure_cert_image} to ${partition} at offset ${secure_cert_offset}")
else()
    message(WARNING "Secure cert image not found: ${secure_cert_image}")
endif()

# Note: We do NOT flash the partition-table.bin as it would overwrite the valid
# partition table defined in partitions.csv and cause boot failures
