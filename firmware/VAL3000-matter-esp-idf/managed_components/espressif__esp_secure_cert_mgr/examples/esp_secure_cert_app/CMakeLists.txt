# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_secure_cert_app)

if(CONFIG_EXAMPLE_ESP_SECURE_CERT_ENABLE_CI)
    # Flash the custom partition named `esp_secure_cert`
    set(partition esp_secure_cert)
    idf_build_get_property(project_dir PROJECT_DIR)
    set(image_file ${project_dir}/tests/esp_secure_cert.bin)
    partition_table_get_partition_info(offset "--partition-name ${partition}" "offset")
    if(EXISTS ${image_file})
        esptool_py_flash_target_image(flash "${partition}" "${offset}" "${image_file}")
        message(STATUS "Flashed ${image_file} to ${partition} at offset ${offset}")
    else()
        message(WARNING "Secure cert image not found: ${image_file}")
    endif()
endif()
