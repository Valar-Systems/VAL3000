set(src_dir "")

if(CONFIG_TEST_ESP_SECURE_CERT_SECURE_VERIFICATION)
    list(APPEND src_dir "secure_verification")
endif()

if(CONFIG_TEST_ESP_SECURE_CERT_TLV_LEGACY)
    list(APPEND src_dir "tlv_legacy")
endif()

if(CONFIG_TEST_ESP_SECURE_CERT_CRYPTO)
    list(APPEND src_dir "crypto")
endif()

if(CONFIG_TEST_ESP_SECURE_CERT_BASICS)
    list(APPEND src_dir "basics")
endif()

if(CONFIG_TEST_ESP_SECURE_CERT_WRITE)
    list(APPEND src_dir "write")
endif()

# Check IDF version for WHOLE_ARCHIVE support (available from IDF 5.0)
if(IDF_VERSION_MAJOR GREATER_EQUAL 5)
    idf_component_register(
        SRC_DIRS "." ${src_dir}
        INCLUDE_DIRS "." "../../private_include"
        PRIV_REQUIRES unity efuse bootloader_support nvs_flash
        WHOLE_ARCHIVE
    )
else()
    # For IDF 4.x, register without WHOLE_ARCHIVE
    idf_component_register(
        SRC_DIRS "." ${src_dir}
        INCLUDE_DIRS "." "../../private_include"
        PRIV_REQUIRES unity efuse bootloader_support nvs_flash
    )
endif()
