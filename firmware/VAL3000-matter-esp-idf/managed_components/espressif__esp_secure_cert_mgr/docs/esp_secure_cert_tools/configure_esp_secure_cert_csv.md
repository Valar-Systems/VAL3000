# CSV Format in configure_esp_secure_cert.py

## Overview
The CSV format allows you to configure esp_secure_cert partition on devices. For this, user needs to provide the CSV file
containing all the configurations and data, that need to store in esp_secure_cert partition. You can provide this CSV file to configure_esp_secure_cert.py tool to generate appropriate esp_secure_cert partition.

## How to use CSV format

### Fields of CSV file

- The CSV file should follow this format:

`tlv_type,tlv_subtype,data_value,data_type,priv_key_type,algorithm,key_size,efuse_id,efuse_key`

1. **tlv_type**: Type of the TLV entry
    - The following are the TLV types you can use to associate data in your CSV file:
        - `ESP_SECURE_CERT_CA_CERT_TLV` (0): CA certificate
        - `ESP_SECURE_CERT_DEV_CERT_TLV` (1): Device certificate
        - `ESP_SECURE_CERT_PRIV_KEY_TLV` (2): Private key
        - `ESP_SECURE_CERT_USER_DATA_1_TLV` (51): User custom data 1
        - `ESP_SECURE_CERT_USER_DATA_2_TLV` (52): User custom data 2
        - `ESP_SECURE_CERT_USER_DATA_3_TLV` (53): User custom data 3
        - `ESP_SECURE_CERT_USER_DATA_4_TLV` (54): User custom data 4
        - `ESP_SECURE_CERT_USER_DATA_5_TLV` (55): User custom data 5
    
    **NOTE - Both string version i.e ESP_SECURE_CERT_CA_CERT_TLV or the numerical version i.e. 0, 1, 2 are supported**    

    - You can use these TLV types in the `tlv_type` column of your CSV to specify what kind of data you are associating.
2. **tlv_subtype**: Subtype of the TLV entry.
    - This is an integer value ranging from 0 up to 254.
    - Used to distinguish between multiple entries of the same `tlv_type`.
    - If you need to store more than one entry of the same type, use different subtypes to differentiate them.
3. **data_value**: The actual data or relative path to data file.
4. **data_type**: Specifies the format of the data provided. The supported data types are:
    - `file`: Path to a file containing the data.
    - `string`: The data itself as a string (for example, the contents of a PEM file), enclosed in double quotes. You can either paste the entire PEM content in a single line with `\n` for newlines, or copy-paste the multi-line PEM content directly inside double quotes.
    - `hex`: Data provided as a hexadecimal string.
    - `base64`: Data provided as a base64-encoded string.

        **Important:**  
        - For `priv_key`, `ca_cert`, and `device_cert` TLV types, only `file` and `string` are supported as `data_type`.  
            - For the `file` type, provide the relative path to your PEM file or DER file.
            - For the `string` type, you can either:
                - **Paste the PEM content directly inside double quotes (multi-line is allowed), or**
                - **Paste the PEM content in a single line without any `\n` characters in the string, all inside double quotes.**
        - For all other TLV types (such as custom data) other than `priv_key`, `ca_cert` and `device_cert`, all data types (`file`, `string`, `hex`, `base64`) are supported.
5. **priv_key_type**: Specifies how the private key is stored and used. There are three supported types:
    - `plaintext`: The private key will be stored directly in flash as plaintext.
    - `rsa_ds`: The private key will be used with the Digital Signature (DS) peripheral. This requires that the DS peripheral is supported and enabled on your chip.
    - `ecdsa_peripheral`: The private key will be used with the ECDSA peripheral. This requires that the ECDSA peripheral is supported and enabled on your chip.

    **NOTE - If you select `rsa_ds` or `ecdsa_peripheral`, the corresponding hardware peripheral must be available and enabled for your target chip. If you select `plaintext`, the key will be stored in flash as plaintext.**
6. **algorithm**: Cryptographic algorithm to use. (for eg., `RSA` or `ECDSA`) 
7. **key_size**: Size of the cryptographic key (for eg., `2048` for `RSA` or `256` for `ECDSA`)
8. **efuse_id**: ID of the efuse to use.
9. **efuse_key**: The efuse key relative file path. `If the path is not provided, then it will auto generate the efuse key`. **(Only file format is supported here).**

## Custom Data Support
You can now store custom data directly in the esp_secure_cert partition by including it in your CSV file. This allows you to persist user-defined configuration or metadata alongside certificates and keys, using the available custom TLV types (e.g., ESP_SECURE_CERT_USER_DATA_1_TLV, etc.).

## Examples of CSV

**For TLV types other than the private key, the columns `priv_key_type`, `algorithm`, `key_size`, `efuse_id`, and `efuse_key` should be left empty.**

**1. Plaintext key CSV example (plaintext)**

The CSV file for configuring the ESP Secure Cert partition must have the following columns, in order:

```
tlv_type,tlv_subtype,data_value,data_type,priv_key_type,algorithm,key_size,efuse_id,efuse_key
ESP_SECURE_CERT_CA_CERT_TLV,0,cacert.pem,file,,,,,
ESP_SECURE_CERT_DEV_CERT_TLV,0,client.crt,file,,,,,
ESP_SECURE_CERT_PRIV_KEY_TLV,0,client.key,file,plaintext,RSA,2048,,
```

Here, RSA key stored as the plaintext of size 2048.

**2. ECDSA key CSV example (ECDSA peripheral)**

```
tlv_type,tlv_subtype,data_value,data_type,priv_key_type,algorithm,key_size,efuse_id,efuse_key
ESP_SECURE_CERT_CA_CERT_TLV,0,ecdsa_cacert.pem,file,,,,,
ESP_SECURE_CERT_DEV_CERT_TLV,0,ecdsa_client.crt,file,,,,,
ESP_SECURE_CERT_PRIV_KEY_TLV,0,ecdsa_client.key,file,ecdsa_peripheral,ECDSA,256,1,ecdsa_efuse.key
```

Here, ECDSA key is stored as private key, using ECDSA peripheral. `ecdsa_efuse.key` is give as efuse_key. If it is not provided then, it will be auto-generated by the software.

**3. RSA key CSV example (DS peripheral)**

```
tlv_type,tlv_subtype,data_value,data_type,priv_key_type,algorithm,key_size,efuse_id,efuse_key
ESP_SECURE_CERT_CA_CERT_TLV,0,cacert.pem,file,,,,,
ESP_SECURE_CERT_DEV_CERT_TLV,0,client.crt,file,,,,,
ESP_SECURE_CERT_PRIV_KEY_TLV,0,client.key,file,rsa_ds,RSA,2048,1,
```

Here, RSA key is stored as private key, using DS peripheral.

**4. Custom data CSV example**

```
tlv_type,tlv_subtype,data_value,data_type,priv_key_type,algorithm,key_size,efuse_id,efuse_key
ESP_SECURE_CERT_USER_DATA_1_TLV,0,"Device Model: ESP32-S3-DevKit-C",string,,,,
ESP_SECURE_CERT_USER_DATA_2_TLV,0,DEADBEEFCAFEBABE1234567890ABCDEF,hex,,,,
```

Here, `Device Model` and `DEADBEEFCAFEBABE1234567890ABCDEF1` is stored as custom data with `string` and `hex` as data_type respectively.

**5. Key as string CSV example**

#### Fig. 1
```
tlv_type,tlv_subtype,data_value,data_type,priv_key_type,algorithm,key_size,efuse_id,efuse_key
ESP_SECURE_CERT_PRIV_KEY_TLV,0,"-----BEGIN PRIVATE KEY-----MIIEvg......iPbeKNca-----END PRIVATE KEY-----",string,plaintext,RSA,2048,1,
```

#### Fig. 2
```
ESP_SECURE_CERT_PRIV_KEY_TLV,0,"-----BEGIN EC PRIVATE KEY-----
MHcCAQEEIKLY3NrF5q4bw/TqTE0a4MGI70bitMzz2AoLIIcMeHi8oAoGCCqGSM49
AwEHoUQDQgAETd9kH9hu1IunygTHEbhF2gz/X3rOWhH5lVUIJoWI6agksE3Mv86a
bqCKthpA8nFqUKj9qiyAoLxzoejsrywlNA==
-----END EC PRIVATE KEY-----",string,ecdsa_peripheral,ECDSA,256,1,
```

Here, RSA key used as private key, has provided as string. For you can provide the string as single line without containing any `\n` in it as shown in Fig. 1 or just directly paste the key as it is as shown Fig. 2.
