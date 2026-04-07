# ESP Secure Cert Partition Parsing

To parse an existing ESP Secure Cert partition binary, use the `configure_esp_secure_cert.py` script with the `--parse_bin` option. This will extract all available information from the esp_secure_cert partition.

### Example Command

```
configure_esp_secure_cert.py --parse_bin esp_secure_cert_data/esp_secure_cert.bin
```

- Replace `esp_secure_cert_data/esp_secure_cert.bin` with the path to your partition binary if different.

## What Happens During Parsing?

1. **A new folder `esp_secure_cert_parsed_data/` is created** in the current directory. This folder will contain all extracted files and metadata.
2. **For each TLV entry, the following steps are performed:**
   - **CA Certificate / Device Certificate:**
     - Extracted and saved as PEM or DER files (e.g., `cacert_0_<subtype>.pem`, `devcert_1_<subtype>.pem`).
   - **Private Key:**
     - If the key is plaintext, it is extracted and saved as a PEM or DER file (e.g., `privkey_2_<subtype>.pem`).
     - If the key uses RSA DS (Digital Signature peripheral), the following are extracted:
       - `ds_data_3_<subtype>.bin` (encrypted key data)
       - `ds_context_4_<subtype>.bin` (context for hardware DS)
     - If the key uses ECDSA peripheral, the following is extracted:
       - `sec_cfg_6_<subtype>.bin` (security config for hardware ECDSA)
   - **Custom User Data:**
     - No file is generated for custom data entries. The data is included in the CSV as a string or hex value.

3. **A CSV file `esp_secure_cert_parsed.csv` is generated** in the output folder. This CSV contains a row for each TLV entry, mapping the TLV type, subtype, data value (filename or inline data), data type, key type, algorithm, key size, efuse ID, and efuse key file (if applicable). This CSV can be used to regenerate or re-flash the partition.

4. **A raw TLV metadata file `tlv_entries_raw.txt` is generated** in the output folder. This file contains a detailed summary of all TLV entries, including their offsets, types, lengths, CRCs, and a human-readable dump of their contents. This is useful for debugging and auditing the partition.

## Output Folder Structure

After running the parse command, you will find:

```
esp_secure_cert_parsed_data/
├── cacert_0_<subtype>.pem           # CA certificate (if present)
├── devcert_1_<subtype>.pem          # Device certificate (if present)
├── privkey_2_<subtype>.pem          # Private key (if plaintext)
├── ds_data_3_<subtype>.bin          # Encrypted DS data (if RSA DS used)
├── ds_context_4_<subtype>.bin       # DS context (if RSA DS used)
├── sec_cfg_6_<subtype>.bin          # Security config (if ECDSA peripheral used)
├── esp_secure_cert_parsed.csv       # CSV with all TLV entries
└── tlv_entries_raw.txt              # Raw TLV metadata and summary
```

- **Not all files will be present in every partition.** The actual files depend on the contents and configuration of your partition.
- **Custom data entries** are included in the CSV but do not generate separate files.
