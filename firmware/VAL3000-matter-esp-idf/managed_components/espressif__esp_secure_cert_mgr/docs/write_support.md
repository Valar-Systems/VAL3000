# ESP Secure Cert Write Support

This document describes the write support architecture for the `esp_secure_cert` partition.

## Overview

The write APIs enable runtime modification of the `esp_secure_cert` partition, supporting:
- Direct flash writes (on-device provisioning)
- Buffer writes (host-side partition generation)
- HMAC-based encryption and key derivation

## Write Flow

```mermaid
flowchart TD
    A[esp_secure_cert_append_tlv] --> B{Validate TLV Info}
    B -->|Invalid| C[Return Error]
    B -->|Valid| D{Write Mode?}

    D -->|FLASH| E[Acquire Write Lock]
    E -->|Locked| F[Return WRITE_IN_PROGRESS]
    E -->|OK| G[Unmap Partition]
    G --> H[Prepare TLV Buffer]
    H --> I{Check Duplicate?}
    I -->|Exists| J[Return TLV_ALREADY_EXISTS]
    I -->|OK| K[Get Next Write Offset]
    K --> L{Erase Check?}
    L -->|Yes & Not Erased| M{Auto Erase?}
    M -->|Yes| N[Erase Partition]
    M -->|No| O[Return FLASH_NOT_ERASED]
    L -->|No or Erased| P[Write to Flash]
    N --> P
    P --> Q[Verify Write]
    Q --> R[Unmap Partition]
    R --> S[Release Lock]
    S --> T[Return OK]

    D -->|BUFFER| U[Write to Buffer]
    U --> V[Update bytes_written]
    V --> T
```

## TLV Structure

Each TLV entry written to flash:

```
┌─────────────────────────────────────────────────────────────┐
│                    TLV Header (12 bytes)                    │
├─────────┬────────┬────────┬──────────┬─────────┬───────────┤
│  Magic  │ Flags  │  Type  │ Subtype  │ Length  │ Reserved  │
│ 4 bytes │ 1 byte │ 1 byte │  1 byte  │ 2 bytes │  3 bytes  │
├─────────┴────────┴────────┴──────────┴─────────┴───────────┤
│                     Data (variable)                         │
├─────────────────────────────────────────────────────────────┤
│                 Padding (16-byte aligned)                   │
├─────────────────────────────────────────────────────────────┤
│                   TLV Footer (4 bytes)                      │
│                       CRC32                                 │
└─────────────────────────────────────────────────────────────┘
```

## Write Modes

```mermaid
flowchart LR
    subgraph Flash Mode
        A1[Device Flash] --> B1[esp_partition_write]
        B1 --> C1[Verify Read-back]
        C1 --> D1[Unmap Cache]
    end

    subgraph Buffer Mode
        A2[Memory Buffer] --> B2[memcpy]
        B2 --> C2[Update Offset]
    end
```

| Mode | Use Case | Verification |
|------|----------|--------------|
| `FLASH` | On-device provisioning | Read-back + CRC |
| `BUFFER` | Host partition generation | None (in-memory) |

## Concurrency Control

```mermaid
sequenceDiagram
    participant T1 as Task 1
    participant Lock as atomic_bool
    participant Flash as Flash

    T1->>Lock: atomic_compare_exchange(false→true)
    Lock-->>T1: OK (acquired)
    T1->>Flash: Write TLV
    Flash-->>T1: Done
    T1->>Lock: atomic_store(false)

    Note over T1,Lock: Non-blocking fail-fast lock
```

The write lock uses `atomic_compare_exchange_strong()` for:
- Thread-safe access without OS primitives
- Non-blocking fail-fast behavior
- Minimal overhead (1 byte)

## HMAC-based ECDSA Key Derivation

```mermaid
flowchart TD
    subgraph Write Time
        A[Generate Salt] --> B[Write Salt TLV]
        B --> C[Write Key Marker TLV]
        C --> D[Set DERIVATION Flag]
    end

    subgraph Read Time
        E[Read Salt TLV] --> F[Read Key Marker]
        F --> G{DERIVATION Flag?}
        G -->|Yes| H[PBKDF2-HMAC-SHA256]
        H --> I[2048 iterations]
        I --> J[32-byte Raw Key]
        J --> K[Convert to DER]
        K --> L[Return 121-byte Key]
        G -->|No| M[Return Stored Key]
    end

    style H fill:#f96
    style I fill:#f96
```

**Key never stored in flash** - derived on-demand from:
- Salt (stored in partition)
- HMAC key (stored in eFuse, inaccessible to software)

## Batch Write Optimization

```mermaid
flowchart TD
    A[Validate All Entries] --> B[Acquire Lock Once]
    B --> C[Check Total Space]
    C --> D[Write Entry 1]
    D --> E[Unmap - Refresh Cache]
    E --> F[Write Entry 2]
    F --> G[Unmap - Refresh Cache]
    G --> H[...]
    H --> I[Write Entry N]
    I --> J[Release Lock]
    J --> K[Final Unmap]
```

Benefits:
- Single lock acquisition for all entries
- Upfront validation prevents partial writes
- Cache refresh between entries ensures correct offsets

## Error Code Categories

| Range | Category |
|-------|----------|
| `0x7001-0x7005` | TLV validation errors |
| `0x700A-0x700F` | Partition/flash errors |
| `0x7014-0x7017` | Config/buffer errors |
| `0x701E-0x7020` | HMAC encryption errors |
| `0x7028-0x7029` | Memory errors |
| `0x7032` | Concurrency error |
| `0x703C-0x7041` | ECDSA derivation errors |

## Configuration Structure

```c
typedef struct {
    esp_secure_cert_write_mode_t mode;
    union {
        struct {
            bool check_erase;    // Verify erased before write
            bool auto_erase;     // Auto-erase if needed
        } flash;
        struct {
            uint8_t *buffer;     // Target buffer
            size_t buffer_size;  // Buffer capacity
            size_t *bytes_written; // Output: actual size
        } buffer;
    };
    uint32_t reserved[4];        // Future extensions
} esp_secure_cert_write_config_t;
```

The `reserved` field ensures API/ABI compatibility for future enhancements.

## See Also

- [TLV Format](format.md) - Partition format details
- [esp_secure_cert_write.h](../include/esp_secure_cert_write.h) - API reference
- [README.md](../README.md) - Usage examples
