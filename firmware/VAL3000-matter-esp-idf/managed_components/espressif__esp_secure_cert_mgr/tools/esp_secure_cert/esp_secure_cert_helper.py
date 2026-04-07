from typing import Dict
from cryptography.hazmat.primitives import serialization
from cryptography.x509 import (
    load_pem_x509_certificate,
    load_der_x509_certificate
)

import os

esp_secure_cert_data_dir = 'esp_secure_cert_data'

def load_private_key(key_file_path: str,
                     password: str = None) -> Dict[str, str]:
    """
    Load a private key from a file in either PEM or DER format.

    Args:
        key_file_path (str): Path to the private key file.
        password (str): Password to decrypt the private key,
                        if it is encrypted.
    Returns:
        Dict[str, str]: A dictionary with the `"encoding"` and `"bytes"` keys.
        The `"encoding"` key holds a value
        of type `str` (a member of the `serialization.Encoding` enum)
        and the `"bytes"` key holds a value of type `bytes`.

    Raises:
        FileNotFoundError: If the private key file cannot be found or read.
        ValueError: If the private key file is not in PEM or DER format.

    """
    result = {}

    try:
        with open(key_file_path, "rb") as key_file:
            key = key_file.read()
    except FileNotFoundError:
        raise FileNotFoundError(f"Key file not found: {key_file_path}")

    try:
        # Attempt to load the key as a PEM-encoded private key
        private_key = serialization.load_pem_private_key(
                key,
                password=password)

        result["encoding"] = serialization.Encoding.PEM.value
        key_encoding = serialization.Encoding.PEM
        key_enc_alg = serialization.NoEncryption()
        priv_key_format = serialization.PrivateFormat.TraditionalOpenSSL

        result["bytes"] = private_key.private_bytes(
            encoding=key_encoding,
            format=priv_key_format,
            encryption_algorithm=key_enc_alg
        )
        result["key_instance"] = private_key
        return result
    except ValueError:
        pass

    try:
        private_key = serialization.load_der_private_key(
            key,
            password=password
        )
        result["encoding"] = serialization.Encoding.DER.value
        key_encoding = serialization.Encoding.DER
        priv_key_format = serialization.PrivateFormat.TraditionalOpenSSL
        key_enc_alg = serialization.NoEncryption()
        result["bytes"] = private_key.private_bytes(
            encoding=key_encoding,
            format=priv_key_format,
            encryption_algorithm=key_enc_alg
        )
        result["key_instance"] = private_key
        return result
    except ValueError:
        raise ValueError("Unsupported key encoding format,"
                         " Please provide PEM or DER encoded key")


def convert_der_key_to_pem(key_file_path: str, password: str = None) -> bytes:
    """
    Convert a key from DER format to PEM format, or return the PEM key as-is.
    """
    with open(key_file_path, 'rb') as key_file:
        key_data = key_file.read()

    try:
        # First, try to load the key as a PEM-encoded private key
        private_key = serialization.load_pem_private_key(
            key_data,
            password=password
        )
        # If successful, return the original PEM data
        return key_data
    except ValueError:
        pass  # If it fails, it might be a DER key, so continue

    try:
        # Attempt to load the key as a DER-encoded private key
        private_key = serialization.load_der_private_key(
            key_data,
            password=password
        )
    except ValueError:
        raise ValueError("Unsupported key encoding format. "
                         "Please provide a PEM or DER encoded key.")

    # Convert the DER key to PEM format
    pem_key = private_key.private_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PrivateFormat.TraditionalOpenSSL,
        encryption_algorithm=serialization.NoEncryption()
    )

    return pem_key


def load_certificate(cert_file_path: str) -> Dict[str, str]:
    """
    Load a certificate from a file in either PEM or DER format.

    Args:
        cert_file_path (str): The path to the certificate file.

    Returns:
        Dict[str, str]: A dictionary with the `"encoding"` and `"bytes"` keys.
        The `"encoding"` key holds a value of
        type `str` (a member of the `serialization.Encoding enum)
        and the `"bytes"` key holds a value of type `bytes`.

    Raises:
        FileNotFoundError: If the certificate file cannot be found or read.
        ValueError: If the certificate file is not in PEM or DER format.
    """
    result = {}

    if (cert_file_path is None):
        result["encoding"] = None
        result["bytes"] = b''
        return result

    try:
        with open(cert_file_path, "rb") as cert_file:
            cert_data = cert_file.read()
    except FileNotFoundError:
        raise FileNotFoundError(f"Cert file not found: {cert_file_path}")

    try:
        cert = load_pem_x509_certificate(cert_data)
        result["encoding"] = serialization.Encoding.PEM.value
        cert_encoding = serialization.Encoding.PEM
        result["bytes"] = cert.public_bytes(encoding=cert_encoding)
        result["cert_instance"] = cert
        return result
    except ValueError:
        pass

    try:
        cert = load_der_x509_certificate(cert_data)
        result["encoding"] = serialization.Encoding.DER.value
        cert_encoding = serialization.Encoding.DER
        result["bytes"] = cert.public_bytes(encoding=cert_encoding)
        result["cert_instance"] = cert
        return result
    except ValueError:
        raise ValueError("Unsupported certificate encoding format,"
                         "Please provide PEM or DER encoded certificate")

def get_efuse_key_file(efuse_key_spec):
    """
    Get efuse key file path:
    - None or empty: return None (auto-generate)
    - file path: return path if exists
    - otherwise: return None (auto-generate)
    """
    if not efuse_key_spec:
        return None

    if os.path.exists(efuse_key_spec):
        print(f"Using custom efuse key file: {efuse_key_spec}")
        return efuse_key_spec
    else:
        print(f"Warning: efuse key file '{efuse_key_spec}' not found, using auto-generated key")
        return None
