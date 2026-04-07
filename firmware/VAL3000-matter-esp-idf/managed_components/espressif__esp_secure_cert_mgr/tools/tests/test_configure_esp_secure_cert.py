#!/usr/bin/env python
# SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Apache-2.0

import hashlib
import os
import shutil
import subprocess
import sys
import tempfile
import unittest


class ConfigureEspSecureCertTest(unittest.TestCase):
    """Test class for configure_esp_secure_cert.py script functionality"""

    def setUp(self):
        """Set up test environment before each test"""
        # Get the directory paths
        self.test_dir = os.path.dirname(os.path.abspath(__file__))
        self.tools_dir = os.path.dirname(self.test_dir)
        self.script_path = os.path.join(self.tools_dir, 'configure_esp_secure_cert.py')
        self.input_data_dir = os.path.join(self.test_dir, 'input_data')

        # Input files
        self.csv_file = os.path.join(self.input_data_dir, 'esp_secure_cert_config_examples.csv')
        self.expected_bin = os.path.join(self.input_data_dir, 'esp_secure_cert.bin')

        # Create a temporary working directory
        self.temp_dir = tempfile.mkdtemp()
        self.original_cwd = os.getcwd()
        os.chdir(self.temp_dir)

        # Copy required certificate files to temp directory
        # The script expects these files to be in the current working directory
        cert_files = ['cacert.pem', 'client.crt', 'client.key']
        for cert_file in cert_files:
            src_path = os.path.join(self.input_data_dir, cert_file)
            dst_path = os.path.join(self.temp_dir, cert_file)
            if os.path.exists(src_path):
                shutil.copy2(src_path, dst_path)

        # Expected output directory and file
        self.output_dir = os.path.join(self.temp_dir, 'esp_secure_cert_data')
        self.generated_bin = os.path.join(self.output_dir, 'esp_secure_cert.bin')

        # Verify required files exist
        self.assertTrue(
            os.path.exists(self.script_path),
            f"Script not found: {self.script_path}")
        self.assertTrue(
            os.path.exists(self.csv_file),
            f"CSV file not found: {self.csv_file}")
        self.assertTrue(
            os.path.exists(self.expected_bin),
            f"Expected binary not found: {self.expected_bin}")

    def tearDown(self):
        """Clean up after each test"""
        os.chdir(self.original_cwd)
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def _calculate_sha256(self, file_path):
        """Calculate SHA256 hash of a file"""
        sha256_hash = hashlib.sha256()
        with open(file_path, 'rb') as f:
            for chunk in iter(lambda: f.read(4096), b""):
                sha256_hash.update(chunk)
        return sha256_hash.hexdigest()

    def _run_configure_script(self, args):
        """Run the configure_esp_secure_cert.py script with given arguments"""
        cmd = [sys.executable, self.script_path] + args
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        return result

    def test_generate_esp_secure_cert_bin_from_csv(self):
        """Test generating esp_secure_cert.bin from CSV configuration"""
        # Run the script with CSV input
        args = [
            '--esp_secure_cert_csv', self.csv_file,
            '--target_chip', 'esp32c3',
            '--skip_flash'  # Skip flashing to avoid hardware dependency
        ]

        result = self._run_configure_script(args)

        # Check if script executed successfully
        self.assertEqual(
            result.returncode, 0,
            f"Script failed with stderr: {result.stderr}\n"
            f"stdout: {result.stdout}")

        # Verify output directory was created
        self.assertTrue(
            os.path.exists(self.output_dir),
            f"Output directory not created: {self.output_dir}")

        # Verify binary file was generated
        self.assertTrue(
            os.path.exists(self.generated_bin),
            f"Generated binary not found: {self.generated_bin}")

        # Calculate SHA256 hashes
        generated_hash = self._calculate_sha256(self.generated_bin)
        expected_hash = self._calculate_sha256(self.expected_bin)

        # Compare hashes
        self.assertEqual(
            generated_hash, expected_hash,
            f"SHA256 mismatch:\n"
            f"Generated: {generated_hash}\n"
            f"Expected:  {expected_hash}\n"
            f"Generated file: {self.generated_bin}\n"
            f"Expected file: {self.expected_bin}")

    def test_script_help_option(self):
        """Test that the script shows help when --help is provided"""
        result = self._run_configure_script(['--help'])

        # Help should exit with code 0
        self.assertEqual(result.returncode, 0)

        # Help output should contain usage information
        self.assertIn('usage:', result.stdout.lower())
        self.assertIn('esp_secure_cert', result.stdout.lower())

    def test_script_missing_csv_file(self):
        """Test script behavior with missing CSV file"""
        non_existent_csv = os.path.join(self.temp_dir, 'non_existent.csv')

        args = [
            '--esp_secure_cert_csv', non_existent_csv,
            '--target_chip', 'esp32c3',
            '--skip_flash'
        ]

        result = self._run_configure_script(args)

        # Script should fail with non-zero exit code
        self.assertNotEqual(result.returncode, 0)

    def test_generated_files_structure(self):
        """Test that all expected files are generated in the output
        directory"""
        args = [
            '--esp_secure_cert_csv', self.csv_file,
            '--target_chip', 'esp32c3',
            '--skip_flash'
        ]

        result = self._run_configure_script(args)
        self.assertEqual(
            result.returncode, 0, f"Script failed: {result.stderr}")

        # Check that the binary file exists and has reasonable size
        self.assertTrue(os.path.exists(self.generated_bin))

        # Binary should not be empty
        bin_size = os.path.getsize(self.generated_bin)
        self.assertGreater(bin_size, 0, "Generated binary file is empty")

    def test_custom_data_from_file(self):
        """Test adding custom data from a file to esp_secure_cert.bin"""
        # Create a test custom data file
        custom_data_file = os.path.join(self.temp_dir, 'custom_data.txt')
        custom_data_content = b"Custom device configuration data"
        with open(custom_data_file, 'wb') as f:
            f.write(custom_data_content)

        # Create a CSV with custom data entry
        csv_content = (
            "tlv_type,tlv_subtype,data_value,data_type,priv_key_type,"
            "algorithm,key_size,efuse_id,efuse_key\n"
            "ESP_SECURE_CERT_CA_CERT_TLV,0,cacert.pem,file,,,,,\n"
            "ESP_SECURE_CERT_DEV_CERT_TLV,0,client.crt,file,,,,,\n"
            "ESP_SECURE_CERT_PRIV_KEY_TLV,0,client.key,file,plaintext,"
            "RSA,,,\n"
            "ESP_SECURE_CERT_USER_DATA_3_TLV,0,custom_data.txt,file,,,,,\n"
        )

        csv_file = os.path.join(self.temp_dir, 'test_custom_data.csv')
        with open(csv_file, 'w') as f:
            f.write(csv_content)

        # Run the script
        args = [
            '--esp_secure_cert_csv', csv_file,
            '--target_chip', 'esp32c3',
            '--skip_flash'
        ]
        result = self._run_configure_script(args)

        # Check if script executed successfully
        self.assertEqual(
            result.returncode, 0,
            f"Script failed: {result.stderr}")

        # Verify binary was generated
        generated_bin = os.path.join(
            self.temp_dir, 'esp_secure_cert_data', 'esp_secure_cert.bin')
        self.assertTrue(
            os.path.exists(generated_bin),
            "Binary file not generated")

        # Parse the binary to verify custom data was added
        parse_result = self._run_configure_script(
            ['--parse_bin', generated_bin])
        self.assertEqual(
            parse_result.returncode, 0,
            f"Parse failed: {parse_result.stderr}")

        # Check that custom data appears in parsed output
        self.assertIn(
            'ESP_SECURE_CERT_USER_DATA_3_TLV', parse_result.stdout,
            "Custom data TLV not found in parsed output")

        # Verify the parsed CSV contains the custom data entry
        parsed_csv = os.path.join(
            self.temp_dir, 'esp_secure_cert_parsed_data',
            'esp_secure_cert_parsed.csv')
        self.assertTrue(
            os.path.exists(parsed_csv),
            "Parsed CSV not generated")

        with open(parsed_csv, 'r') as f:
            parsed_content = f.read()

        self.assertIn(
            'ESP_SECURE_CERT_USER_DATA_3_TLV', parsed_content,
            "Custom data not found in parsed CSV")


class ConfigureEspSecureCertIntegrationTest(unittest.TestCase):
    """Integration tests for configure_esp_secure_cert.py with different
    configurations"""

    def setUp(self):
        """Set up test environment"""
        self.test_dir = os.path.dirname(os.path.abspath(__file__))
        self.tools_dir = os.path.dirname(self.test_dir)
        self.script_path = os.path.join(
            self.tools_dir, 'configure_esp_secure_cert.py')
        self.input_data_dir = os.path.join(self.test_dir, 'input_data')

        # Create temporary directory
        self.temp_dir = tempfile.mkdtemp()
        self.original_cwd = os.getcwd()
        os.chdir(self.temp_dir)

        # Copy required certificate files to temp directory
        # The script expects these files to be in the current working directory
        cert_files = ['cacert.pem', 'client.crt', 'client.key']
        for cert_file in cert_files:
            src_path = os.path.join(self.input_data_dir, cert_file)
            dst_path = os.path.join(self.temp_dir, cert_file)
            if os.path.exists(src_path):
                shutil.copy2(src_path, dst_path)

    def tearDown(self):
        """Clean up after test"""
        os.chdir(self.original_cwd)
        if os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)

    def test_different_target_chips(self):
        """Test script with different target chip configurations"""
        csv_file = os.path.join(
            self.test_dir, 'input_data',
            'esp_secure_cert_config_examples.csv')

        # Test with different target chips
        target_chips = ['esp32', 'esp32s3']

        for chip in target_chips:
            with self.subTest(chip=chip):
                # Clean up previous run
                output_dir = os.path.join(
                    self.temp_dir, 'esp_secure_cert_data')
                if os.path.exists(output_dir):
                    shutil.rmtree(output_dir)

                args = [
                    '--esp_secure_cert_csv', csv_file,
                    '--target_chip', chip,
                    '--skip_flash'
                ]

                result = subprocess.run(
                    [sys.executable, self.script_path] + args,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )

                self.assertEqual(
                    result.returncode, 0,
                    f"Script failed for {chip}: {result.stderr}")

                # Verify binary was generated
                generated_bin = os.path.join(
                    output_dir, 'esp_secure_cert.bin')
                self.assertTrue(
                    os.path.exists(generated_bin),
                    f"Binary not generated for {chip}")


if __name__ == '__main__':
    # Run tests
    unittest.main(verbosity=2)
