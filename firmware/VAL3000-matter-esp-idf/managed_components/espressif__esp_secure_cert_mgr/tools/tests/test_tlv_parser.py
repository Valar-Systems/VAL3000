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


class TlvParserTest(unittest.TestCase):
    """Test class for TLV parser functionality in configure_esp_secure_cert.py"""

    def setUp(self):
        """Set up test environment before each test"""
        # Get the directory paths
        self.test_dir = os.path.dirname(os.path.abspath(__file__))
        self.tools_dir = os.path.dirname(self.test_dir)
        self.script_path = os.path.join(self.tools_dir, 'configure_esp_secure_cert.py')
        self.input_data_dir = os.path.join(self.test_dir, 'input_data')

        # Input files
        self.bin_file = os.path.join(self.input_data_dir, 'esp_secure_cert.bin')
        self.expected_parsed_dir = os.path.join(self.input_data_dir, 'esp_secure_cert_parsed_data_expected')

        # Create a temporary working directory
        self.temp_dir = tempfile.mkdtemp()
        self.original_cwd = os.getcwd()
        os.chdir(self.temp_dir)

        # Copy the binary file to temp directory for parsing
        self.temp_bin_file = os.path.join(self.temp_dir, 'esp_secure_cert.bin')
        shutil.copy2(self.bin_file, self.temp_bin_file)

        # Expected output directory and files
        self.output_dir = os.path.join(self.temp_dir, 'esp_secure_cert_parsed_data')

        # Verify required files exist
        self.assertTrue(os.path.exists(self.script_path),
                       f"Script not found: {self.script_path}")
        self.assertTrue(os.path.exists(self.bin_file),
                       f"Binary file not found: {self.bin_file}")
        self.assertTrue(os.path.exists(self.expected_parsed_dir),
                       f"Expected parsed directory not found: {self.expected_parsed_dir}")

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

    def test_parse_esp_secure_cert_bin(self):
        """Test parsing esp_secure_cert.bin and comparing generated files with expected"""
        # Run the script with --parse_bin option
        args = ['--parse_bin', 'esp_secure_cert.bin']
        result = self._run_configure_script(args)

        # Check if script executed successfully
        self.assertEqual(result.returncode, 0,
                        f"Parse script failed with stderr: {result.stderr}\nstdout: {result.stdout}")

        # Verify output directory was created
        self.assertTrue(os.path.exists(self.output_dir),
                       f"Parsed output directory not created: {self.output_dir}")

        # Verify contents subdirectory was created
        contents_dir = os.path.join(self.output_dir, 'contents')
        self.assertTrue(os.path.exists(contents_dir),
                       f"Contents subdirectory not created: {contents_dir}")

        # Get content files for debugging (after output directory is confirmed to exist)
        contents_files = []
        if os.path.exists(self.output_dir) and os.path.exists(contents_dir):
            contents_files = [f for f in os.listdir(contents_dir) if os.path.isfile(os.path.join(contents_dir, f))]

        # Get list of expected files (including directory structure)
        expected_files = []
        if os.path.exists(self.expected_parsed_dir):
            for root, dirs, files in os.walk(self.expected_parsed_dir):
                for file in files:
                    rel_path = os.path.relpath(os.path.join(root, file), self.expected_parsed_dir)
                    expected_files.append(rel_path)

        # Ensure we have expected files to compare against
        self.assertGreater(len(expected_files), 0,
                          f"No expected files found in {self.expected_parsed_dir}")

        # Verify that the main expected files are present
        expected_main_files = ['esp_secure_cert_parsed.csv', 'tlv_entries_raw.txt']
        for main_file in expected_main_files:
            self.assertIn(main_file, expected_files,
                         f"Required file {main_file} not found in expected files")

        # Get list of generated files (including directory structure)
        generated_files = []
        if os.path.exists(self.output_dir):
            for root, dirs, files in os.walk(self.output_dir):
                for file in files:
                    rel_path = os.path.relpath(os.path.join(root, file), self.output_dir)
                    generated_files.append(rel_path)

        # Calculate expected contents files for debugging
        expected_contents_files = [f for f in expected_files if f.startswith('contents/')]

        # Debug: Print file lists for troubleshooting
        print(f"\nExpected files ({len(expected_files)}): {sorted(expected_files)}")
        print(f"Generated files ({len(generated_files)}): {sorted(generated_files)}")
        print(f"Contents directory files: {sorted(contents_files) if contents_files else 'None'}")
        print(f"Contents directory exists: {os.path.exists(contents_dir)}")
        print(f"Output directory: {self.output_dir}")
        print(f"Expected directory: {self.expected_parsed_dir}")
        print(f"Expected contents files: {expected_contents_files}")
        if contents_files:
            print(f"Found contents files: {['contents/' + f for f in contents_files]}")
        else:
            print("Found contents files: None")

        # Verify all expected files were generated and match
        for expected_file in expected_files:
            generated_file_path = os.path.join(self.output_dir, expected_file)
            expected_file_path = os.path.join(self.expected_parsed_dir, expected_file)

            # Check if file exists
            self.assertTrue(os.path.exists(generated_file_path),
                           f"Expected parsed file not generated: {expected_file}\n"
                           f"Expected path: {generated_file_path}\n"
                           f"Available files: {sorted(generated_files)}")

            # For text files (CSV and TXT), compare content with normalization
            if expected_file.endswith('.csv') or expected_file.endswith('.txt'):
                with open(expected_file_path, 'r', encoding='utf-8') as f:
                    expected_content = f.read().strip().replace('\r\n', '\n')
                with open(generated_file_path, 'r', encoding='utf-8') as f:
                    generated_content = f.read().strip().replace('\r\n', '\n')

                self.assertEqual(generated_content, expected_content,
                                f"Content mismatch for {expected_file}:\n"
                                f"Expected file: {expected_file_path}\n"
                                f"Generated file: {generated_file_path}")
            else:
                # For binary files (PEM, DER, BIN), compare SHA256 hashes
                generated_hash = self._calculate_sha256(generated_file_path)
                expected_hash = self._calculate_sha256(expected_file_path)

                self.assertEqual(generated_hash, expected_hash,
                                f"SHA256 mismatch for {expected_file}:\n"
                                f"Generated: {generated_hash}\n"
                                f"Expected:  {expected_hash}\n"
                                f"Generated file: {generated_file_path}\n"
                                f"Expected file: {expected_file_path}")

        # Verify contents directory structure matches expectations
        if expected_contents_files:
            self.assertTrue(os.path.exists(contents_dir),
                           f"Contents directory should exist but doesn't: {contents_dir}")
            self.assertGreater(len(contents_files), 0,
                              f"Expected contents files but contents directory is empty: {contents_dir}")
            print(f"Expected {len(expected_contents_files)} contents files, found {len(contents_files)}")

        # Verify that no unexpected files were generated
        unexpected_files = set(generated_files) - set(expected_files)
        if len(unexpected_files) > 0:
            print(f"\nWARNING: Unexpected files generated: {sorted(unexpected_files)}")
            print(f"Expected files: {sorted(expected_files)}")
            print(f"Generated files: {sorted(generated_files)}")
        self.assertEqual(len(unexpected_files), 0,
                        f"Unexpected files generated: {sorted(unexpected_files)}")

    def test_parse_bin_help_option(self):
        """Test that the script shows help when --help is provided"""
        result = self._run_configure_script(['--help'])

        # Help should exit with code 0
        self.assertEqual(result.returncode, 0)

        # Help output should contain usage information
        self.assertIn('usage:', result.stdout.lower())
        self.assertIn('parse_bin', result.stdout.lower())

    def test_parse_bin_missing_file(self):
        """Test script behavior with missing binary file"""
        non_existent_bin = os.path.join(self.temp_dir, 'non_existent.bin')

        args = ['--parse_bin', non_existent_bin]
        result = self._run_configure_script(args)

        # Script should fail with non-zero exit code
        self.assertNotEqual(result.returncode, 0)

        # Should contain error message about missing file
        self.assertIn('does not exist', result.stdout)

    def test_parse_bin_invalid_file_extension(self):
        """Test script behavior with invalid file extension"""
        # Create a dummy file with wrong extension
        invalid_file = os.path.join(self.temp_dir, 'test.txt')
        with open(invalid_file, 'w') as f:
            f.write('dummy content')

        args = ['--parse_bin', invalid_file]
        result = self._run_configure_script(args)

        # Script should fail with non-zero exit code
        self.assertNotEqual(result.returncode, 0)

        # Should contain error message about invalid extension
        self.assertIn('not a .bin file', result.stdout)


if __name__ == '__main__':
    # Run tests
    unittest.main(verbosity=2)
