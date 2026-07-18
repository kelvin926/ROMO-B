#!/usr/bin/env python3
"""Regression tests for safe hardware configuration generation."""

from __future__ import annotations

import pathlib
import tempfile
import unittest

import yaml

from generate_hardware_config import hardware_template, write_config


class GenerateHardwareConfigTest(unittest.TestCase):
    def test_template_contains_safe_pending_defaults(self) -> None:
        config = hardware_template("FTDI_TEST", "enxTEST")
        self.assertEqual(config["serial"]["adapter_serial"], "FTDI_TEST")
        self.assertEqual(config["serial"]["baud"], 115200)
        self.assertEqual(config["lidar"]["interface"], "enxTEST")
        self.assertEqual(config["lidar"]["lidar_ip"], "pending")
        self.assertFalse(config["lidar"]["calibrated"])

    def test_existing_calibration_is_not_replaced_without_force(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "hardware.yaml"
            sentinel = "calibration: preserve-me\n"
            output.write_text(sentinel, encoding="utf-8")
            with self.assertRaises(FileExistsError):
                write_config(output, "new-adapter", "new-interface")
            self.assertEqual(output.read_text(encoding="utf-8"), sentinel)

    def test_force_replaces_file_atomically_with_valid_yaml(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = pathlib.Path(directory) / "hardware.yaml"
            output.write_text("old: true\n", encoding="utf-8")
            write_config(output, "FTDI_TEST", "enxTEST", force=True)
            config = yaml.safe_load(output.read_text(encoding="utf-8"))
            self.assertEqual(config["schema_version"], 1)
            self.assertEqual(config["serial"]["adapter_serial"], "FTDI_TEST")
            self.assertEqual(config["lidar"]["interface"], "enxTEST")
            self.assertEqual(list(output.parent.glob(".*.tmp")), [])


if __name__ == "__main__":
    unittest.main()
