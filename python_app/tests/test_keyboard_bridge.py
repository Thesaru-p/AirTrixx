from __future__ import annotations

import sys
import tempfile
import time
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from keyboard_bridge import KeyboardBridge


class KeyboardBridgeAntennaTests(unittest.TestCase):
    def _bridge(self, tmpdir: str) -> KeyboardBridge:
        root = Path(tmpdir)
        return KeyboardBridge(
            dataset_path=root / "raw_samples.csv",
            model_path=root / "word_knn_model.npz",
            words_path=root / "words.txt",
        )

    def test_ingests_antenna_keyboard_tof_without_usb(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            bridge = self._bridge(tmpdir)
            now_s = time.monotonic()

            self.assertTrue(
                bridge.ingest_antenna_device(
                    {
                        "status": "ok",
                        "sequence": 42,
                        "t_ms": 1234,
                        "tof": {
                            "sensor_1_mm": 81,
                            "sensor_2_mm": 139,
                            "sensor_3_mm": 128,
                            "sensor_4_mm": None,
                        },
                        "valid": {
                            "sensor_1": True,
                            "sensor_2": True,
                            "sensor_3": True,
                            "sensor_4": False,
                        },
                    },
                    now_s=now_s,
                )
            )
            snapshot = bridge.snapshot()

        self.assertFalse(bridge.is_connected)
        self.assertTrue(snapshot["app_connected"])
        self.assertTrue(snapshot["antenna_active"])
        self.assertEqual(snapshot["source"], "antenna_espnow")
        self.assertEqual(snapshot["status"], "Antenna ESP-NOW")
        self.assertEqual(snapshot["tof"]["sensor_1_mm"], 81)
        self.assertIsNone(snapshot["tof"]["sensor_4_mm"])
        self.assertFalse(snapshot["valid"]["sensor_4"])

    def test_training_can_arm_from_antenna_source(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            bridge = self._bridge(tmpdir)
            bridge.ingest_antenna_device(
                {
                    "status": "ok",
                    "sequence": 1,
                    "tof": {
                        "sensor_1_mm": 260,
                        "sensor_2_mm": 260,
                        "sensor_3_mm": 260,
                        "sensor_4_mm": 260,
                    },
                    "valid": {
                        "sensor_1": True,
                        "sensor_2": True,
                        "sensor_3": True,
                        "sensor_4": True,
                    },
                }
            )

            self.assertTrue(bridge.start_training(["hello"], repetitions=1, include_command_words=False))
            self.assertTrue(bridge.arm_next_training_sample())
            self.assertEqual(bridge.training_status, "Swipe 'hello' (1/1)")


if __name__ == "__main__":
    unittest.main()
