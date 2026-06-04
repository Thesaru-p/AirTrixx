from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from runtime_acceleration import configure_opencv_acceleration, request_windows_high_performance_gpu


class RuntimeAccelerationTests(unittest.TestCase):
    def test_source_run_does_not_set_windows_gpu_preference(self) -> None:
        with patch("runtime_acceleration.platform.system", return_value="Windows"):
            with patch.object(sys, "frozen", False, create=True):
                self.assertEqual(request_windows_high_performance_gpu(), [])

    def test_opencv_opencl_is_enabled_when_available(self) -> None:
        class FakeOpenCL:
            enabled = False

            @staticmethod
            def haveOpenCL() -> bool:
                return True

            @classmethod
            def setUseOpenCL(cls, value: bool) -> None:
                cls.enabled = value

            @classmethod
            def useOpenCL(cls) -> bool:
                return cls.enabled

        class FakeCV2:
            ocl = FakeOpenCL

            @staticmethod
            def setUseOptimized(_value: bool) -> None:
                pass

        self.assertEqual(configure_opencv_acceleration(FakeCV2), ["OpenCV OpenCL acceleration is enabled."])


if __name__ == "__main__":
    unittest.main()
