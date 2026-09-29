"""Optional installed camera extension: real offscreen capture, ownership and stereo."""

import gc
import os
import subprocess
import sys
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

try:
    from robotics_platform import _camera as camera
except ImportError:
    if os.environ.get("RP_REQUIRE_CAMERA"):
        raise
    camera = None


@unittest.skipIf(camera is None, "optional camera extension is not installed")
class CameraCaptureTests(unittest.TestCase):
    def setUp(self):
        shaders = Path(camera.__file__).parent / "shaders"
        self.host = camera.OffscreenRenderer(shaders)
        self.scene = camera.Scene()
        box = camera.Instance()
        box.mesh = camera.box_mesh()
        transform = np.eye(4)
        transform[2, 3] = 3
        box.transform = transform
        self.scene.instances = [box]
        self.intrinsics = camera.Intrinsics()
        self.intrinsics.width, self.intrinsics.height = 64, 48
        self.intrinsics.fx = self.intrinsics.fy = 50
        self.intrinsics.cx, self.intrinsics.cy = 31.5, 23.5
        self.appearance = camera.Appearance()
        self.appearance.shadows = self.appearance.reflections = False
        self.appearance.caustics = 0
        self.noise = camera.DepthNoise()
        self.noise.enabled = False

    def capture(self, x=0, color=True, depth=True):
        view = camera.View()
        view.view = camera.optical_view([x, 0, 0], [1, 0, 0, 0])
        view.projection = self.intrinsics.projection()
        view.eye = [x, 0, 0]
        return self.host.capture(self.scene, view, self.appearance, 0, 64, 48, color, depth)

    def test_owned_readonly_pixels_survive_processing_and_producers(self):
        raw = self.capture()
        processor = camera.Processor(7)
        frame = processor.process(self.intrinsics, self.noise, raw, jpeg=True)
        rgb, depth = frame.rgb, frame.depth
        self.assertEqual(rgb.shape, (48, 64, 3))
        self.assertEqual(depth.shape, (48, 64))
        self.assertFalse(rgb.flags.writeable)
        self.assertFalse(depth.flags.writeable)
        self.assertTrue(frame.jpeg.startswith(b"\xff\xd8"))
        self.assertAlmostEqual(float(depth[24, 32]), 2.5, places=4)
        saved = rgb.copy()
        del frame, raw, processor, self.host
        gc.collect()
        np.testing.assert_array_equal(rgb, saved)
        self.assertAlmostEqual(float(depth[24, 32]), 2.5, places=4)

    def test_actual_right_eye_capture_has_calibrated_disparity(self):
        processor = camera.Processor(7)
        left = processor.process(self.intrinsics, self.noise, self.capture()).depth
        right = processor.process(self.intrinsics, self.noise, self.capture(x=0.1)).depth
        left_columns = np.nonzero(np.isfinite(left[24]))[0]
        right_columns = np.nonzero(np.isfinite(right[24]))[0]
        disparity = left_columns.mean() - right_columns.mean()
        self.assertAlmostEqual(disparity, self.intrinsics.fx * 0.1 / 2.5, places=5)

    def test_scene_instance_copies_survive_replacement_and_scene_destruction(self):
        instances = self.scene.instances
        instances[0].visible = False
        self.assertTrue(self.scene.instances[0].visible)
        self.scene.instances = []
        del self.scene
        gc.collect()
        self.assertFalse(instances[0].visible)
        self.assertIsNotNone(instances[0].mesh)
        self.scene = camera.Scene()
        instances[0].visible = True
        self.scene.instances = instances
        frame = camera.Processor(7).process(self.intrinsics, self.noise, self.capture())
        self.assertAlmostEqual(float(frame.depth[24, 32]), 2.5, places=4)

    def test_capture_moves_to_workers_and_processors_replay_after_reset(self):
        expected = self.capture().rgb.copy()
        with ThreadPoolExecutor(max_workers=2) as workers:
            futures = [workers.submit(self.capture) for _ in range(2)]
            for result in futures:
                np.testing.assert_array_equal(result.result().rgb, expected)
        self.noise.enabled = True
        raw = self.capture()
        processor = camera.Processor(23)
        first = processor.process(self.intrinsics, self.noise, raw).depth.copy()
        processor.reset(23)
        np.testing.assert_array_equal(
            processor.process(self.intrinsics, self.noise, raw).depth, first
        )

    def test_omitted_buffers_and_wrong_calibration_are_explicit(self):
        raw = self.capture(color=False)
        self.assertEqual(raw.rgb.size, 0)
        processor = camera.Processor(1)
        with self.assertRaises(ValueError):
            processor.process(self.intrinsics, self.noise, raw, jpeg=True)
        self.intrinsics.width, self.intrinsics.height = 48, 64
        with self.assertRaisesRegex(ValueError, "dimensions"):
            processor.process(self.intrinsics, self.noise, raw)

    def test_extension_loads_without_native_simulation_or_ros(self):
        subprocess.run([sys.executable, "-c", "\n".join([
            "import sys",
            "from robotics_platform import _camera",
            "assert 'robotics_platform._native' not in sys.modules",
            "assert not any(k == 'rclpy' or k.startswith('rclpy.') for k in sys.modules)",
        ])], check=True)
        self.assertTrue(self.host.device)


if __name__ == "__main__":
    unittest.main()
