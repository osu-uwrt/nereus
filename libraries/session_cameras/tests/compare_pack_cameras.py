#!/usr/bin/env python3
"""Pixel comparison of rp_session_cameras against python/src/robotics_platform/pack_cameras.py.

Both render the same resolved scenario at fixed root poses with the scenario seed (noise on, always
mode). Usage (needs the Python `_camera` extension built from this tree, RP_BUILD_CAMERA_PYTHON):
  compare_pack_cameras.py --python-package DIR_WITH_robotics_platform_incl__camera.so \
      --capture-tool build/.../session_cameras_capture --scenario content/packs/scenarios/talos_uwrt
"""
import argparse
import json
import math
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

def yaw(deg):
    return [math.cos(math.radians(deg) / 2), 0, 0, math.sin(math.radians(deg) / 2)]


def pitch(deg):
    return [math.cos(math.radians(deg) / 2), 0, math.sin(math.radians(deg) / 2), 0]


POSES = {  # name -> (sensor, position, orientation_wxyz)
    "gate_ffc": ("ffc", [3.0, 0.0, -1.2], [1, 0, 0, 0]),
    "yawed_ffc": ("ffc", [6.0, -2.0, -1.0], yaw(30)),
    "down_dfc": ("dfc", [10.0, 3.0, -0.8], [1, 0, 0, 0]),
    "pitched_ffc": ("ffc", [4.0, 1.0, -0.9], pitch(30)),
}
TIME_S = 1.25


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--python-package", required=True, type=Path)
    ap.add_argument("--capture-tool", required=True, type=Path)
    ap.add_argument("--scenario", required=True, type=Path)
    ap.add_argument("--shaders", type=Path, default=Path(__file__).resolve().parents[3] / "libraries/rendering/shaders")
    ap.add_argument("--resolved-json", type=Path, required=True, help="pack tool output for the C++ side")
    args = ap.parse_args()
    sys.path.insert(0, str(args.python_package))
    from robotics_platform import pack_cameras as pc
    from robotics_platform.packs import resolve_scenario

    resolved = resolve_scenario(args.scenario)
    t0 = time.perf_counter()
    cams = pc.PackCameras(resolved, shader_directory=args.shaders)
    print(f"python construct {time.perf_counter() - t0:.2f} s, device {cams.describe()['device']}")
    worst = 0
    with tempfile.TemporaryDirectory() as tmp:
        for name, (sensor, position, quat) in POSES.items():
            cams.reset(resolved.scenario["seed"])
            start = time.perf_counter()
            capture = cams.capture(sensor, position, quat, TIME_S, jpeg_quality=None)
            py_ms = (time.perf_counter() - start) * 1e3
            prefix = Path(tmp) / name
            subprocess.run([str(args.capture_tool), str(args.resolved_json), sensor, *map(repr, position),
                            *map(repr, quat), str(TIME_S), str(prefix)], check=True)
            info = json.loads(prefix.with_suffix(".json").read_text())
            h, w = info["height"], info["width"]
            rgb = np.fromfile(prefix.with_suffix(".rgb"), np.uint8).reshape(h, w, 3).astype(int)
            depth = np.fromfile(prefix.with_suffix(".depth"), np.float32).reshape(h, w)
            ref_rgb = np.asarray(capture.left.rgb).astype(int)
            ref_depth = np.asarray(capture.left.depth)
            diff = np.abs(rgb - ref_rgb)
            both = np.isfinite(depth) & np.isfinite(ref_depth)
            nan_mismatch = int((np.isfinite(depth) != np.isfinite(ref_depth)).sum())
            ddiff = np.abs(depth[both] - ref_depth[both])
            print(f"{name}: rgb max|d|={diff.max()} mean|d|={diff.mean():.5f} identical={(diff == 0).mean():.6f}; "
                  f"depth max|d|={ddiff.max() if ddiff.size else 0:.3e} m, nan-mismatch={nan_mismatch}/{h * w}; "
                  f"python capture {py_ms:.0f} ms, cpp render {info['render_ms']:.0f} + process {info['process_ms']:.0f} ms")
            worst = max(worst, diff.max())
            if diff.max() != 0 or nan_mismatch or (ddiff.size and ddiff.max() > 0):
                print("  MISMATCH", file=sys.stderr)
                return 1
    print("identical")
    return 0


if __name__ == "__main__":
    sys.exit(main())
