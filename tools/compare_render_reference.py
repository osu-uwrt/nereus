#!/usr/bin/env python3
"""Compare complete fixed-frame buffers captured on the same GL backend."""

import argparse
import hashlib
import json
import math
from array import array
from pathlib import Path

from check import ROOT


def compare(reference: Path, actual: Path):
    manifest = json.loads((reference / "manifest.json").read_text())
    width, height = manifest["dimensions"]
    fields = {"rgba": 4, "opaque": 16, "depth": 4, "composite": 16, "composite-depth": 4}
    results = []
    for name, digest in manifest["outputs_sha256"].items():
        if Path(name).suffix in (".view", ".lights"):
            if hashlib.sha256((reference / name).read_bytes()).hexdigest() != digest:
                raise ValueError(f"modified reference input: {name}")
    for case in manifest.get("cases", list(range(6))):
        for field, stride in fields.items():
            name = f"{case}.{field}"
            expected = (reference / name).read_bytes()
            observed = (actual / name).read_bytes()
            digest = hashlib.sha256(expected).hexdigest()
            if manifest["outputs_sha256"][name] != digest:
                raise ValueError(f"modified reference: {name}")
            if len(expected) != width * height * stride or len(observed) != len(expected):
                raise ValueError(f"wrong buffer size: {name}")
            if field != "rgba":
                values = array("f")
                values.frombytes(observed)
                if not all(math.isfinite(v) for v in values):
                    raise ValueError(f"nonfinite captured values: {name}")
                if "depth" in field and not all(0 <= v <= 1 for v in values):
                    raise ValueError(f"invalid depth: {name}")
            elif len(set(observed)) < 32:
                raise ValueError(f"insufficient image content: {name}")
            if observed != expected:
                count = sum(a != b for a, b in zip(expected, observed))
                raise ValueError(f"{name}: {count} bytes differ from original")
            results.append({"file": name, "bytes": len(observed), "sha256": digest})
        if (actual / f"{case}.depth").read_bytes() != (
            actual / f"{case}.composite-depth"
        ).read_bytes():
            raise ValueError("water composition changed scene depth")
    if (actual / "0.rgba").read_bytes() == (actual / "4.rgba").read_bytes():
        raise ValueError("reflection-enabled/disabled fixtures did not change output")
    if 6 in manifest.get("cases", []):
        for field in ("depth", "composite-depth"):
            off = (actual / f"6.{field}").read_bytes()
            if any((actual / f"{case}.{field}").read_bytes() != off for case in (7, 8)):
                raise ValueError("indicator color changed geometry/depth")
        for field in ("rgba", "opaque", "composite"):
            captures = [(actual / f"{case}.{field}").read_bytes() for case in (6, 7, 8)]
            if len(set(captures)) != 3:
                raise ValueError("indicator off/red/individual colors did not change output")
    native = [
        "libraries/rendering/src/assets.cpp",
        "libraries/rendering/include/robotics/rendering/assets.hpp",
        "libraries/rendering/src/scene.cpp",
        "libraries/rendering/src/renderer.cpp",
        "libraries/rendering/include/robotics/rendering/scene.hpp",
        "libraries/rendering/include/robotics/rendering/renderer.hpp",
        "applications/render_capture/main.cpp",
        "applications/render_capture/benchmark.hpp",
    ]
    native += [
        str(p.relative_to(ROOT)) for p in sorted((ROOT / "libraries/rendering/shaders").iterdir())
    ]
    return {
        "scope": manifest["scope"],
        "backend": manifest["backend"],
        "dimensions": [width, height],
        "time_seconds": manifest["time_seconds"],
        "comparison": f"All {len(results)} buffers byte-identical, finite, and nonempty; water leaves depth unchanged.",
        "original_capture": manifest,
        "native_performance": json.loads((actual / "benchmark.json").read_text()),
        "native_sources_sha256": {
            name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in native
        },
        "outputs": results,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("actual", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    report = compare(args.reference, args.actual)
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(report["comparison"])


if __name__ == "__main__":
    main()
