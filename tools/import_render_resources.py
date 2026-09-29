#!/usr/bin/env python3
"""Import pinned original shaders and upstream GLM headers for local renderer extraction."""

import argparse
import hashlib
import io
import json
import subprocess
import tarfile
from pathlib import Path

from capture_stage_reference import REVISION, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulator_repo", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    glm_root = "camera_faker/include/external/glm/"
    shader_root = "camera_faker/shaders/pool/"
    archive = subprocess.check_output(
        ["git", "-C", str(args.simulator_repo), "archive", REVISION, glm_root, shader_root]
    )
    outputs, sources = {}, {}
    with tarfile.open(fileobj=io.BytesIO(archive)) as bundle:
        for entry in bundle.getmembers():
            if not entry.isfile():
                continue
            if entry.name.startswith(glm_root):
                destination = "third_party/glm/glm/" + entry.name[len(glm_root) :]
            else:
                filename = entry.name[len(shader_root) :]
                if filename.split(".")[0] not in ("scene", "shadow", "water", "bloom", "post"):
                    continue
                destination = "libraries/rendering/shaders/" + filename
            data = bundle.extractfile(entry).read()
            sources[entry.name] = hashlib.sha256(data).hexdigest()
            if destination.endswith("/scene.frag"):
                shader = data.decode().replace(
                    "uniform int outdoor;", "uniform int outdoor;\nuniform int waterEnabled;"
                )
                shader = shader.replace(
                    "float worldZ=world.z-waterLevel,eyeZ=eye.z-waterLevel;",
                    "float worldZ=waterEnabled==1?world.z-waterLevel:1.,eyeZ=waterEnabled==1?eye.z-waterLevel:1.;",
                )
                data = shader.encode()
            outputs[destination] = data
    metadata = {
        "source_revision": REVISION,
        "scope": "Original scene/shadow/water/bloom/post shaders; scene.frag adds an explicit water-enabled guard and upstream GLM 1.0.0 headers. No original host or ROS code.",
        "sources_sha256": sources,
        "outputs_sha256": {
            name: hashlib.sha256(data).hexdigest() for name, data in outputs.items()
        },
        "importer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    outputs["docs/reference/render_resources.json"] = (
        json.dumps(metadata, indent=2) + "\n"
    ).encode()
    for name, data in outputs.items():
        path = ROOT / name
        if args.check:
            if path.read_bytes() != data:
                raise ValueError(f"stale render resource: {name}")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
    print(f"{'Verified' if args.check else 'Imported'} {len(outputs)} pinned render resources")


if __name__ == "__main__":
    main()
