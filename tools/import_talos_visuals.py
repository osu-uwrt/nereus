#!/usr/bin/env python3
"""Copy pinned original Talos body/rotors and preserve visual metadata; offline only."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import yaml
from capture_stage_reference import REVISION, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("simulator_repo", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    source_root = "camera_faker/models/talos3/"
    destination = "content/visuals/talos/"
    sources = {}
    outputs = {}

    def read(name):
        data = subprocess.check_output(
            ["git", "-C", str(args.simulator_repo), "show", f"{REVISION}:{source_root}{name}"]
        )
        sources[source_root + name] = hashlib.sha256(data).hexdigest()
        return data

    settings = yaml.safe_load(read("thrusters.yaml"))
    outputs[destination + "Talos3_body.glb"] = read("Talos3_body.glb")
    rotors = []
    for rotor in settings["rotors"]:
        name = rotor["mesh"]
        if Path(name).parent != Path("rotors") or Path(name).suffix != ".glb":
            raise ValueError("unexpected pinned rotor path")
        outputs[destination + name] = read(name)
        rotors.append(
            {key: rotor[key] for key in ("id", "input_index", "pivot", "axis", "direction", "mesh")}
        )
    for name in ("Talos3_body.json", "material_repairs.json", "README.md", "thrusters.yaml"):
        outputs[destination + "provenance/" + name] = read(name)
    inventory = {
        "schema_version": 1,
        "kind": "visual_resource_inventory",
        "scope": "Original body and eight rotors only; launcher, claw, magnet and LEDs still separate work.",
        "coordinate_frame": "cad",
        "position_units": "metres",
        "body_mesh": "Talos3_body.glb",
        "rotors": rotors,
        "rotor_animation": {
            key: settings[key]
            for key in ("timeout", "force_deadband", "speed_scale", "force_to_rpm")
        },
    }
    outputs[destination + "inventory.json"] = (json.dumps(inventory, indent=2) + "\n").encode()
    ordered = sorted(rotors, key=lambda rotor: rotor["input_index"])
    if [rotor["input_index"] for rotor in ordered] != list(range(len(ordered))):
        raise ValueError("pinned rotor channels must have contiguous input indices")
    rig = {
        "version": 1,
        "inputs": [rotor["id"] for rotor in ordered],
        **inventory["rotor_animation"],
        "rotors": [
            {
                "input": ordered[rotor["input_index"]]["id"],
                "parent_frame": "cad",
                "child_frame": "rotor_mesh/" + rotor["id"],
                **{key: rotor[key] for key in ("pivot", "axis", "direction")},
            }
            for rotor in rotors
        ],
    }
    outputs["content/visuals/scenes/talos_rotors.yaml"] = yaml.safe_dump(
        rig, sort_keys=False
    ).encode()
    manifest = {
        "source_repository": "https://github.com/osu-uwrt/riptide_simulator",
        "source_revision": REVISION,
        "scope": inventory["scope"],
        "sources_sha256": sources,
        "importer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "outputs_sha256": {
            name: hashlib.sha256(data).hexdigest() for name, data in outputs.items()
        },
        "license_status": "Unresolved original asset metadata; local use authorized. See docs/PROVENANCE.md before public redistribution.",
    }
    outputs[destination + "provenance/manifest.json"] = (
        json.dumps(manifest, indent=2) + "\n"
    ).encode()
    for name, data in outputs.items():
        path = ROOT / name
        if args.check:
            if path.read_bytes() != data:
                raise ValueError(f"stale imported asset: {name}")
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
    print(f"{'Verified' if args.check else 'Imported'} {len(outputs)} pinned visual files")


if __name__ == "__main__":
    main()
