#!/usr/bin/env python3
"""Import pinned Talos body/rotors and indicator geometry; offline only."""

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

    def read_path(path):
        data = subprocess.check_output(
            ["git", "-C", str(args.simulator_repo), "show", f"{REVISION}:{path}"]
        )
        sources[path] = hashlib.sha256(data).hexdigest()
        return data

    def read(name):
        return read_path(source_root + name)

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
        "scope": "Original body, eight rotors and indicator geometry; launcher, claw and magnet remain separate work.",
        "coordinate_frame": "cad",
        "position_units": "metres",
        "body_mesh": "Talos3_body.glb",
        "indicators_scene": "../scenes/talos_indicators.yaml",
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
    indicator_bytes = read_path("c_simulator/robots/talos/config/status_lights.yaml")
    outputs[destination + "provenance/status_lights.yaml"] = indicator_bytes
    indicators = yaml.safe_load(indicator_bytes)
    groups = []
    for light in indicators["lights"]:
        if light["pose"][3:] != [0, 0, 0]:
            raise ValueError("pinned indicator mount requires a rotation conversion")
        groups.append({
            "id": "indicator/" + light["id"],
            "source": "simulation",
            "frame": "cad",
            "instances": [{
                "box": light["size"],
                "pose": {"position": light["pose"][:3], "orientation_wxyz": [1, 0, 0, 0]},
                "material": "emissive",
                "radiance": light["radiance"],
                "casts_shadow": False,
                "color_channel": "indicator/" + light["id"],
            }],
        })
    outputs["content/visuals/scenes/talos_indicators.yaml"] = yaml.safe_dump(
        {"version": 1, "groups": groups}, sort_keys=False
    ).encode()
    composition_source = "content/visuals/scenes/talos_pool.yaml"
    composition_bytes = (ROOT / composition_source).read_bytes()
    composition = yaml.safe_load(composition_bytes)
    composition["groups"].extend(groups)
    outputs["content/visuals/scenes/talos_indicator_pool.yaml"] = yaml.safe_dump(
        composition, sort_keys=False
    ).encode()
    manifest = {
        "source_repository": "https://github.com/osu-uwrt/riptide_simulator",
        "source_revision": REVISION,
        "scope": inventory["scope"],
        "sources_sha256": sources,
        "composition_sources_sha256": {composition_source: hashlib.sha256(composition_bytes).hexdigest()},
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
