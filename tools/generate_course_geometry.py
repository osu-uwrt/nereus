#!/usr/bin/env python3
"""Generate the procedural course meshes of the task pack as GLB assets (numpy only).

Reads dimensions from the task pack itself (crate regions in bins.yaml, octagon region and sign
frames in surface.yaml) and writes crate lattice/liner and octagon ring GLBs next to the pack
assets. Coordinates are meters, +Z up (like the other pack GLBs). Run from the repository root:

    python3 tools/generate_course_geometry.py            # write files
    python3 tools/generate_course_geometry.py --hashes   # also print sha256 lines for tasks.yaml

The pack asset entries (sha256) in tasks.yaml must be refreshed after regenerating.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from pathlib import Path

import numpy as np
import yaml

# Relative to the repository root, which the script must be run from.
PACK = Path("content/packs/tasks/robosub_2026")

# Material base colors (glTF baseColorFactor, RGBA).
NAVY = (0.025, 0.055, 0.13, 1.0)
WHITE_LINER = (0.92, 0.94, 0.91, 1.0)
PVC = (0.94, 0.95, 0.91, 1.0)


class Mesh:
    """A single-material triangle mesh built up as position, normal and index lists."""

    def __init__(self, color):
        self.color = color
        self.positions: list = []
        self.normals: list = []
        self.indices: list = []

    def quad(self, corners, normal):
        """Append a quad (corners in winding order) as two triangles sharing `normal`."""
        base = len(self.positions)
        self.positions += [np.asarray(c, float) for c in corners]
        self.normals += [np.asarray(normal, float)] * 4
        self.indices += [base, base + 1, base + 2, base, base + 2, base + 3]

    def box(self, center, size):
        """Axis-aligned box: one flat-shaded quad per face, outward normals."""
        center, size = np.asarray(center, float), np.asarray(size, float)
        for axis in range(3):
            for sign in (-1, 1):
                n = np.zeros(3)
                n[axis] = sign
                u = np.zeros(3)
                u[(axis + 1) % 3] = 1
                w = np.zeros(3)
                w[(axis + 2) % 3] = sign
                corners = [
                    center + (n * 0.5 + (a - 0.5) * u + (b - 0.5) * w) * size
                    for a, b in ((0, 0), (1, 0), (1, 1), (0, 1))
                ]
                self.quad(corners, n)

    def tube(self, a, b, radius, segments=16):
        """Open cylinder from a to b with smooth radial normals (no end caps)."""
        a, b = np.asarray(a, float), np.asarray(b, float)

        # u, v: unit vectors perpendicular to the axis, from a helper axis not parallel to it.
        axis = (b - a) / np.linalg.norm(b - a)
        u = np.cross(axis, (0, 0, 1) if abs(axis[2]) < 0.9 else (0, 1, 0))
        u /= np.linalg.norm(u)
        v = np.cross(axis, u)

        # One vertex pair (a end, b end) per angle step; consecutive pairs form a quad.
        base = len(self.positions)
        for i in range(segments + 1):
            angle = 2 * math.pi * i / segments
            n = u * math.cos(angle) + v * math.sin(angle)
            self.positions += [a + radius * n, b + radius * n]
            self.normals += [n, n]
            if i < segments:
                for j in (0, 1, 3, 0, 3, 2):
                    self.indices.append(base + 2 * i + j)


def write_glb(path: Path, meshes: dict[str, Mesh]) -> None:
    """Write a glTF 2.0 binary with one node and one primitive (and material) per mesh."""
    blob = bytearray()
    views, accessors, primitives, materials = [], [], [], []

    # Append data to the BIN chunk (4-byte aligned) as a new buffer view; returns its index.
    def add(data: bytes, target: int) -> int:
        while len(blob) % 4:
            blob.append(0)
        views.append(
            {"buffer": 0, "byteOffset": len(blob), "byteLength": len(data), "target": target}
        )
        blob.extend(data)
        return len(views) - 1

    for name, mesh in meshes.items():
        pos = np.asarray(mesh.positions, np.float32)
        nor = np.asarray(mesh.normals, np.float32)
        idx = np.asarray(mesh.indices, np.uint32)
        first = len(accessors)

        # glTF enums: 34962/34963 = vertex/index buffer targets, 5126 = float, 5125 = uint32.
        accessors += [
            {
                "bufferView": add(pos.tobytes(), 34962),
                "componentType": 5126,
                "count": len(pos),
                "type": "VEC3",
                "min": pos.min(0).tolist(),
                "max": pos.max(0).tolist(),
            },
            {
                "bufferView": add(nor.tobytes(), 34962),
                "componentType": 5126,
                "count": len(nor),
                "type": "VEC3",
            },
            {
                "bufferView": add(idx.tobytes(), 34963),
                "componentType": 5125,
                "count": len(idx),
                "type": "SCALAR",
            },
        ]
        materials.append(
            {
                "name": name,
                "doubleSided": True,
                "pbrMetallicRoughness": {
                    "baseColorFactor": list(mesh.color),
                    "metallicFactor": 0.0,
                    "roughnessFactor": 0.7,
                },
            }
        )
        # mode 4 = triangle list.
        primitives.append(
            {
                "attributes": {"POSITION": first, "NORMAL": first + 1},
                "indices": first + 2,
                "material": len(materials) - 1,
                "mode": 4,
            }
        )

    document = {
        "asset": {"version": "2.0", "generator": "tools/generate_course_geometry.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": path.stem}],
        "meshes": [{"primitives": primitives}],
        "materials": materials,
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": len(blob)}],
    }

    # GLB container: 12-byte header, then the JSON chunk (space-padded) and BIN chunk (zero-padded).
    text = json.dumps(document, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    while len(blob) % 4:
        blob.append(0)
    total = 12 + 8 + len(text) + 8 + len(blob)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(
        struct.pack("<4sII", b"glTF", 2, total)
        + struct.pack("<I4s", len(text), b"JSON")
        + text
        + struct.pack("<I4s", len(blob), b"BIN\0")
        + bytes(blob)
    )


def crate(params: dict) -> tuple[Mesh, Mesh]:
    """CleverMade crate in its frame (z = 0 interior floor): navy lattice/base and white liner."""
    outer, inner = params["outer_width_m"], params["inner_width_m"]
    base = params["base_thickness_m"]
    height = params["outer_height_m"] - base
    liner = params["liner_thickness_m"]
    liner_h = height * params["liner_height_fraction"]
    lattice, lining = Mesh(NAVY), Mesh(WHITE_LINER)

    # Base slab below the interior floor, then the four walls (two axes x two sides).
    lattice.box((0, 0, -base / 2 - 0.0005), (outer, outer, base))
    for axis in range(2):
        for sign in (-1, 1):

            def wall(mesh, across, along, z, thick, length, h):
                """Box at `across` on this axis: `thick` deep, `length` along the wall, `h` tall."""
                p, size = [0.0, 0.0, z], [thick, thick, h]
                p[axis], p[1 - axis], size[1 - axis] = across, along, length
                mesh.box(p, size)

            edge = sign * (outer / 2 - 0.007)
            for bar in range(9):  # navy lattice: vertical bars, horizontal rails, corner posts
                wall(
                    lattice,
                    edge,
                    -outer / 2 + 0.007 + bar * (outer - 0.014) / 8,
                    height / 2,
                    0.008,
                    0.009,
                    height,
                )
            for bar in range(6):
                wall(lattice, edge, 0, 0.01 + bar * (height - 0.02) / 5, 0.012, outer, 0.012)
            wall(lattice, edge, 0, height - 0.008, 0.016, outer, 0.016)  # top rail

            # The white liner panel just inside the inner width.
            wall(lining, sign * (inner / 2 - liner / 2), 0, liner_h / 2, liner, inner, liner_h)
    return lattice, lining


def octagon_ring(region: dict, frames: dict, ring: str) -> Mesh:
    """PVC ring and sign hangers in the ring frame (the octagon at the water surface)."""
    apothem, radius = region["apothem_m"], region["pipe_radius_m"]
    outer = apothem / math.cos(math.pi / 8)
    mesh = Mesh(PVC)

    # Eight pipe segments between the octagon's vertices (outer = circumradius).
    for i in range(8):
        a, b = (i + 0.5) * math.pi / 4, (i + 1.5) * math.pi / 4
        mesh.tube(
            (outer * math.cos(a), outer * math.sin(a), 0),
            (outer * math.cos(b), outer * math.sin(b), 0),
            radius,
        )
    ring_z = frames[ring]["position_m"][2]
    for name in region["facing"]["targets"]:
        x, y, z = frames[name]["position_m"]
        # 12 inch sign, origin at its center: the hanger runs from the ring to the sign's top edge.
        mesh.tube((x, y, 0), (x, y, z + 0.1524 - ring_z), 0.002)
    return mesh


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--hashes", action="store_true")
    args = parser.parse_args()

    # Dimensions come from the pack's own region and frame definitions.
    bins = yaml.safe_load((PACK / "bins.yaml").read_text())
    surface = yaml.safe_load((PACK / "surface.yaml").read_text())
    crate_region = next(r for r in bins["regions"] if r["type"] == "open_crate")["parameters"]
    lattice, lining = crate(crate_region)
    frames = {f["id"]: f for f in surface["frames"]}
    octagon = next(r for r in surface["regions"] if r["id"] == "octagon")["parameters"]
    outputs = {
        PACK / "assets/course/crate_lattice.glb": {"lattice": lattice},
        PACK / "assets/course/crate_liner.glb": {"liner": lining},
        PACK / "assets/course/octagon_ring.glb": {
            "pvc": octagon_ring(octagon, frames, "octagon_ring")
        },
    }

    for path, meshes in outputs.items():
        write_glb(path, meshes)
        if args.hashes:
            print(f"{path}: sha256 {hashlib.sha256(path.read_bytes()).hexdigest()}")


if __name__ == "__main__":
    main()
