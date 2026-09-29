#!/usr/bin/env python3
"""Offline test-data capture from the pinned original, never used by the runtime.

Usage: python capture_payload_reference.py ORIGINAL_REPO [OUTPUT_CSV]
Requires NumPy and git. Does not read the native implementation or its content.
"""
import csv
import hashlib
import io
from pathlib import Path
import subprocess
import sys

import numpy as np

REVISION = "07647eebe706f96ea7b76db3cc9802735a146698"
SOURCE = "c_simulator/scripts/payload_model.py"
FIELDS = ("mass", "displaced_volume", "added_mass", "length", "radius", "drag_axial",
          "drag_lateral", "center_of_mass", "center_of_buoyancy", "center_of_drag",
          "angular_damping", "spring_energy")


def capture(repository, output):
    source = subprocess.check_output(["git", "-C", str(repository), "show", f"{REVISION}:{SOURCE}"])
    original = {}
    exec(compile(source, f"{REVISION}:{SOURCE}", "exec"), original)
    cfg = dict(mass=.01222, displaced_volume=1.2e-5, added_mass=.003,
               length=.08299993, radius=.013, drag_axial=.015, drag_lateral=.35,
               center_of_mass=.02, center_of_buoyancy=-.02, center_of_drag=-.025,
               angular_damping=.01, spring_energy=.047, water_level=0.0)
    buffer = io.StringIO(newline="")
    writer = csv.writer(buffer, lineterminator="\n")
    writer.writerow(["# original", REVISION, SOURCE, hashlib.sha256(source).hexdigest()])
    writer.writerow(["# case columns", "id", "model(0=fixed_axis/1=finned)", "neutral_buoyancy",
                     *FIELDS, "water_density", "water_level", "water_vx", "water_vy", "water_vz", "dt"])
    writer.writerow(["# state columns", "case", "tick", "mesh_x", "mesh_y", "mesh_z", "com_vx", "com_vy", "com_vz",
                     *[f"r{i}{j}" for i in range(3) for j in range(3)], "world_wx", "world_wy", "world_wz"])
    for case in range(16):
        model = int(case >= 8)
        variant = case % 8
        parameters = dict(cfg, neutral_buoyancy=(variant == 4))
        # Body +X is the projectile axis; retain full fin orientation.
        a, b, c = .31, -.42, .27
        rx = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
        ry = np.array([[np.cos(b), 0, np.sin(b)], [0, 1, 0], [-np.sin(b), 0, np.cos(b)]])
        rz = np.array([[np.cos(c), -np.sin(c), 0], [np.sin(c), np.cos(c), 0], [0, 0, 1]])
        r = rz @ ry @ rx
        p, v, omega = np.array([-.2, .3, -.6]), np.array([2.3, -.7, .4]), np.array([1., -.5, .7])
        water, density, dt = np.array([.2, -.1, .1]), 998.2, .002
        if variant == 1:  # Entire flight starts above water.
            p[2], v[2] = .2, 2.
        elif variant == 2:  # Entry occurs within the first caller step.
            p[2], v[2], dt = .003, -.5, .01
        elif variant == 3:  # Exit followed by re-entry; stage-dependent immersion.
            p[2], v[2], dt = -.003, 2., .01
        elif variant == 5:  # Exactly at surface is dry, both models.
            p[2], v[2] = 0., .5
        elif variant == 6:  # Damping requires many RK4 substeps per caller step.
            dt, density = .025, 1025.
            parameters["angular_damping"] = .018
            parameters["water_level"] = .4
            p[2] = -.2
        elif variant == 7:  # No damping, asymmetric COM inertia, dry gyroscopic motion.
            parameters["angular_damping"] = 0.
            p[2], v[2], dt = 20., 1., .001
            omega = np.array([5., -3., 2.])
        writer.writerow(["# case", case, model, int(parameters["neutral_buoyancy"]),
                         *[parameters[key] for key in FIELDS], density, parameters["water_level"], *water, dt])
        for tick in range(101):
            values = np.concatenate((p, v, r.ravel(), omega))
            if not np.isfinite(values).all():
                raise RuntimeError(f"Nonfinite original fixture case {case}, tick {tick}")
            writer.writerow([case, tick, *values])
            if tick == 100:
                break
            if model:
                p, v, r, omega = original["advance_rotating"](p, v, r, omega, parameters, water, density, dt)
            else:
                p, v = original["advance"](p, v, r[:, 0], parameters, water, density, dt)
    Path(output).write_text(buffer.getvalue())


if __name__ == "__main__":
    capture(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else Path(__file__).with_name("payload_reference.csv"))
