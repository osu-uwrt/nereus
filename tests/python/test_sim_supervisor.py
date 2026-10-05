"""The simulator supervisor's pool switching: pack lookup, run folders, water checks and the process restart."""

import importlib.util
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location(
    "sim_supervisor", ROOT / "integrations/uwrt/launch/sim_supervisor.py"
)
assert _spec and _spec.loader
supervisor: Any = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(supervisor)


def water(**overrides: Any) -> dict[str, Any]:
    parameters = {
        "water_density_kg_m3": 998.2,
        "water_level_m": 0.0,
        "current_m_s": [0.0, 0.0, 0.0],
        "current_oscillation_amplitude_m_s": [0.0, 0.0, 0.0],
        "current_oscillation_frequency_hz": 0.1,
    }
    parameters.update(overrides)
    return {"pool": {"parameters": parameters}}


class SimSupervisorTests(unittest.TestCase):
    def test_pools_and_folders(self) -> None:
        self.assertEqual(supervisor.scenario_folder("rpac").name, "talos_uwrt_rpac")
        folder = ROOT / "content/packs/scenarios/talos_uwrt"
        self.assertEqual(supervisor.scenario_folder(str(folder)), folder.resolve())
        with self.assertRaises(ValueError):
            supervisor.scenario_folder(str(ROOT / "content/packs"))

    def test_run_folders(self) -> None:
        self.assertEqual(supervisor.output_for("/tmp/run", 1), "/tmp/run")
        self.assertEqual(supervisor.output_for("/tmp/run", 3), "/tmp/run-3")

    def test_water_changes(self) -> None:
        # a frequency without an oscillating current changes nothing
        self.assertEqual(
            supervisor.water_changes(water(), water(current_oscillation_frequency_hz=0.0)), []
        )
        self.assertEqual(
            supervisor.water_changes(water(), water(water_density_kg_m3=1025.0)),
            ["water_density_kg_m3"],
        )
        swirl = water(current_oscillation_amplitude_m_s=[0.1, 0.0, 0.0])
        self.assertIn(
            "current_oscillation_frequency_hz",
            supervisor.water_changes(swirl, water(current_oscillation_frequency_hz=0.3)),
        )

    def test_both_uwrt_pools_resolve_with_the_same_water(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            robosub = supervisor.resolve(
                supervisor.scenario_folder("robosub"), Path(directory) / "a.json"
            )
            rpac = supervisor.resolve(
                supervisor.scenario_folder("rpac"), Path(directory) / "b.json"
            )
        self.assertEqual(robosub["pool"]["id"], "robosub_2026")
        self.assertEqual(rpac["pool"]["id"], "rpac_divewell")
        self.assertEqual(supervisor.water_changes(robosub, rpac), [])

    def test_restart_from_a_short_lived_thread_survives(self) -> None:
        # The simulator stand-in records its arguments and runs until stopped; the restart happens from a
        # thread that then ends, as a switch does (the child must not die with that thread).
        with tempfile.TemporaryDirectory() as directory:
            fake = Path(directory) / "fake_sim.py"
            fake.write_text(
                f"#!{sys.executable}\n"
                "import signal, sys, time\n"
                "signal.signal(signal.SIGINT, lambda *_: sys.exit(0))\n"
                "open(sys.argv[3] + '.args', 'w').write(' '.join(sys.argv[1:]))\n"
                "time.sleep(60)\n"
            )
            fake.chmod(0o755)
            simulator = supervisor.Simulator([str(fake), "--no-cameras"], f"{directory}/run")
            simulator.start("first.json")
            thread = threading.Thread(
                target=lambda: (simulator.stop(), simulator.start("second.json"))
            )
            thread.start()
            thread.join()
            time.sleep(1.0)
            self.assertIsNone(
                simulator.process.poll(), "the restarted simulator died with the thread"
            )
            simulator.stop()
            self.assertEqual(simulator.process.returncode, 0)  # SIGINT: a clean exit
            self.assertEqual(
                Path(f"{directory}/run-2.args").read_text(),
                f"second.json --output {directory}/run-2 --no-cameras",
            )
