#!/usr/bin/env python3
"""Runs nereus-sim and restarts it in another scenario on request (the viewer's View > Pool).

    sim_supervisor.py --resolved FILE --scenario FOLDER --output DIR [--namespace /talos] -- NEREUS_SIM [ARGS...]

The simulator runs as `NEREUS_SIM <resolved> --output <dir> ARGS`. A std_msgs/String on
<namespace>/simulator/load_scenario names a scenario pack folder (or a pool from POOLS); the supervisor resolves
it, stops the simulator (SIGINT: it writes its run records) and starts it again in the new scenario, in a new
output folder (<output>-2, <output>-3, ...). The robot stack keeps running; the new simulator's clock starts from
the wall clock, so ROS time keeps going forward (unless the old run was faster than real time).

Its state is latched on <namespace>/simulator/supervisor as JSON:
{"state": "running" | "switching" | "stopped" | "error", "scenario": folder, "pool": pool id, "output": dir,
 "message": text}. The viewer reads it to know the simulator can switch pools.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import queue
import signal
import subprocess
import sys
import threading
from concurrent.futures import Future
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[3]
# The resolver needs the pack tools' dependencies (ruamel.yaml, jsonschema), which ./build.sh installs into
# .venv; ros2 launch itself runs under the system Python.
PYTHON = str(ROOT / ".venv/bin/python") if (ROOT / ".venv/bin/python").exists() else sys.executable
SCENARIOS = ROOT / "content/packs/scenarios"
# pool:=<name> picks the matching UWRT scenario.
POOLS = {"robosub": "talos_uwrt", "rpac": "talos_uwrt_rpac"}
# Pool parameters the MPC's generated plant model reads (mpc_sim_model.py).
WATER = (
    "water_density_kg_m3",
    "water_level_m",
    "current_m_s",
    "current_oscillation_amplitude_m_s",
    "current_oscillation_frequency_hz",
)


def scenario_folder(name: str) -> Path:
    """A scenario pack folder from a path or a pool name in POOLS."""
    if name in POOLS:
        return SCENARIOS / POOLS[name]
    folder = Path(name).expanduser()
    if not (folder / "scenario.yaml").is_file():
        raise ValueError(f"not a scenario pack (no scenario.yaml): {name}")
    return folder.resolve()


def resolve(scenario: Path | str, out: Path | str) -> dict[str, Any]:
    """`python -m nereus.packs resolve` into `out`; the resolved document."""
    environment = {
        **os.environ,
        "PYTHONPATH": os.pathsep.join([str(ROOT / "python/src"), os.environ.get("PYTHONPATH", "")]),
    }
    result = subprocess.run(
        [PYTHON, "-m", "nereus.packs", "resolve", str(scenario), "-o", str(out)],
        cwd=str(ROOT),
        env=environment,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        lines = (result.stderr or result.stdout).strip().splitlines()
        raise RuntimeError(lines[-1] if lines else "resolve failed")
    document: dict[str, Any] = json.loads(Path(out).read_text())
    return document


def output_for(base: str, run: int) -> str:
    """Run 1 writes to `base`, later runs to `base-2`, `base-3`, ..."""
    return base if run == 1 else f"{base}-{run}"


def water_changes(before: dict[str, Any], after: dict[str, Any]) -> list[str]:
    """Pool water parameters that differ between two resolved documents (an oscillation frequency only counts
    when either pool's current oscillates)."""
    a, b = before["pool"]["parameters"], after["pool"]["parameters"]
    amplitude = "current_oscillation_amplitude_m_s"
    still = not any(a.get(amplitude, [0])) and not any(b.get(amplitude, [0]))
    return [
        key
        for key in WATER
        if a.get(key) != b.get(key) and not (still and key == "current_oscillation_frequency_hz")
    ]


def _die_with_parent() -> None:
    """In the simulator: SIGTERM if the supervisor dies without stopping it (Linux). The signal follows the
    *thread* that started the process, so Simulator starts it from a thread that lives as long as it does."""
    libc = ctypes.CDLL(None, use_errno=True)
    libc.prctl(1, signal.SIGTERM)  # PR_SET_PDEATHSIG


class Simulator:
    """The nereus-sim process: started, stopped and restarted in another scenario, from any thread."""

    def __init__(self, command: list[str], output: str) -> None:
        self.binary, self.extra = command[0], command[1:]
        self.base = output
        self.run = 0  # runs started so far; run N writes to output_for(base, N)
        self.process: subprocess.Popen[bytes] | None = None
        self.stopping = False  # set once a stop was requested (or an exit reported) for this run
        # Start requests for _starter: the command and a future for the started process.
        self._starts: queue.Queue[tuple[list[str], Future[subprocess.Popen[bytes]]]] = queue.Queue()
        threading.Thread(target=self._starter, daemon=True).start()

    def _starter(self) -> None:  # the one thread that starts the simulator (see _die_with_parent)
        while True:
            command, result = self._starts.get()
            try:
                result.set_result(
                    subprocess.Popen(command, cwd=str(ROOT), preexec_fn=_die_with_parent)
                )
            except Exception as error:  # noqa: BLE001  (handed to the caller)
                result.set_exception(error)

    def start(self, resolved: str) -> str:
        """Starts the next run on `resolved`; returns its output folder."""
        self.run += 1
        output = output_for(self.base, self.run)
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        self.stopping = False
        result: Future[subprocess.Popen[bytes]] = Future()
        self._starts.put(([self.binary, resolved, "--output", output, *self.extra], result))
        self.process = result.result()
        return output

    def stop(self, timeout: float = 15) -> None:
        """Stops the running simulator with SIGINT, killing it after `timeout` seconds."""
        process, self.stopping = self.process, True
        if process is None or process.poll() is not None:
            return
        process.send_signal(signal.SIGINT)  # as Ctrl-C: the simulator writes its records
        try:
            process.wait(timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def main(argv: list[str] | None = None) -> int:
    """Parses the arguments, starts the first run and serves load_scenario requests until shutdown."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--resolved", required=True, help="the first scenario, already resolved")
    parser.add_argument("--scenario", required=True, help="its scenario pack folder")
    parser.add_argument(
        "--output", required=True, help="run records of the first run (later runs: -2, -3, ...)"
    )
    parser.add_argument("--namespace", default="/talos")
    parser.add_argument("command", nargs=argparse.REMAINDER, help="-- nereus-sim [its options]")
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("missing the nereus-sim command after --")

    # ROS is imported only once the arguments are valid.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    # The node, its latched status topic, and the state shared by the callbacks and the switcher thread:
    # `lock` is held for a whole switch; `current` and `document` describe the running scenario.
    rclpy.init()
    node = rclpy.create_node("sim_supervisor")
    latched = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    topic = args.namespace.rstrip("/") + "/simulator"
    status_publisher = node.create_publisher(String, topic + "/supervisor", latched)
    simulator = Simulator(command, args.output)
    lock = threading.Lock()
    current: dict[str, Any] = {"scenario": str(Path(args.scenario).resolve())}
    document = json.loads(Path(args.resolved).read_text())

    # Publish the latched status (see the module docstring) and log it.
    def publish(state: str, message: str = "") -> None:
        status = {
            "state": state,
            "scenario": current["scenario"],
            "pool": document["pool"]["id"],
            "output": output_for(simulator.base, max(simulator.run, 1)),
            "message": message,
        }
        status_publisher.publish(String(data=json.dumps(status)))
        node.get_logger().info(f"{state}: {status['pool']} {message}".rstrip())

    # Resolve the requested scenario first (a bad request leaves the running simulator alone), then
    # restart the simulator on it.
    def switch(name: str) -> None:
        nonlocal document
        try:
            folder = scenario_folder(name)
            output = output_for(simulator.base, simulator.run + 1)
            resolved = resolve(folder, f"{output}.resolved.json")
        except (ValueError, RuntimeError, OSError) as error:
            publish("error", f"{name}: {error}")
            return
        changes = water_changes(document, resolved)
        simulator.stop()
        current["scenario"], document = str(folder), resolved
        simulator.start(f"{output}.resolved.json")
        note = (
            f"water differs from the last pool ({', '.join(changes)}): a generated MPC model is stale"
            if changes
            else ""
        )
        publish("running", note)

    # Switches run one at a time, off the ROS callback thread (resolving takes seconds).
    requests: queue.Queue[str] = queue.Queue()

    def switcher() -> None:
        while True:
            name = requests.get()
            with lock:
                switch(name)

    threading.Thread(target=switcher, daemon=True).start()

    # load_scenario callback: one switch at a time; requests during a switch are dropped.
    def requested(message: String) -> None:
        if lock.locked() or not requests.empty():
            node.get_logger().warning(
                f"ignoring load_scenario {message.data}: a switch is in progress"
            )
            return
        publish("switching", message.data)
        requests.put(message.data)

    node.create_subscription(String, topic + "/load_scenario", requested, 10)

    def watch() -> None:  # a simulator that exits by itself (crash, or Ctrl-C on it alone)
        if lock.acquire(blocking=False):
            process = simulator.process
            if process is not None and process.poll() is not None and not simulator.stopping:
                simulator.stopping = True
                publish("stopped", f"nereus-sim exited with code {process.returncode}")
            lock.release()

    node.create_timer(1.0, watch)

    # First run on the scenario sim.launch.py already resolved, then serve requests until shutdown.
    simulator.start(args.resolved)
    publish("running")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        simulator.stop()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
