"""Launch isolated old/new UWRT hold trials and compare their measured errors."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import signal
import subprocess
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from riptide_sim_config.profiles import resolve

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def run_trial(kind, output, domain, scenario, sdk):
    output.mkdir()
    env = dict(
        os.environ,
        ROS_DOMAIN_ID=str(domain),
        ROS_LOCALHOST_ONLY="1",
        RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
    )
    env["PYTHONPATH"] = os.pathsep.join(
        [str(ROOT / "integrations/ros2/python"), str(sdk), env.get("PYTHONPATH", "")]
    )
    if kind == "old":
        config = resolve(output=output / "resolved", overrides={"with_tasks": "false"})
        simulator = [
            "ros2",
            "launch",
            "c_simulator",
            "physics_simulator.launch.py",
            "robot:=talos",
            "with_tasks:=false",
            f"resolved_config:={config}",
        ]
    else:
        simulator = [
            sys.executable,
            "-m",
            "nereus_ros",
            str(scenario),
            "--output",
            str(output / "bridge"),
        ]
    commands = [simulator, ["ros2", "launch", str(HERE / "stack.launch.py")]]
    processes, logs = [], []
    try:
        for index, command in enumerate(commands):
            log = (output / f"process-{index}.log").open("w")
            logs.append(log)
            processes.append(
                subprocess.Popen(
                    command,
                    env=env,
                    cwd=ROOT,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
        command = [sys.executable, str(HERE / "hold.py"), "--output", str(output / "hold")]
        (output / "commands.json").write_text(json.dumps(commands + [command], indent=2) + "\n")
        with (output / "hold.log").open("w") as log:
            measured = subprocess.Popen(
                command,
                env=env,
                cwd=ROOT,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            processes.append(measured)
            deadline = time.monotonic() + 190
            while measured.poll() is None:
                if any(p.poll() is not None for p in processes[:-1]):
                    raise RuntimeError(f"{kind} stack process exited; inspect {output}")
                if time.monotonic() > deadline:
                    raise RuntimeError(f"{kind} hold timed out; inspect {output}")
                time.sleep(0.2)
            if measured.returncode:
                raise RuntimeError(f"{kind} hold failed; inspect {output}/hold.log")
        return json.loads((output / "hold/summary.json").read_text())
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()


def compare(old, new):
    failures = []
    if len(old["phases"]) != 2 or len(new["phases"]) != 2:
        return ["both trials must contain two completed holds"]
    if new["placement_alignment"]["position_error_m"] > (
        2 * old["placement_alignment"]["position_error_m"] + 0.01
    ):
        failures.append("post-placement estimation error exceeds 2x baseline plus 1 cm")
    for phase, (left, right) in enumerate(zip(old["phases"], new["phases"], strict=True)):
        if left["target"] != right["target"]:
            failures.append(f"phase {phase}: different targets")
        for label, trial in (("old", left), ("new", right)):
            if not trial["motion_enabled"] or len(trial["last_forces"]) != 8:
                failures.append(f"{label} phase {phase}: controller inactive")
            for name, max_age in (
                ("truth", 0.15),
                ("estimate", 0.2),
                ("forces", 0.15),
                ("motion", 1.5),
            ):
                if trial["latest_age_s"].get(name, math.inf) > max_age:
                    failures.append(f"{label} phase {phase}: stale {name}")
            for kind, data in trial["metrics"].items():
                if not all(math.isfinite(value) for value in data.values()):
                    failures.append(f"{label} phase {phase} {kind}: nonfinite evidence")
                if not 0.95 <= data["real_time_factor"] <= 1.05:
                    failures.append(f"{label} phase {phase} {kind}: not real time")
                if data["depth_rms_m"] > 0.02 or data["heading_rms_rad"] > math.radians(2):
                    failures.append(f"{label} phase {phase} {kind}: absolute hold error")
        for kind in ("truth", "estimate"):
            a, b = left["metrics"][kind], right["metrics"][kind]
            bounds = {
                "depth_rms_m": max(0.01, 2 * a["depth_rms_m"] + 0.003),
                "heading_rms_rad": max(
                    math.radians(1), 2 * a["heading_rms_rad"] + math.radians(0.2)
                ),
                "depth_max_m": a["depth_max_m"] + 0.02,
                "heading_max_rad": a["heading_max_rad"] + math.radians(2),
            }
            for metric, limit in bounds.items():
                if not math.isfinite(b[metric]) or b[metric] > limit:
                    failures.append(f"phase {phase} {kind} {metric}: {b[metric]} > {limit}")
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=195)
    parser.add_argument("--repeat", type=int, default=2)
    parser.add_argument(
        "--scenario", type=Path, default=ROOT / "content/packs/scenarios/talos_uwrt"
    )
    parser.add_argument("--sdk", type=Path, default=ROOT / "build/step2-sdk")
    args = parser.parse_args()
    if args.repeat < 2 or not 1 <= args.domain <= 232 - 2 * args.repeat:
        parser.error("use at least two repeats and private DDS domains between 1 and 230")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    machine = {
        "system": platform.platform(),
        "machine": platform.machine(),
        "logical_cpus": os.cpu_count(),
        "memory_bytes": os.sysconf("SC_PHYS_PAGES") * os.sysconf("SC_PAGE_SIZE"),
    }
    for name in ("cpu.max", "memory.max"):
        path = Path("/sys/fs/cgroup") / name
        if path.exists():
            machine[name] = path.read_text().strip()
    (output / "machine.json").write_text(json.dumps(machine, indent=2) + "\n")
    files = [
        Path(get_package_share_directory("riptide_controllers2")) / "config/talos_autoff.yaml",
        ROOT.parent / "src/riptide_core/riptide_descriptions/config/talos.yaml",
        Path(get_package_share_directory("riptide_hardware2")) / "cfg/talos_ekf.yaml",
    ]
    hashes = {str(p): digest(p) for p in files}
    (output / "stack-inputs.json").write_text(json.dumps(hashes, indent=2) + "\n")
    failures, trials = [], []
    for trial in range(args.repeat):
        old = run_trial(
            "old",
            output / f"old-{trial}",
            args.domain + 2 * trial,
            args.scenario.resolve(),
            args.sdk.resolve(),
        )
        new_path = output / f"new-{trial}"
        new = run_trial(
            "new",
            new_path,
            args.domain + 2 * trial + 1,
            args.scenario.resolve(),
            args.sdk.resolve(),
        )
        summary = json.loads((new_path / "bridge/summary.json").read_text())
        counters = summary["counters"]
        errors = compare(old, new)
        for key in ("rejected_commands", "unavailable_samples", "alignments_failed"):
            if any(counters[key].values()):
                errors.append(f"bridge {key}: {counters[key]}")
        for trigger in ("startup", "placement"):
            sent = counters["alignments"].get(trigger, 0)
            superseded = counters["alignments_superseded"].get(trigger, 0)
            acknowledged = counters["alignments_acknowledged"].get(trigger, 0)
            if sent + superseded != 1 or acknowledged != sent:
                errors.append(f"unaccounted or unacknowledged EKF alignment: {trigger}")
        if counters["alignments"].get("placement", 0) < 1:
            errors.append("no EKF alignment after final placement")
        for sensor, stats in summary["stream_stats"].items():
            if stats["dropped_pending"]:
                errors.append(f"bridge dropped pending sensor samples: {sensor}")
        trials.append({"old": old, "new": new, "bridge": summary, "failures": errors})
        failures.extend(f"trial {trial}: {error}" for error in errors)
        print(json.dumps({"trial": trial, "failures": errors}), flush=True)
    if any(digest(p) != hashes[str(p)] for p in files):
        failures.append("stack configuration changed during comparison")
    result = {"passed": not failures, "failures": failures, "trials": trials}
    (output / "comparison.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"passed": not failures, "failures": failures, "evidence": str(output)}))
    return bool(failures)


if __name__ == "__main__":
    sys.exit(main())
