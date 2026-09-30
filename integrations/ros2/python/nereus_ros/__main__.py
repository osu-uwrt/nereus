"""``python -m nereus_ros <scenario> --output <run dir>``: run one bridged scenario.

Every run resolves the packs, validates the bridge against installed ROS types before the
first step, and writes resolved.json, execution.json and (on exit) summary.json.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

from nereus.pack_runtime import create_runtime
from nereus.packs import PackError, resolve_scenario

from .core import BridgeCore, BridgeError
from .mapping import MappingError

GAP_NOTE = ("Mechanisms, payloads and selected task packs run in the session; pending task "
            "entries are not executed. Selected cameras load their referenced scene assets.")


def _plain(value: Any) -> Any:
    """Detach read-only mapping/tuple views returned by the task runtime."""
    return dict(value) if hasattr(value, "items") else list(value)


def _write(path: Path, document: dict[str, Any]) -> None:
    path.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def execution_record(resolved: Any, core: BridgeCore, sensors: list[str],
                     deferred: tuple[str, ...], duration_ns: int | None) -> dict[str, Any]:
    config = core.config
    return {
        "format": "nereus_ros.execution",
        "version": 1,
        "resolved_content_sha256": resolved.manifest()["content_sha256"],
        "timestep_ns": core.timestep_ns,
        "duration_ns": duration_ns,
        "clock": {"epoch_ns": core.epoch_ns, "reset_policy": core.reset_policy,
                  "real_time_factor": core.real_time_factor, "topic": config["clock"]["topic"]},
        "namespace": config["namespace"],
        "node_name": config.get("node_name", "nereus_bridge"),
        "parameters": {"real_time_factor": "double on the bridge node; 0 pauses stepping and /clock, "
                                           "negative or non-finite values are rejected"},
        "world_frame": core.world_frame,
        "sensors": {"selected": sensors, "not_executed": list(deferred),
                    "selected_without_stream": [name for name in sensors if not any(
                        stream["native"].removeprefix("sensor:").split(".")[0] == name
                        for stream in config["streams"]
                        if stream["native"].startswith("sensor:"))]},
        "streams": [
            {key: stream.get(key) for key in
             ("id", "direction", "topic", "message_type", "native", "frame_id", "rate_hz",
              "format")}
            for stream in config["streams"]
        ],
        "services": [
            {key: service[key] for key in ("id", "service", "service_type", "action")}
            for service in config.get("services", [])
        ],
        "tf": config.get("tf", {}),
        "cameras": None if core.cameras is None else core.cameras.describe(),
        "estimator_alignment": config.get("placement", {}).get("estimator_alignment"),
        "unresolved": {"note": GAP_NOTE, "items": resolved.unresolved},
        "not_executed_config": {
            "scenario.run.options": "defaults passed to task hooks; simulator/run_command start "
                                    "overrides them per run",
            "tf.lookup": "external owners; uses latest live TF, not stamp-matched transforms",
        },
    }


def select_sensors(resolved: Any, selection: list[str] | None) -> tuple[list[str], list[str]]:
    """Separate camera acquisition from native navigation sensors without changing pack data."""
    sensors = {sensor["id"]: sensor for sensor in resolved.robot["sensors"]}
    chosen = ([key for key, value in sensors.items() if value.get("enabled", True)]
              if selection is None else selection)
    if len(chosen) != len(set(chosen)) or not set(chosen) <= sensors.keys():
        raise ValueError("selected sensors must have unique ids from the robot pack")
    if any(not sensors[key].get("enabled", True) for key in chosen):
        raise ValueError("selected sensor is disabled in the robot pack")
    cameras = [key for key in chosen if sensors[key]["type"] == "stereo_camera"]
    return [key for key in chosen if key not in cameras], cameras


def without_cameras(resolved: Any) -> Any:
    """The scenario with every camera sensor stream removed from its bridge (no GPU run)."""
    import dataclasses

    cameras = {item["id"] for item in resolved.robot["sensors"] if item["type"] == "stereo_camera"}
    if resolved.bridge is None:
        return resolved
    bridge = dict(resolved.bridge)
    bridge["streams"] = [
        stream for stream in bridge["streams"]
        if "image" not in stream and not (
            stream["native"].startswith("sensor:")
            and stream["native"].removeprefix("sensor:").split(".")[0] in cameras)
    ]
    return dataclasses.replace(resolved, bridge=bridge)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m nereus_ros")
    parser.add_argument("scenario", type=Path, help="scenario pack folder or file")
    parser.add_argument("--output", type=Path, required=True, help="run directory to create")
    parser.add_argument("--sensors", help="comma-separated robot sensor ids to execute "
                        "(default: every enabled sensor)")
    parser.add_argument("--duration", type=float, help="stop after this many simulated seconds")
    parser.add_argument("--no-cameras", action="store_true",
                        help="run without camera sensors and drop their image/camera_info streams "
                        "(no GPU needed; smoke tests and operator-interface work)")
    parser.add_argument("--validate-only", action="store_true",
                        help="resolve, validate against ROS types and write records; no stepping")
    arguments = parser.parse_args(argv)
    try:
        resolved = resolve_scenario(arguments.scenario)
        if arguments.no_cameras:
            resolved = without_cameras(resolved)
            arguments.sensors = ",".join(
                item["id"] for item in resolved.robot["sensors"]
                if item.get("enabled", True) and item["type"] != "stereo_camera"
                and (arguments.sensors is None or item["id"] in arguments.sensors.split(",")))
        selection = None if arguments.sensors is None else [
            item for item in arguments.sensors.split(",") if item]
        native_ids, camera_ids = select_sensors(resolved, selection)
        pack = create_runtime(resolved, sensor_ids=native_ids)
        cameras = None
        if camera_ids:
            from nereus.pack_cameras import PackCameras

            from .camera_bridge import CameraBridge

            cameras = CameraBridge(resolved, PackCameras(resolved, camera_ids), lambda _: None)
        sensors = [*native_ids, *camera_ids]
        deferred = tuple(key for key in pack.deferred_sensor_ids if key not in camera_ids)
        epoch_ns = time.time_ns()
        duration_ns = None if arguments.duration is None else round(arguments.duration * 1e9)
        arguments.output.mkdir(parents=True, exist_ok=False)
        resolved.dump(arguments.output / "resolved.json")
        # Validate every mapping against installed ROS types before any middleware exists.
        preflight = BridgeCore(resolved, pack, epoch_ns=epoch_ns, cameras=cameras)
        _write(arguments.output / "execution.json",
               execution_record(resolved, preflight, sensors, deferred,
                                duration_ns))
    except (PackError, BridgeError, MappingError, ValueError, OSError, ImportError) as error:
        print(f"nereus_ros: {error}", file=sys.stderr)
        return 1
    if arguments.validate_only:
        print(f"validated {arguments.scenario}; records in {arguments.output}")
        return 0

    import rclpy

    from .node import BridgeNode, run

    rclpy.init()
    node = None
    reason = "duration reached"
    failure = None
    try:
        node = BridgeNode(lambda lookup: BridgeCore(resolved, pack, epoch_ns=epoch_ns,
                                                   lookup=lookup, cameras=cameras),
                          resolved.bridge["namespace"],
                          resolved.bridge.get("node_name", "nereus_bridge"))
        node.start_cameras()
        run(node, duration_ns)
    except KeyboardInterrupt:
        reason = "interrupted"
    except Exception as error:
        reason = f"failed: {type(error).__name__}: {error}"
        failure = error
        raise
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # a repeated Ctrl-C must not cut the records
        camera_error = None
        if cameras is not None:
            try:
                cameras.close()  # Finish workers before writing counters or destroying publishers.
            except Exception as error:
                camera_error = error
                reason = f"failed: {type(error).__name__}: {error}"
        snapshot = pack.runtime.observe()
        core = preflight if node is None else node.core
        tasks = core.session.tasks
        _write(arguments.output / "tasks.json", {
            "format": "nereus_ros.tasks", "version": 1,
            "scores": {} if tasks is None else dict(tasks.snapshot()["scores"]),
            "run": core.session.run_snapshot(),
            "counters": dict(core.session.task_counters),
            "events": json.loads(json.dumps(core.task_events, default=_plain)),
        })
        _write(arguments.output / "summary.json", {
            "format": "nereus_ros.summary", "version": 1, "stop": reason,
            "ticks": snapshot.tick, "elapsed_ns": snapshot.elapsed_ns,
            "generation": snapshot.generation,
            "killed": core.killed, "counters": asdict(core.counters),
            "camera_stats": {} if cameras is None else cameras.stats(),
            "stream_stats": {name: {"acquired": stream.stats.acquired,
                                    "delivered": stream.stats.delivered,
                                    "unavailable": stream.stats.unavailable,
                                    "dropped_pending": stream.stats.dropped_pending,
                                    "dropped_delivered": stream.stats.dropped_delivered}
                             for name, stream in pack.streams.items()},
        })
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if camera_error is not None and failure is None:
            raise camera_error
    return 0


if __name__ == "__main__":
    sys.exit(main())
