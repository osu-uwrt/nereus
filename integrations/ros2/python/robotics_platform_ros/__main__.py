"""``python -m robotics_platform_ros <scenario> --output <run dir>``: run one bridged scenario.

Every run resolves the packs, validates the bridge against installed ROS types before the
first step, and writes resolved.json, execution.json and (on exit) summary.json.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any

from robotics_platform.pack_runtime import create_runtime
from robotics_platform.packs import PackError, resolve_scenario

from .core import BridgeCore, BridgeError
from .mapping import MappingError

GAP_NOTE = ("Diagnostic only: this bridge executes no task scoring, mechanisms or rendering, "
            "so declared task assets, hook modules and pending items are not loaded.")


def _write(path: Path, document: dict[str, Any]) -> None:
    path.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def execution_record(resolved: Any, core: BridgeCore, sensors: list[str],
                     deferred: tuple[str, ...], duration_ns: int | None) -> dict[str, Any]:
    config = core.config
    return {
        "format": "robotics_platform_ros.execution",
        "version": 1,
        "resolved_content_sha256": resolved.manifest()["content_sha256"],
        "timestep_ns": core.timestep_ns,
        "duration_ns": duration_ns,
        "clock": {"epoch_ns": core.epoch_ns, "reset_policy": core.reset_policy,
                  "real_time_factor": core.real_time_factor, "topic": config["clock"]["topic"]},
        "namespace": config["namespace"],
        "world_frame": core.world_frame,
        "sensors": {"selected": sensors, "not_executed": list(deferred),
                    "selected_without_stream": [name for name in sensors if not any(
                        stream["native"].removeprefix("sensor:").split(".")[0] == name
                        for stream in config["streams"]
                        if stream["native"].startswith("sensor:"))]},
        "streams": [
            {key: stream[key] for key in
             ("id", "direction", "topic", "message_type", "native", "frame_id", "rate_hz")}
            for stream in config["streams"]
        ],
        "services": [
            {key: service[key] for key in ("id", "service", "service_type", "action")}
            for service in config.get("services", [])
        ],
        "tf": config.get("tf", {}),
        "estimator_alignment": config.get("placement", {}).get("estimator_alignment"),
        "unresolved": {"note": GAP_NOTE, "items": resolved.unresolved},
        "not_executed_config": {
            "scenario.run": "task scoring/run control is not executed by this bridge",
            "tf.lookup": "external owners; uses latest live TF, not stamp-matched transforms",
            "services[].required_from_step": "planning metadata; never an execution switch",
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m robotics_platform_ros")
    parser.add_argument("scenario", type=Path, help="scenario pack folder or file")
    parser.add_argument("--output", type=Path, required=True, help="run directory to create")
    parser.add_argument("--sensors", help="comma-separated robot sensor ids to execute "
                        "(default: every enabled sensor)")
    parser.add_argument("--duration", type=float, help="stop after this many simulated seconds")
    parser.add_argument("--validate-only", action="store_true",
                        help="resolve, validate against ROS types and write records; no stepping")
    arguments = parser.parse_args(argv)
    try:
        resolved = resolve_scenario(arguments.scenario)
        selection = None if arguments.sensors is None else [
            item for item in arguments.sensors.split(",") if item]
        pack = create_runtime(resolved, sensor_ids=selection)
        sensors = list(pack.streams)
        epoch_ns = time.time_ns()
        duration_ns = None if arguments.duration is None else round(arguments.duration * 1e9)
        arguments.output.mkdir(parents=True, exist_ok=False)
        resolved.dump(arguments.output / "resolved.json")
        # Validate every mapping against installed ROS types before any middleware exists.
        preflight = BridgeCore(resolved, pack, epoch_ns=epoch_ns)
        _write(arguments.output / "execution.json",
               execution_record(resolved, preflight, sensors, pack.deferred_sensor_ids,
                                duration_ns))
    except (PackError, BridgeError, MappingError, ValueError, FileExistsError) as error:
        print(f"robotics_platform_ros: {error}", file=sys.stderr)
        return 1
    if arguments.validate_only:
        print(f"validated {arguments.scenario}; records in {arguments.output}")
        return 0

    import rclpy

    from .node import BridgeNode, run

    rclpy.init()
    node = BridgeNode(lambda lookup: BridgeCore(resolved, pack, epoch_ns=epoch_ns, lookup=lookup),
                      resolved.bridge["namespace"])
    reason = "duration reached"
    try:
        run(node, duration_ns)
    except KeyboardInterrupt:
        reason = "interrupted"
    except Exception as error:
        reason = f"failed: {type(error).__name__}: {error}"
        raise
    finally:
        snapshot = pack.runtime.observe()
        _write(arguments.output / "summary.json", {
            "format": "robotics_platform_ros.summary", "version": 1, "stop": reason,
            "ticks": snapshot.tick, "elapsed_ns": snapshot.elapsed_ns,
            "generation": snapshot.generation,
            "killed": node.core.killed, "counters": asdict(node.core.counters),
            "stream_stats": {name: {"acquired": stream.stats.acquired,
                                    "delivered": stream.stats.delivered,
                                    "unavailable": stream.stats.unavailable,
                                    "dropped_pending": stream.stats.dropped_pending,
                                    "dropped_delivered": stream.stats.dropped_delivered}
                             for name, stream in pack.streams.items()},
        })
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
