"""Exercise pack cameras through the actual UWRT detector, mapper, controllers and EKF.

Private ROS domain only. The mapper starts with a deliberately displaced gate-repair
prior; detections are compared to physical pack geometry using stamped simulation truth.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import time
from collections import Counter, deque
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory as share
from compare import ROOT, stop
from ruamel.yaml import YAML


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def prepare(output):
    from robotics_platform import _native as native
    from robotics_platform.pack_runtime import create_runtime
    from robotics_platform.packs import resolve_scenario

    packs = output / "packs"
    shutil.copytree(ROOT / "content/packs", packs)
    yaml = YAML()
    robot_path = packs / "robots/talos/robot.yaml"
    robot = yaml.load(robot_path)
    for sensor in robot["sensors"]:
        if sensor["type"] == "stereo_camera":
            sensor["enabled"] = True
    with robot_path.open("w") as target:
        yaml.dump(robot, target)
    bridge_path = packs / "bridges/uwrt_talos/bridge.yaml"
    bridge = yaml.load(bridge_path)
    proposals = yaml.load(ROOT / "proposals/step1/packs/bridges/uwrt_talos/bridge.yaml")
    ids = {stream["id"] for stream in bridge["streams"]}
    bridge["streams"].extend(
        copy.deepcopy(
            [
                stream
                for stream in proposals["streams"]
                if stream["native"].startswith(("sensor:ffc.", "sensor:dfc."))
                and stream["id"] not in ids
            ]
        )
    )
    with bridge_path.open("w") as target:
        yaml.dump(bridge, target)
    scenario_path = packs / "scenarios/talos_uwrt/scenario.yaml"
    scenario = yaml.load(scenario_path)
    resolved = resolve_scenario(scenario_path)
    pack = create_runtime(resolved, sensor_ids=[])
    placement = next(p for p in scenario["task_placements"] if p["task"] == "gate")
    yaw = math.radians(placement["yaw_deg"])
    gate = native.Pose()
    gate.translation = placement["position_m"]
    gate.orientation_wxyz = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    root = native.Pose()
    root.orientation_wxyz = [math.cos((yaw + math.pi) / 2), 0, 0, math.sin((yaw + math.pi) / 2)]
    eye = pack.frames.from_root("ffc_left_optical")
    root.translation = gate.apply([2.5, 0, 0.5]) - root.apply(eye.translation)
    scenario["initial"]["position_m"] = root.translation.tolist()
    scenario["initial"]["orientation_wxyz"] = root.orientation_wxyz.tolist()
    with scenario_path.open("w") as target:
        yaml.dump(scenario, target)
    resolved = resolve_scenario(scenario_path)
    resolved.dump(output / "resolved.json")
    repair = next(
        frame
        for task in resolved.task_definitions
        if task["id"] == "gate"
        for frame in task["frames"]
        if frame["id"] == "gate_repair"
    )
    expected_local = np.array(repair["position_m"], float)
    reference = root.compose(pack.frames.from_root(resolved.robot["reference_frame"]))
    reference_eye = pack.frames.from_root(resolved.robot["reference_frame"]).inverse().compose(eye)
    mapper_source = Path(share("riptide_mapping2")) / "config/config.yaml"
    mapper = yaml.load(mapper_source)
    prior = expected_local + [0, 0.4, 0]
    configured = mapper["/talos/riptide_mapping2"]["ros__parameters"]["init_data"]["gate_repair"][
        "pose"
    ]
    for axis, value in zip("xyz", prior):
        configured[axis] = float(value)
    mapper_path = output / "mapping.yaml"
    with mapper_path.open("w") as target:
        yaml.dump(mapper, target)
    write(
        output / "fixture.json",
        {
            "camera_distance_m": 2.5,
            "expected_local": expected_local.tolist(),
            "mapper_prior_local": prior.tolist(),
            "expected_world": gate.apply(expected_local).tolist(),
            "target_reference_position": reference.translation.tolist(),
            "target_reference_orientation_wxyz": reference.orientation_wxyz.tolist(),
            "calibration": {
                sensor["id"]: sensor["parameters"]
                for sensor in resolved.robot["sensors"]
                if sensor["type"] == "stereo_camera"
            },
            "model_sha256": {
                str(file.relative_to(Path(share("tensor_detector")))): hashlib.sha256(
                    file.read_bytes()
                ).hexdigest()
                for camera in ("rs26_ffc_finals_ncnn_model", "dfc_rs_26_ncnn_model")
                for file in (Path(share("tensor_detector")) / "weights" / camera).rglob("*")
                if file.is_file()
            },
            "mapping_source_sha256": hashlib.sha256(mapper_source.read_bytes()).hexdigest(),
            "mapping_override_sha256": hashlib.sha256(mapper_path.read_bytes()).hexdigest(),
            "thresholds": {
                "minimum_detections": 35,
                "detection_error_p95_m": 0.30,
                "final_mapping_error_m": 0.25,
                "mapping_movement_m": 0.25,
            },
        },
    )
    return (
        scenario_path,
        mapper_path,
        reference,
        reference_eye,
        gate.apply(expected_local),
        expected_local,
        prior,
    )


def measure(args, fixture, processes):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from riptide_msgs2.msg import ControllerCommand, KillSwitchReport, MappingTargetInfo
    from riptide_msgs2.srv import MappingTarget
    from robotics_platform import _native as native
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    from std_msgs.msg import Bool, Float32MultiArray
    from std_srvs.srv import SetBool, Trigger
    from vision_msgs.msg import Detection3DArray

    _, _, reference, reference_eye, expected_world, expected_local, prior = fixture
    rclpy.init()
    node = Node("platform_perception_acceptance")
    latest, rows, truth = {}, [], deque(maxlen=10000)
    arrays, infos = [], {}
    phase = ["startup"]
    fixture_data = json.loads((args.output / "fixture.json").read_text())
    sign_normal = -reference.apply([1.0, 0.0, 0.0]) + reference.translation

    def angle(q, target):
        norm = float(np.linalg.norm(q))
        if not math.isfinite(norm) or abs(norm - 1) > 1e-5:
            return float("inf")
        return 2 * math.acos(min(1.0, abs(float(np.dot(q, target)))))

    stamps = {
        f"{camera}/{kind}": set() for camera in ("ffc", "dfc") for kind in ("rgb", "depth", "info")
    }

    def stamp_ns(s):
        return s.sec * 1_000_000_000 + s.nanosec

    def navigation(kind, message):
        latest[kind] = message
        if kind == "truth":
            p = message.pose.pose
            pose = native.Pose()
            pose.translation = [p.position.x, p.position.y, p.position.z]
            pose.orientation_wxyz = [
                p.orientation.w,
                p.orientation.x,
                p.orientation.y,
                p.orientation.z,
            ]
            truth.append((stamp_ns(message.header.stamp), pose))
            error = None
            if "estimate" in latest:
                ep = latest["estimate"].pose.pose.position
                error = float(np.linalg.norm(pose.translation - [ep.x, ep.y, ep.z]))
            rows.append(
                {
                    "kind": "hold",
                    "phase": phase[0],
                    "wall": time.monotonic(),
                    "stamp_ns": stamp_ns(message.header.stamp),
                    "position_error_m": float(
                        np.linalg.norm(pose.translation - reference.translation)
                    ),
                    "attitude_error_rad": angle(pose.orientation_wxyz, reference.orientation_wxyz),
                    "ekf_position_error_m": error,
                }
            )

    def mapped(message):
        p = message.pose.pose.position
        rows.append(
            {
                "kind": "mapping",
                "phase": phase[0],
                "covariance": list(message.pose.covariance),
                "orientation_wxyz": [
                    message.pose.pose.orientation.w,
                    message.pose.pose.orientation.x,
                    message.pose.pose.orientation.y,
                    message.pose.pose.orientation.z,
                ],
                "wall": time.monotonic(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame": message.header.frame_id,
                "position": [p.x, p.y, p.z],
                "error_m": float(np.linalg.norm(np.array([p.x, p.y, p.z]) - expected_local)),
                "movement_m": float(np.linalg.norm(np.array([p.x, p.y, p.z]) - prior)),
            }
        )

    def detections(message):
        arrays.append(
            {
                "wall": time.monotonic(),
                "stamp_ns": stamp_ns(message.header.stamp),
                "frame": message.header.frame_id,
                "phase": phase[0],
            }
        )
        for detection in message.detections:
            for result in detection.results:
                if result.hypothesis.class_id != "gate_repair":
                    continue
                header = detection.header
                if not truth:
                    continue
                acquired = stamp_ns(header.stamp)
                stamp, world_reference = min(truth, key=lambda item: abs(item[0] - acquired))
                p = result.pose.pose.position
                world_eye = world_reference.compose(reference_eye)
                world = world_eye.apply([p.x, p.y, p.z])
                q = result.pose.pose.orientation
                orientation = [q.w, q.x, q.y, q.z]
                det_pose = native.Pose()
                det_pose.orientation_wxyz = orientation
                normal = world_eye.compose(det_pose).apply([0.0, 0.0, 1.0]) - world_eye.translation
                normal_error = math.acos(float(np.clip(np.dot(normal, sign_normal), -1, 1)))
                rows.append(
                    {
                        "kind": "detection",
                        "phase": phase[0],
                        "headers_match": header == message.header,
                        "orientation_wxyz": orientation,
                        "normal_error_rad": normal_error,
                        "wall": time.monotonic(),
                        "stamp_ns": acquired,
                        "frame": header.frame_id,
                        "truth_gap_ns": abs(stamp - acquired),
                        "camera_position": [p.x, p.y, p.z],
                        "world_position": world.tolist(),
                        "confidence": result.hypothesis.score,
                        "error_m": float(np.linalg.norm(world - expected_world)),
                    }
                )

    for kind, topic, cls in (
        ("truth", "simulator/ground_truth", Odometry),
        ("estimate", "odometry/filtered", Odometry),
        ("motion", "controller/motion_enabled", Bool),
        ("forces", "thruster_forces", Float32MultiArray),
        ("mapping_status", "state/mapping", MappingTargetInfo),
    ):
        node.create_subscription(
            cls, "/talos/" + topic, lambda msg, k=kind: navigation(k, msg), 100
        )
    node.create_subscription(Detection3DArray, "/talos/detected_objects", detections, 100)
    node.create_subscription(PoseWithCovarianceStamped, "/talos/mapping/gate_repair", mapped, 100)
    topics = []
    for camera in ("ffc", "dfc"):
        for kind, path, cls in (
            ("rgb", "rgb/image_rect_color/compressed", CompressedImage),
            ("depth", "depth/depth_registered", Image),
            ("info", "rgb/camera_info", CameraInfo),
        ):
            topic = f"/talos/{camera}/zed_node/{path}"
            topics.append(topic)

            def image(message, key=f"{camera}/{kind}"):
                stamps[key].add(stamp_ns(message.header.stamp))
                latest[key] = message.header.frame_id
                if key.endswith("/info"):
                    infos[key] = {
                        "width": message.width,
                        "height": message.height,
                        "k": list(message.k),
                        "p": list(message.p),
                        "frame": message.header.frame_id,
                    }
                if key.endswith("/rgb") and key not in infos:
                    (args.output / (key.replace("/", "_") + ".jpg")).write_bytes(
                        bytes(message.data)
                    )
                    infos[key] = True

            node.create_subscription(
                cls, topic, image, QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
            )
    linear = node.create_publisher(ControllerCommand, "/talos/controller/linear", 10)
    angular = node.create_publisher(ControllerCommand, "/talos/controller/angular", 10)
    kill = node.create_publisher(KillSwitchReport, "/talos/command/software_kill", 10)
    report = KillSwitchReport(
        kill_switch_id=1, sender_id="platform_perception_acceptance", switch_asserting_kill=True
    )
    node.create_timer(0.25, lambda: kill.publish(report))
    teleop = node.create_client(SetBool, "/talos/setTeleop")
    mapping_target = node.create_client(MappingTarget, "/talos/mapping_target")
    reset_mapping = node.create_client(Trigger, "/talos/mapping/reset_mapping")
    switch_camera = node.create_client(SetBool, "/talos/set_camera_is_dfc")
    outcome = {"passed": False}

    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            for process in processes:
                if process.poll() is not None:
                    raise RuntimeError(f"stack process exited: {process.args}")
            rclpy.spin_once(node, timeout_sec=min(0.02, max(0, until - time.monotonic())))

    def call(client, request):
        until = time.monotonic() + 10
        while not client.service_is_ready():
            if time.monotonic() > until:
                raise RuntimeError(f"service unavailable: {client.srv_name}")
            spin(0.05)
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() > until:
                raise RuntimeError(f"service timed out: {client.srv_name}")
            spin(0.05)
        return future.result()

    def target_mapping(locked):
        request = MappingTarget.Request()
        request.target_info.target_object = "gate_repair"
        request.target_info.lock_map = locked
        call(mapping_target, request)
        until = time.monotonic() + 5
        while (
            "mapping_status" not in latest
            or latest["mapping_status"].target_object != "gate_repair"
            or latest["mapping_status"].lock_map != locked
        ):
            if time.monotonic() > until:
                raise RuntimeError("mapping target/lock not acknowledged")
            spin(0.05)

    def switch(to_dfc):
        name = "dfc" if to_dfc else "ffc"
        phase[0] = f"switch_{name}"
        before = time.monotonic()
        response = call(switch_camera, SetBool.Request(data=to_dfc))
        if not response.success:
            raise RuntimeError(response.message)
        until = time.monotonic() + 20
        while (
            len(
                {
                    a["stamp_ns"]
                    for a in arrays
                    if a["wall"] > before
                    and a["frame"] == f"talos/{name}_left_camera_optical_frame"
                }
            )
            < 3
        ):
            if time.monotonic() > until:
                raise RuntimeError(f"no fresh detector output after {name} switch")
            spin(0.05)
        return {"camera": name, "wall_s": time.monotonic() - before}

    try:
        deadline = time.monotonic() + 40
        while not all(
            key in latest for key in ("truth", "estimate", "motion", "ffc/info", "dfc/info")
        ):
            if time.monotonic() > deadline:
                raise RuntimeError(f"startup timeout: {list(latest)}")
            spin(0.1)
        while True:
            if time.monotonic() > deadline:
                raise RuntimeError("EKF alignment timeout")
            a, b = latest["truth"].pose.pose.position, latest["estimate"].pose.pose.position
            if np.linalg.norm([a.x - b.x, a.y - b.y, a.z - b.z]) < 0.10:
                break
            spin(0.1)
        while True:
            if time.monotonic() > deadline:
                raise RuntimeError("controller mode timeout")
            if not teleop.service_is_ready():
                spin(0.1)
                continue
            future = teleop.call_async(SetBool.Request(data=False))
            while not future.done():
                spin(0.05)
                if time.monotonic() > deadline:
                    raise RuntimeError("controller service timeout")
            if future.result().success:
                break
            spin(0.2)
        target = ControllerCommand(mode=ControllerCommand.POSITION)
        target.setpoint_vect.x, target.setpoint_vect.y, target.setpoint_vect.z = map(
            float, reference.translation
        )
        direction = ControllerCommand(mode=ControllerCommand.POSITION)
        q = reference.orientation_wxyz
        (
            direction.setpoint_quat.w,
            direction.setpoint_quat.x,
            direction.setpoint_quat.y,
            direction.setpoint_quat.z,
        ) = map(float, q)
        node.create_timer(0.1, lambda: (linear.publish(target), angular.publish(direction)))
        report.switch_asserting_kill = False
        spin(8)
        call(reset_mapping, Trigger.Request())
        target_mapping(True)
        phase[0] = "locked"
        spin(6)
        locked = [r for r in rows if r["phase"] == "locked" and r["kind"] == "mapping"]
        locked_detections = [r for r in rows if r["phase"] == "locked" and r["kind"] == "detection"]
        target_mapping(False)
        phase[0] = "measure"
        started = time.monotonic()
        while time.monotonic() - started < args.measure:
            spin(0.2)
            measured = [r for r in rows if r["kind"] == "detection" and r["phase"] == "measure"]
            mapping = [r for r in rows if r["kind"] == "mapping" and r["phase"] == "measure"]
            if (
                time.monotonic() - started >= 10
                and len({r["stamp_ns"] for r in measured}) >= 35
                and len(mapping) >= 5
                and all(r["error_m"] < 0.25 for r in mapping[-5:])
            ):
                break
        measured_wall = time.monotonic() - started
        switches = [switch(True), switch(False)]
        phase[0] = "return_ffc"
        spin(5)
        nodes = Counter(node.get_node_names_and_namespaces())
        required = [
            "complete_controller",
            "controller_overseer",
            "ekf_localization_node",
            "yolo_orientation",
            "riptide_mapping2",
        ]
        publishers = {
            topic: len(node.get_publishers_info_by_topic(topic))
            for topic in [
                "/clock",
                "/talos/detected_objects",
                "/talos/mapping/gate_repair",
                *topics,
            ]
        }
        detected = [r for r in rows if r["kind"] == "detection" and r["phase"] == "measure"]
        mapped_rows = [r for r in rows if r["kind"] == "mapping" and r["phase"] == "measure"]
        errors = [r["error_m"] for r in detected]
        final = mapped_rows[-1] if mapped_rows else None
        failures = []
        if any(nodes[(name, "/talos")] != 1 for name in required):
            failures.append("missing/duplicate real nodes")
        if any(count != 1 for count in publishers.values()):
            failures.append("missing/duplicate publishers")
        if len({r["stamp_ns"] for r in detected}) < 35:
            failures.append("fewer than35 distinct measured gate_repair detections")
        if len(locked_detections) < 3 or not locked or any(r["movement_m"] > 0.04 for r in locked):
            failures.append("locked mapper negative control failed")
        if any(
            not r["headers_match"] or r["normal_error_rad"] > math.radians(20) for r in detected
        ):
            failures.append("detection headers or sign-plane orientation invalid")
        if any(
            not np.isfinite([*r["position"], *r["covariance"], *r["orientation_wxyz"]]).all()
            for r in mapped_rows
        ):
            failures.append("mapper produced nonfinite state")
        if any(
            not np.isfinite([*r["camera_position"], r["confidence"], *r["orientation_wxyz"]]).all()
            or r["camera_position"][2] <= 0
            for r in detected
        ):
            failures.append("detector produced invalid state")
        if len(mapped_rows) < 5 or any(
            r["error_m"] > 0.25 or angle(r["orientation_wxyz"], [1, 0, 0, 0]) > math.radians(20)
            for r in mapped_rows[-5:]
        ):
            failures.append("mapping did not sustain correct position/orientation")
        if final is not None and max(final["covariance"][i] for i in (0, 7, 14)) >= 0.05:
            failures.append("mapper position buffer did not warm/converge")
        if errors and float(np.quantile(errors, 0.95)) > 0.30:
            failures.append("detection p95 exceeds 0.30m")
        if any(r["truth_gap_ns"] > 20_000_000 for r in detected):
            failures.append("truth match older than20ms")
        if any(r["frame"] != "talos/ffc_left_camera_optical_frame" for r in detected):
            failures.append("wrong optical frame")
        if (
            final is None
            or final["frame"] != "gate_frame"
            or final["error_m"] > 0.25
            or final["movement_m"] < 0.25
        ):
            failures.append("mapper did not converge away from displaced prior")
        synchronized = {
            camera: len(
                set.intersection(*(stamps[f"{camera}/{kind}"] for kind in ("rgb", "depth", "info")))
            )
            for camera in ("ffc", "dfc")
        }
        ffc_stamps = set.intersection(*(stamps[f"ffc/{kind}"] for kind in ("rgb", "depth", "info")))
        if any(r["stamp_ns"] not in ffc_stamps for r in detected):
            failures.append("detection lacks observed synchronized camera acquisition")
        for camera in ("ffc", "dfc"):
            calibration = fixture_data["calibration"][camera]
            k = calibration["intrinsics_left"]
            info = infos[f"{camera}/info"]
            expected_k = [k["fx"], 0, k["cx"], 0, k["fy"], k["cy"], 0, 0, 1]
            if (
                [info["width"], info["height"]] != calibration["resolution_px"]
                or not np.allclose(info["k"], expected_k)
                or info["frame"] != f"talos/{camera}_left_camera_optical_frame"
            ):
                failures.append(f"{camera} calibration differs from pack")
        returned = [r for r in rows if r["kind"] == "detection" and r["phase"] == "return_ffc"]
        if len(returned) < 3:
            failures.append("FFC detections did not resume after camera switch")
        holds = [r for r in rows if r["kind"] == "hold" and r["phase"] == "measure"]
        hold_metrics = {
            key: float(np.quantile([r[key] for r in holds if r[key] is not None], 0.95))
            for key in ("position_error_m", "attitude_error_rad", "ekf_position_error_m")
        }
        hold_metrics["real_time_factor"] = (
            (holds[-1]["stamp_ns"] - holds[0]["stamp_ns"])
            / 1e9
            / (holds[-1]["wall"] - holds[0]["wall"])
        )
        if (
            hold_metrics["position_error_m"] > 0.20
            or hold_metrics["attitude_error_rad"] > 0.10
            or hold_metrics["ekf_position_error_m"] > 0.15
            or hold_metrics["real_time_factor"] < 0.9
        ):
            failures.append("real controller/EKF hold or real-time factor failed under camera load")
        if "forces" not in latest or max(abs(v) for v in latest["forces"].data) < 0.1:
            failures.append("no active thruster commands")
        if min(synchronized.values()) < 10:
            failures.append("missing synchronized camera groups")
        if not latest["motion"].data:
            failures.append("controller motion not enabled")
        outcome = {
            "passed": not failures,
            "failures": failures,
            "hold": hold_metrics,
            "locked_detections": len(locked_detections),
            "switches": switches,
            "return_ffc_detections": len(returned),
            "unique_detections": len({r["stamp_ns"] for r in detected}),
            "measured_wall_s": measured_wall,
            "detections": len(detected),
            "detection_p95_m": None if not errors else float(np.quantile(errors, 0.95)),
            "final_mapping": final,
            "synchronized_groups": synchronized,
            "publishers": publishers,
            "nodes": [[name, ns, count] for (name, ns), count in nodes.items()],
            "measure_s": args.measure,
        }
        print(json.dumps(outcome), flush=True)
        return not failures
    finally:
        report.switch_asserting_kill = True
        kill.publish(report)
        write(args.output / "perception.json", outcome)
        write(args.output / "camera_info.json", infos)
        write(args.output / "detector_frames.json", arrays)
        write(
            args.output / "image_stamps.json", {key: sorted(value) for key, value in stamps.items()}
        )
        (args.output / "observations.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, default=ROOT / "build/step3-camera-python")
    parser.add_argument("--domain", type=int, default=220)
    parser.add_argument("--measure", type=float, default=60)
    args = parser.parse_args()
    if not 1 <= args.domain <= 232:
        parser.error("choose a private domain in1..232")
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    env = dict(
        os.environ,
        ROS_DOMAIN_ID=str(args.domain),
        ROS_LOCALHOST_ONLY="1",
        RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
    )
    env["PYTHONPATH"] = os.pathsep.join(
        [str(ROOT / "integrations/ros2/python"), str(args.sdk.resolve()), env.get("PYTHONPATH", "")]
    )
    os.environ.update(
        {k: env[k] for k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION")}
    )
    sys.path.insert(0, str(args.sdk.resolve()))
    from robotics_platform import _native

    write(
        args.output / "native_module.json",
        {
            "path": _native.__file__,
            "sha256": hashlib.sha256(Path(_native.__file__).read_bytes()).hexdigest(),
        },
    )
    fixture = prepare(args.output)
    scenario, mapper = fixture[:2]
    commands = [
        [
            sys.executable,
            "-m",
            "robotics_platform_ros",
            str(scenario),
            "--output",
            str(args.output / "bridge"),
        ],
        ["ros2", "launch", str(ROOT / "integrations/uwrt/acceptance/stack.launch.py")],
        [
            "ros2",
            "launch",
            "tensor_detector",
            "tensorrt.launch.py",
            "robot:=talos",
            "use_sim_time:=true",
        ],
        [
            "ros2",
            "run",
            "riptide_mapping2",
            "mapping.py",
            "--ros-args",
            "-r",
            "__ns:=/talos",
            "-r",
            "__node:=riptide_mapping2",
            "--params-file",
            str(mapper),
            "--params-file",
            str(Path(share("riptide_mapping2")) / "config/binary_classifier.yaml"),
            "-p",
            "use_sim_time:=true",
        ],
    ]
    write(args.output / "commands.json", commands)
    processes, logs = [], []
    try:
        for i, command in enumerate(commands):
            log = (args.output / f"process-{i}.log").open("w")
            logs.append(log)
            processes.append(
                subprocess.Popen(
                    command,
                    cwd=ROOT,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
        passed = measure(args, fixture, processes)
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
