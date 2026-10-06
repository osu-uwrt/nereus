"""Measure the unchanged UWRT controllers/EKF against a separately launched simulator."""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from riptide_msgs2.msg import ControllerCommand, KillSwitchReport
from robot_localization.srv import SetPose
from std_msgs.msg import Bool, Float32MultiArray
from std_srvs.srv import SetBool


def yaw(q):
    """Heading (rad) of a geometry_msgs quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrapped(angle):
    """The angle wrapped to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    """Holds two depth/heading setpoints and writes truth vs EKF tracking metrics to --output."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--settle", type=float, default=25.0)
    parser.add_argument("--measure", type=float, default=20.0)
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID", "0") == "0":
        parser.error("use a private ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
    args.output.mkdir(parents=True, exist_ok=False)

    # Node state: the latest message per kind, and truth/estimate samples while a phase records.
    rclpy.init()
    node = Node("platform_hold_acceptance")
    latest, rows, alignment_rows = {}, [], []
    recording = [None]

    def receive(kind, msg):
        latest[kind] = (time.monotonic(), msg)
        if kind in ("truth", "estimate") and recording[0] is not None:
            p = msg.pose.pose
            rows.append(
                {
                    "phase": recording[0],
                    "kind": kind,
                    "wall": time.monotonic(),
                    "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    "xyz": [p.position.x, p.position.y, p.position.z],
                    "yaw": yaw(p.orientation),
                }
            )

    # Subscriptions, controller command publishers and the services used below.
    for kind, topic, typ in (
        ("truth", "simulator/ground_truth", Odometry),
        ("estimate", "odometry/filtered", Odometry),
        ("forces", "thruster_forces", Float32MultiArray),
        ("motion", "controller/motion_enabled", Bool),
    ):
        node.create_subscription(typ, "/talos/" + topic, lambda msg, k=kind: receive(k, msg), 100)
    linear = node.create_publisher(ControllerCommand, "/talos/controller/linear", 10)
    angular = node.create_publisher(ControllerCommand, "/talos/controller/angular", 10)
    kill = node.create_publisher(KillSwitchReport, "/talos/command/software_kill", 10)
    placement = node.create_client(SetPose, "/talos/set_sim_pose")
    teleop = node.create_client(SetBool, "/talos/setTeleop")
    report = KillSwitchReport(
        kill_switch_id=1, sender_id="platform_hold_acceptance", switch_asserting_kill=True
    )

    # Spin the node for `seconds` of wall time.
    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, until - time.monotonic())))

    # Synchronous service call with 10 s timeouts for discovery and the reply.
    def call(client, request):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("missing service " + client.srv_name)
        future = client.call_async(request)
        until = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not future.done():
            raise RuntimeError("service timed out " + client.srv_name)
        return future.result()

    # Wait (up to 20 s) until the EKF estimate matches truth (10 cm, 0.05 rad) for 0.5 s with fresh
    # messages; logs every check to alignment_rows.
    def wait_for_alignment(phase):
        started, stable_since = time.monotonic(), None
        errors = {}
        while time.monotonic() - started < 20.0:
            spin(0.05)
            now = time.monotonic()
            truth_time, truth = latest["truth"]
            estimate_time, estimate = latest["estimate"]
            a, b = truth.pose.pose, estimate.pose.pose
            position_error = math.sqrt(
                sum((getattr(a.position, axis) - getattr(b.position, axis)) ** 2 for axis in "xyz")
            )
            heading_error = abs(wrapped(yaw(a.orientation) - yaw(b.orientation)))
            errors = {
                "position_error_m": position_error,
                "heading_error_rad": heading_error,
                "wait_s": now - started,
            }
            alignment_rows.append(
                {
                    **errors,
                    "phase": phase,
                    "difference_m": [
                        getattr(a.position, axis) - getattr(b.position, axis) for axis in "xyz"
                    ],
                }
            )
            if (
                # Original EKF seeds correctly, then drifts ~4.5 cm in XY while
                # killed. This checks the 8 m pose jump, not horizontal holding.
                position_error < 0.10
                and heading_error < 0.05
                and (now - truth_time < 0.15 and now - estimate_time < 0.2)
            ):
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= 0.5:
                    return errors
            else:
                stable_since = None
        raise RuntimeError(f"EKF did not align with physical truth: {errors}")

    try:
        # Wait for exactly one of each stack node and topic publisher (no stale duplicates from an
        # earlier run on the domain).
        deadline = time.monotonic() + 25.0
        required = ("complete_controller", "controller_overseer", "ekf_localization_node")
        topics = (
            "/clock",
            "/talos/simulator/ground_truth",
            "/talos/odometry/filtered",
            "/talos/thruster_forces",
        )
        while True:
            spin(0.5)
            nodes = Counter(node.get_node_names_and_namespaces())
            publishers = {topic: len(node.get_publishers_info_by_topic(topic)) for topic in topics}
            if (
                all(nodes[(name, "/talos")] == 1 for name in required)
                and all(count == 1 for count in publishers.values())
                and "truth" in latest
                and "estimate" in latest
            ):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f"stack discovery timed out: {nodes}, {publishers}")
        for name in ("complete_controller", "controller_overseer", "ekf_localization_node"):
            if nodes[(name, "/talos")] != 1:
                raise RuntimeError(f"expected exactly one real {name}: {nodes}")
        for topic in (
            "/clock",
            "/talos/simulator/ground_truth",
            "/talos/odometry/filtered",
            "/talos/thruster_forces",
        ):
            if len(node.get_publishers_info_by_topic(topic)) != 1:
                raise RuntimeError("expected one publisher: " + topic)

        # Keep the software kill asserted (republished at 4 Hz) while the EKF settles and the vehicle
        # is teleported to (8, 8, -1.15).
        node.create_timer(0.25, lambda: kill.publish(report))
        kill.publish(report)
        startup_alignment = wait_for_alignment("startup")
        request = SetPose.Request()
        request.pose.header.frame_id = "map"
        request.pose.pose.pose.position.x = 8.0
        request.pose.pose.pose.position.y = 8.0
        request.pose.pose.pose.position.z = -1.15
        request.pose.pose.pose.orientation.w = 1.0
        call(placement, request)
        placement_alignment = wait_for_alignment("placement")

        # Leave teleop mode (retrying for up to 20 s), then release the kill.
        deadline = time.monotonic() + 20.0
        while True:
            response = call(teleop, SetBool.Request(data=False))
            if response.success:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(response.message)
            spin(0.5)
        spin(3.0)
        report.switch_asserting_kill = False

        # Each phase: command a depth and heading at 10 Hz, settle, then measure the hold error.
        phases = []
        for phase, (depth, heading) in enumerate(((-1.4, 0.4), (-1.0, -0.35))):
            target = ControllerCommand(mode=ControllerCommand.POSITION)
            target.setpoint_vect.x, target.setpoint_vect.y, target.setpoint_vect.z = 8.0, 8.0, depth
            direction = ControllerCommand(mode=ControllerCommand.POSITION)
            direction.setpoint_quat.z = math.sin(heading / 2)
            direction.setpoint_quat.w = math.cos(heading / 2)

            def send():
                linear.publish(target)
                angular.publish(direction)

            timer = node.create_timer(0.1, send)
            send()
            recording[0] = phase
            start = time.monotonic()
            spin(args.settle + args.measure)
            node.destroy_timer(timer)
            recording[0] = None

            # Metrics over the samples after the settle period, for truth and the EKF separately.
            cutoff = start + args.settle
            metrics = {}
            for kind in ("truth", "estimate"):
                selected = [
                    r
                    for r in rows
                    if r["phase"] == phase and r["kind"] == kind and r["wall"] >= cutoff
                ]
                if len(selected) < args.measure * 10:
                    raise RuntimeError(f"insufficient {kind} samples: {len(selected)}")
                z = np.array([r["xyz"][2] - depth for r in selected])
                angle = np.array([wrapped(r["yaw"] - heading) for r in selected])
                metrics[kind] = {
                    "samples": len(selected),
                    "depth_rms_m": float(np.sqrt(np.mean(z * z))),
                    "heading_rms_rad": float(np.sqrt(np.mean(angle * angle))),
                    "depth_max_m": float(np.max(np.abs(z))),
                    "heading_max_rad": float(np.max(np.abs(angle))),
                    "real_time_factor": (selected[-1]["stamp"] - selected[0]["stamp"])
                    / (selected[-1]["wall"] - selected[0]["wall"]),
                }
            freshness = {k: time.monotonic() - v[0] for k, v in latest.items()}
            phases.append(
                {
                    "target": {"z": depth, "heading": heading},
                    "metrics": metrics,
                    "latest_age_s": freshness,
                    "motion_enabled": latest["motion"][1].data,
                    "last_forces": list(latest["forces"][1].data),
                }
            )
            print(json.dumps(phases[-1]), flush=True)
        result = {
            "startup_alignment": startup_alignment,
            "placement_alignment": placement_alignment,
            "phases": phases,
            "nodes": [list(k) for k in nodes],
            "settle_s": args.settle,
            "measure_s": args.measure,
            "environment": {
                k: os.environ.get(k)
                for k in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY")
            },
        }
        (args.output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    finally:
        # Always re-kill the vehicle and keep the raw samples, even when a check failed.
        report.switch_asserting_kill = True
        kill.publish(report)
        spin(0.1)
        (args.output / "samples.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
        (args.output / "alignment.jsonl").write_text(
            "".join(json.dumps(row) + "\n" for row in alignment_rows)
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
