"""rclpy transport around BridgeCore: QoS, publishers, subscriptions, services, TF, pacing.

All callbacks run on the stepping thread (``spin_once`` between ticks), so the runtime has a
single owner. Wall time only paces stepping; every stamp comes from simulation time.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robotics_platform import _native as native
from rosgraph_msgs.msg import Clock
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)

from . import mapping
from .core import BridgeCore, Counters, Publication, Transform


def qos(config: dict[str, Any]) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=int(config["depth"]),
        reliability={"reliable": ReliabilityPolicy.RELIABLE,
                     "best_effort": ReliabilityPolicy.BEST_EFFORT}[config["reliability"]],
        durability={"volatile": DurabilityPolicy.VOLATILE,
                    "transient_local": DurabilityPolicy.TRANSIENT_LOCAL}[config["durability"]],
    )


def _stamp(nanoseconds: int) -> Time:
    seconds, rest = divmod(int(nanoseconds), 1_000_000_000)
    return Time(sec=seconds, nanosec=rest)


def _transform_message(item: Transform) -> TransformStamped:
    message = TransformStamped()
    message.header.stamp = _stamp(item.stamp_ns)
    message.header.frame_id, message.child_frame_id = item.parent, item.child
    x, y, z = (float(value) for value in item.translation)
    message.transform.translation.x, message.transform.translation.y = x, y
    message.transform.translation.z = z
    w, qx, qy, qz = (float(value) for value in item.orientation_wxyz)
    rotation = message.transform.rotation
    rotation.w, rotation.x, rotation.y, rotation.z = w, qx, qy, qz
    return message


class BridgeNode(Node):
    """Owns every ROS entity of one bridge pack; never advances the runtime itself."""

    def __init__(self, core_factory: Callable[[Callable[[str, str], Any]], BridgeCore],
                 namespace: str, node_name: str = "robotics_platform_bridge") -> None:
        super().__init__(node_name, namespace=namespace)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.core = core_factory(self.lookup)
        config = self.core.config
        self.declare_parameter(
            "real_time_factor", float(self.core.real_time_factor),
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE, dynamic_typing=True,
                description="Simulation speed relative to wall time; 0 pauses stepping and /clock."))
        self.add_on_set_parameters_callback(self._set_parameters)
        clock = config["clock"]
        self.clock_publisher = self.create_publisher(Clock, clock["topic"], qos(clock["qos"]))
        self.tf_broadcaster = TransformBroadcaster(self) if config.get("tf", {}).get(
            "publish") else None
        self.static_tf_broadcaster = None
        if self.core.static_transforms:
            self.static_tf_broadcaster = StaticTransformBroadcaster(self)
            self.static_tf_broadcaster.sendTransform([
                _transform_message(item) for item in self.core.static_transforms])
        self.publishers_by_stream = {}
        for stream in config["streams"]:
            cls = mapping.message_class(stream["message_type"])
            if stream["direction"] == "publish":
                self.publishers_by_stream[stream["id"]] = self.create_publisher(
                    cls, stream["topic"], qos(stream["qos"]))
            else:
                self.create_subscription(
                    cls, stream["topic"],
                    lambda message, stream=stream["id"]: self.send(
                        self.core.receive(stream, message)),
                    qos(stream["qos"]))
        for service in config.get("services", []):
            cls = mapping.service_class(service["service_type"])
            self.create_service(
                cls, service["service"],
                lambda request, response, service=service["id"]: self.core.call(service, request))
        alignment = config.get("placement", {}).get("estimator_alignment")
        self.alignment_client = None if alignment is None else self.create_client(
            mapping.service_class(alignment["service_type"]), alignment["client"])

    def _set_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "real_time_factor":
                if parameter.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                    return SetParametersResult(successful=False,
                                               reason="real_time_factor must be a number")
                problem = self.core.set_real_time_factor(parameter.value)
                if problem is not None:
                    return SetParametersResult(successful=False, reason=problem)
        return SetParametersResult(successful=True)

    def publish_startup(self) -> None:
        """One-shot latched publications (scenario description)."""
        self.send(self.core.startup_publications())

    def start_cameras(self) -> None:
        if self.core.cameras is not None:
            self.core.cameras.publish = self.send
            self.core.cameras.start()

    def stop_cameras(self) -> None:
        if self.core.cameras is not None:
            self.core.cameras.close()

    def lookup(self, target: str, source: str) -> native.Pose | None:
        try:
            transform = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except TransformException:
            return None
        pose = native.Pose()
        t, r = transform.transform.translation, transform.transform.rotation
        pose.translation = np.array([t.x, t.y, t.z])
        pose.orientation_wxyz = np.array([r.w, r.x, r.y, r.z])
        return pose

    def publish_clock(self, nanoseconds: int) -> None:
        self.clock_publisher.publish(Clock(clock=_stamp(nanoseconds)))

    def send(self, publications: list[Publication]) -> None:
        for item in publications:
            self.publishers_by_stream[item.stream].publish(item.message)

    def broadcast(self, transforms: list[Transform]) -> None:
        if not transforms or self.tf_broadcaster is None:
            return
        messages = [_transform_message(item) for item in transforms]
        self.tf_broadcaster.sendTransform(messages)

    def tick(self) -> None:
        clocks, publications, transforms = self.core.step()
        for stamp in clocks:  # clock strictly before data carrying that time
            self.publish_clock(stamp)
        self.send(publications)
        self.broadcast(transforms)
        if self.core.cameras is not None:
            snapshot = self.core.runtime.observe()
            self.core.cameras.acquire(snapshot, self.core.ros_ns(snapshot.elapsed_ns))
        if self.alignment_client is not None and self.alignment_client.service_is_ready():
            alignment = self.core.pending_alignment()
            if alignment is not None:
                future = self.alignment_client.call_async(alignment.request)
                future.add_done_callback(
                    lambda result, trigger=alignment.trigger: self.alignment_finished(trigger, result))

    def alignment_finished(self, trigger: str, future: Any) -> None:
        try:
            if future.result() is None:
                raise RuntimeError("estimator alignment returned no response")
        except Exception as error:
            Counters.bump(self.core.counters.alignments_failed, trigger)
            self.get_logger().error(f"estimator alignment failed ({trigger}): {error}")
        else:
            Counters.bump(self.core.counters.alignments_acknowledged, trigger)


PAUSED_REFRESH_S = 0.02  # viewer state refresh while paused (the original 50 Hz publish timer)


def run(node: BridgeNode, duration_ns: int | None, max_catchup_ticks: int = 20) -> int:
    """Pace ticks against wall time (scaled by the pack real_time_factor); returns ticks run."""
    core = node.core
    step_s = core.timestep_ns / 1e9
    node.publish_clock(core.clock_ns())
    node.publish_startup()
    ticks, owed = 0, 0.0
    previous = last_refresh = time.monotonic()
    while rclpy.ok():
        for _ in range(32):  # drain ready callbacks on this (the stepping) thread
            rclpy.spin_once(node, timeout_sec=0.0)
        node.send(core.flush())  # operator events, also while paused
        now = time.monotonic()
        if core.real_time_factor <= 0:
            # Paused: no stepping and no /clock; viewers still get fresh state at wall rate.
            owed = 0.0
            if now - last_refresh >= PAUSED_REFRESH_S:
                node.send(core.refresh())
                last_refresh = now
            previous = now
            time.sleep(0.002)
            continue
        owed += (now - previous) * core.real_time_factor
        previous = now
        steps = 0
        while owed >= step_s and steps < max_catchup_ticks:
            node.tick()
            ticks += 1
            owed -= step_s
            steps += 1
            if duration_ns is not None and ticks * core.timestep_ns >= duration_ns:
                return ticks
        if steps == max_catchup_ticks and owed >= step_s:
            node.get_logger().warning(f"falling behind wall time; dropping {owed:.4f} s backlog",
                                      throttle_duration_sec=2.0)
            owed = 0.0
        remaining = (step_s - owed) / max(core.real_time_factor, 1e-9)
        if remaining > 2e-4:
            time.sleep(remaining - 1e-4)
    return ticks
