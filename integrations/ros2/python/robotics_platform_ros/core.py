"""Bridge semantics without middleware: one owner steps the runtime and maps data both ways.

The node (``node.py``) only moves messages; everything here is deterministic in integer
simulation ticks and fully validated against the ROS types and native endpoints before the
first step. Unknown endpoints and unsupported actions fail at construction.
"""

from __future__ import annotations

import fnmatch
import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from robotics_platform import _native as native
from robotics_platform.pack_runtime import PackRuntime

from . import mapping
from .mapping import (
    BOOLEAN,
    MATRIX3,
    QUATERNION_WXYZ,
    SCALAR,
    STRING,
    TIME,
    VECTOR3,
    MappingError,
    Spec,
)

Lookup = Callable[[str, str], "native.Pose | None"]  # (target frame, source frame) -> target_T_source

_IMU = {"specific_force": VECTOR3, "angular_velocity": VECTOR3,
        "force_covariance": MATRIX3, "angular_covariance": MATRIX3}
_ATTITUDE = {"orientation_wxyz": QUATERNION_WXYZ, "covariance": MATRIX3}


def reading_spec(sensor: Mapping[str, Any]) -> dict[str, Any]:
    """Typed native reading fields of one robot sensor (bindings/python/sensors.cpp)."""
    kind = sensor["type"]
    if kind == "imu":
        return dict(_IMU)
    if kind == "attitude":
        return dict(_ATTITUDE)
    if kind == "ahrs":
        return {"inertial": dict(_IMU), "attitude": dict(_ATTITUDE)}
    if kind == "fog":
        axes = len(sensor["parameters"]["axes"])
        return {"angular_rates": Spec("float", (axes,)), "covariance": Spec("float", (axes, axes))}
    if kind == "reference_velocity":
        return {"reference_relative_velocity": VECTOR3, "covariance": MATRIX3}
    if kind == "dvl":
        return {"bottom_relative_velocity": VECTOR3, "covariance": MATRIX3,
                "bottom_distance": SCALAR}
    if kind == "reference_altitude":
        return {"mounted_world_z": SCALAR, "target_world_z": SCALAR, "variance": SCALAR}
    if kind == "pressure":
        return {name: SCALAR for name in
                ("absolute_pressure", "pressure_variance", "depth", "depth_variance")}
    raise MappingError(f"sensor {sensor['id']!r}: no native reading for type {kind!r}")


_ROBOT_STATE = {
    "sim": {"time": TIME},
    "reference_pose": {"position": VECTOR3, "orientation_wxyz": QUATERNION_WXYZ},
    "reference_body_velocity": VECTOR3,
    "body_angular_velocity": VECTOR3,
}
_POSE_ARGUMENTS = {"frame": STRING, "position_m": VECTOR3, "orientation_w": SCALAR,
                   "orientation_x": SCALAR, "orientation_y": SCALAR, "orientation_z": SCALAR}
_COMMAND_ARGUMENTS: dict[str, dict[str, Spec]] = {
    "command:thrusters.set_forces": {"forces_n": Spec("float", (None,))},
    "command:robot.set_killed": {"killed": BOOLEAN},
    "estimate:latest": dict(_POSE_ARGUMENTS),
}
_RESULT = {"accepted": BOOLEAN, "message": STRING}
_ALIGNMENT_TRIGGERS = {"startup", "placement", "reset_to_start", "full_reset"}


class BridgeError(RuntimeError):
    """Bridge configuration cannot run against this runtime and ROS installation."""


def _pose(position: Any, wxyz: Any) -> native.Pose:
    pose = native.Pose()
    pose.translation = np.asarray(position, float)
    pose.orientation_wxyz = np.asarray(wxyz, float)
    return pose


def _rotate(pose: native.Pose, vector: Any) -> np.ndarray:
    return np.asarray(_pose(np.zeros(3), pose.orientation_wxyz).apply(vector), float)


@dataclass
class Publication:
    stream: str
    message: Any


@dataclass
class Transform:
    parent: str
    child: str
    stamp_ns: int
    translation: np.ndarray
    orientation_wxyz: np.ndarray


@dataclass
class Alignment:
    """A request the node sends to the external estimator's pose service."""

    trigger: str
    request: Any


@dataclass
class _Timed:
    period_ns: int
    next_ns: int


@dataclass
class Counters:
    published: dict[str, int] = field(default_factory=dict)
    unavailable_samples: dict[str, int] = field(default_factory=dict)
    rejected_commands: dict[str, int] = field(default_factory=dict)
    filtered_messages: dict[str, int] = field(default_factory=dict)
    service_calls: dict[str, int] = field(default_factory=dict)
    alignments: dict[str, int] = field(default_factory=dict)

    @staticmethod
    def bump(table: dict[str, int], key: str) -> None:
        table[key] = table.get(key, 0) + 1


def _period_ns(rate_hz: float, where: str) -> int:
    period = round(1e9 / rate_hz)
    if period <= 0:
        raise BridgeError(f"{where}: rate_hz {rate_hz} is too high")
    return period


class BridgeCore:
    """Validated bridge over one PackRuntime; the only object that advances it."""

    def __init__(self, resolved: Any, pack: PackRuntime, *, epoch_ns: int,
                 lookup: Lookup | None = None) -> None:
        if resolved.bridge is None:
            raise BridgeError("scenario selects no bridge pack")
        self.config = resolved.bridge
        self.robot = resolved.robot
        self.pack = pack
        self.runtime = pack.runtime
        self.frames = pack.frames
        self.lookup = lookup
        self.counters = Counters()
        self.timestep_ns = int(pack.parameters.timestep_ns)
        clock = self.config["clock"]
        self.real_time_factor = float(clock["real_time_factor"])
        self.epoch_ns = epoch_ns if clock["epoch"] == "system_time_at_start" else 0
        self.reset_policy = clock["reset_policy"]
        self._clock = _Timed(self._stepped_period(clock["rate_hz"], "clock"), 0)
        names = self.config.get("frame_names", {})
        self.world_frame = names.get("world", resolved.scenario["world_frame"])
        self.reference_frame = self.robot["reference_frame"]
        self._root_to_reference = self.frames.from_root(self.reference_frame)
        snapshot = self.runtime.observe()
        self._generation = snapshot.generation
        self._offset_ns = 0
        self._last_ros_ns = self.ros_ns(snapshot.elapsed_ns)
        self.start_state = pack.initial

        safety = self.robot["safety"]
        self.killed = bool(safety["initially_killed"])
        self.kill_stops_thrusters = bool(safety["kill_stops_thrusters"])
        self.commands_while_killed = safety["commands_while_killed"]
        self._thruster_index = self._thruster_permutation()

        self.publishers: dict[str, mapping.Writer] = {}
        self.readers: dict[str, mapping.Reader] = {}
        self.stream_config = {item["id"]: item for item in self.config["streams"]}
        self._sensor_streams: dict[str, list[str]] = {}
        self._timed: dict[str, _Timed] = {}
        self._events: dict[str, list[str]] = {}
        self._compile_streams()
        self.services: dict[str, tuple[mapping.Reader, mapping.Writer, dict[str, Any]]] = {}
        self._compile_services()
        self._tf = self._compile_tf()
        self._compile_alignment()
        self._check_bindings()
        self.latest_estimate: dict[str, Any] | None = None

    # ------------------------------------------------------------ construction

    def _stepped_period(self, rate_hz: float, where: str) -> int:
        period = _period_ns(rate_hz, where)
        if period < self.timestep_ns:
            raise BridgeError(f"{where}: rate_hz {rate_hz} exceeds the physics step rate")
        return period

    def _thruster_permutation(self) -> list[int] | None:
        block = self.config.get("thrusters")
        native_ids = [item["id"] for item in self.robot["thrusters"]]
        if block is None:
            return None
        if sorted(block["order"]) != sorted(native_ids):
            raise BridgeError("thrusters.order must be a permutation of the robot thrusters")
        if len(block["input_scales"]) != len(block["order"]):
            raise BridgeError("thrusters.input_scales must match thrusters.order")
        if set(block["reject"]) != {"wrong_length", "nonfinite"}:
            raise BridgeError("thrusters.reject must list wrong_length and nonfinite: the "
                              "native plant accepts neither")
        return [native_ids.index(name) for name in block["order"]]

    def _sensor(self, name: str, where: str) -> Mapping[str, Any]:
        sensors = {item["id"]: item for item in self.robot["sensors"]}
        if name not in sensors:
            raise BridgeError(f"{where}: unknown robot sensor {name!r}")
        if name not in self.pack.streams:
            raise BridgeError(f"{where}: robot sensor {name!r} is not selected for execution")
        return sensors[name]

    def _compile_streams(self) -> None:
        for stream in self.config["streams"]:
            where = f"streams/{stream['id']}"
            endpoint = stream["native"]
            try:
                cls = mapping.message_class(stream["message_type"])
                if stream["direction"] == "subscribe":
                    if endpoint not in _COMMAND_ARGUMENTS:
                        raise BridgeError(f"{where}: unsupported native endpoint {endpoint!r}")
                    if endpoint == "command:thrusters.set_forces" and self._thruster_index is None:
                        raise BridgeError(f"{where}: thruster commands need a thrusters block")
                    self.readers[stream["id"]] = mapping.compile_reader(
                        cls, stream["fields"], _COMMAND_ARGUMENTS[endpoint],
                        stream.get("accept_if"), where=where)
                    continue
                if "image" in stream:
                    raise BridgeError(f"{where}: image streams are not executed by this bridge")
                if endpoint.startswith("sensor:"):
                    name, _, output = endpoint.removeprefix("sensor:").partition(".")
                    sensor = self._sensor(name, where)
                    if output:
                        raise BridgeError(f"{where}: sensor output {output!r} is not executed")
                    expected = 1e9 / sensor["period_ns"]
                    if not math.isclose(stream["rate_hz"], expected, rel_tol=1e-6):
                        raise BridgeError(f"{where}: rate_hz {stream['rate_hz']} differs from "
                                          f"sensor {name!r} rate {expected:.9g} Hz")
                    sources: dict[str, Any] = {"sample": {"time": TIME},
                                               "reading": reading_spec(sensor)}
                    self._sensor_streams.setdefault(name, []).append(stream["id"])
                elif endpoint in ("timer", "state:robot"):
                    sources = dict(_ROBOT_STATE) if endpoint == "state:robot" else {
                        "sim": {"time": TIME}}
                    period = self._stepped_period(stream["rate_hz"], where)
                    self._timed[stream["id"]] = _Timed(period, period)
                elif endpoint == "event:robot.kill_changed":
                    sources = {"sim": {"time": TIME}, "killed": BOOLEAN}
                    self._events.setdefault(endpoint, []).append(stream["id"])
                else:
                    raise BridgeError(f"{where}: unsupported native endpoint {endpoint!r}")
                self.publishers[stream["id"]] = mapping.compile_writer(
                    cls, stream["fields"], sources, frame_id=stream["frame_id"], where=where)
            except MappingError as error:
                raise BridgeError(str(error)) from None
        for stream in self.config["streams"]:
            if "reply_stream" in stream:
                raise BridgeError(f"streams/{stream['id']}: reply streams are not executed")

    def _compile_services(self) -> None:
        for service in self.config.get("services", []):
            where = f"services/{service['id']}"
            action = service["action"]
            options = dict(service.get("placement", {}))
            if action == "command:robot.place":
                if not options:
                    raise BridgeError(f"{where}: robot.place needs placement options")
                arguments = _POSE_ARGUMENTS if options["pose_source"] == "request" else {}
            elif action == "command:robot.reset_to_start":
                arguments = {}
            else:
                raise BridgeError(f"{where}: unsupported native action {action!r}")
            if not hasattr(self.runtime, "place"):
                raise BridgeError(f"{where}: native Runtime.place is not available")
            if options.get("pose_source") == "estimate" and not self._has_estimate_stream():
                raise BridgeError(f"{where}: pose_source estimate needs an estimate:latest stream")
            try:
                cls = mapping.service_class(service["service_type"])
                reader = mapping.compile_reader(cls.Request, service["request"], arguments,
                                                where=f"{where}/request")
                writer = mapping.compile_writer(cls.Response, service["response"], _RESULT,
                                                where=f"{where}/response")
            except MappingError as error:
                raise BridgeError(str(error)) from None
            self.services[service["id"]] = (reader, writer, {"action": action, **options})

    def _has_estimate_stream(self) -> bool:
        return any(item["native"] == "estimate:latest" for item in self.config["streams"])

    def _compile_tf(self) -> list[tuple[dict[str, Any], _Timed]]:
        entries = []
        for entry in self.config.get("tf", {}).get("publish", []):
            if entry["native"] != "state:robot.reference_pose":
                raise BridgeError(f"tf: unsupported native endpoint {entry['native']!r}")
            if entry["parent"] != self.world_frame:
                raise BridgeError(f"tf: truth transforms must have parent {self.world_frame!r}")
            period = self._stepped_period(entry["rate_hz"], "tf")
            entries.append((entry, _Timed(period, period)))
        return entries

    def _compile_alignment(self) -> None:
        self.alignment = self.config.get("placement", {}).get("estimator_alignment")
        self.alignment_pending: str | None = None
        if self.alignment is None:
            return
        if not set(self.alignment["triggers"]) <= _ALIGNMENT_TRIGGERS:
            raise BridgeError("estimator_alignment: unknown trigger")
        stream = self.stream_config.get(self.alignment["estimate_stream"])
        if stream is None or stream["native"] != "estimate:latest":
            raise BridgeError("estimator_alignment: estimate_stream must be an estimate:latest "
                              "subscription")
        try:
            request = mapping.service_class(self.alignment["service_type"]).Request
            if mapping.ros_field(request, "pose").base != "geometry_msgs/PoseWithCovarianceStamped":
                raise BridgeError("estimator_alignment: service request must be "
                                  "pose: geometry_msgs/PoseWithCovarianceStamped")
        except MappingError as error:
            raise BridgeError(str(error)) from None
        self._alignment_request = request
        if "startup" in self.alignment["triggers"]:
            self.alignment_pending = "startup"

    def _check_bindings(self) -> None:
        """Every declared block must match what executes; nothing is silently ignored."""
        converters = [*self.config.get("converters", []),
                      *(item for item in [*self.config["streams"],
                                          *self.config.get("services", [])]
                        if "converter" in item)]
        if converters:
            raise BridgeError("compiled converters are declared but none is implemented here")
        streams, services = self.stream_config, {
            item["id"]: item for item in self.config.get("services", [])}

        def require(kind: str, name: str, check: bool, expected: str) -> None:
            if not check:
                raise BridgeError(f"{kind} {name!r} must be {expected}")

        kill = self.config.get("kill")
        if kill is not None:
            command, state = streams[kill["command_stream"]], streams[kill["state_stream"]]
            require("kill.command_stream", command["id"],
                    command["native"] == "command:robot.set_killed", "command:robot.set_killed")
            require("kill.state_stream", state["id"],
                    state["native"] == "event:robot.kill_changed", "event:robot.kill_changed")
        reset = self.config.get("reset", {})
        for key in reset:
            require(f"reset.{key}", reset[key], key == "robot_service"
                    and services[reset[key]]["action"] == "command:robot.reset_to_start",
                    "the robot service with action command:robot.reset_to_start (task and full "
                    "resets are not executed by this bridge)")
        placement = self.config.get("placement", {})
        for key, source in (("set_service", "request"), ("sync_service", "estimate")):
            if key in placement:
                service = services[placement[key]]
                require(f"placement.{key}", service["id"],
                        service["action"] == "command:robot.place"
                        and service.get("placement", {}).get("pose_source") == source,
                        f"a command:robot.place service with pose_source {source}")
        sensors = {item["id"]: item for item in self.robot["sensors"]}
        names = self.config.get("frame_names", {})
        for stream in self.config["streams"]:
            name = stream["native"].removeprefix("sensor:").split(".")[0]
            if (stream["native"].startswith("sensor:") and stream["frame_id"]
                    and sensors[name]["frame"] in names):
                expected = names[sensors[name]["frame"]]
                require(f"streams/{stream['id']}.frame_id", stream["frame_id"],
                        stream["frame_id"] == expected,
                        f"{expected!r} (frame_names of sensor frame {sensors[name]['frame']!r})")
        forbidden = self.config.get("tf", {}).get("never_publish", [])
        for entry, _ in self._tf:
            for pattern in forbidden:
                if fnmatch.fnmatchcase(entry["child"], pattern):
                    raise BridgeError(f"tf: {entry['child']!r} is listed in never_publish")

    # ------------------------------------------------------------ time

    def ros_ns(self, native_ns: int) -> int:
        return self.epoch_ns + self._offset_ns + int(native_ns)

    def _observe_generation(self, snapshot: Any) -> None:
        if snapshot.generation == self._generation:
            return
        self._generation = snapshot.generation
        if self.reset_policy == "preserve_ros_epoch_and_time":
            self._offset_ns = self._last_ros_ns - self.epoch_ns + self.timestep_ns
        self._clock.next_ns = 0
        for timed in [*self._timed.values(), *(item for _, item in self._tf)]:
            timed.next_ns = timed.period_ns

    # ------------------------------------------------------------ stepping

    def clock_ns(self) -> int:
        """Current ROS time; the node publishes it before any data at startup."""
        return self._last_ros_ns

    def step(self) -> tuple[list[int], list[Publication], list[Transform]]:
        """Advance exactly one tick; return clock stamps, publications and transforms in the
        order they must be sent (clock first)."""
        snapshot = self.runtime.advance(1)
        self._observe_generation(snapshot)
        now = int(snapshot.elapsed_ns)
        self._last_ros_ns = self.ros_ns(now)
        clocks = []
        if now >= self._clock.next_ns:
            clocks.append(self._last_ros_ns)
            self._clock.next_ns = now - now % self._clock.period_ns + self._clock.period_ns
        publications: list[Publication] = []
        for sensor, native_stream in self.pack.streams.items():
            streams = self._sensor_streams.get(sensor, [])  # unbridged sensors still drain
            for sample in native_stream.drain():
                if sample.value is None:
                    Counters.bump(self.counters.unavailable_samples, sensor)
                    continue
                values = {"sample": {"time": self.ros_ns(sample.header.acquired_ns)},
                          "reading": sample.value}
                publications += [self._publish(stream, values) for stream in streams]
        state = None
        for stream, timed in self._timed.items():
            if now >= timed.next_ns:
                state = state or self._robot_state(snapshot)
                publications.append(self._publish(stream, state))
                timed.next_ns += timed.period_ns
        transforms = []
        for entry, timed in self._tf:
            if now >= timed.next_ns:
                state = state or self._robot_state(snapshot)
                pose = state["reference_pose"]
                transforms.append(Transform(entry["parent"], entry["child"], self._last_ros_ns,
                                            pose["position"], pose["orientation_wxyz"]))
                timed.next_ns += timed.period_ns
        return clocks, publications, transforms

    def _publish(self, stream: str, values: Any) -> Publication:
        Counters.bump(self.counters.published, stream)
        return Publication(stream, self.publishers[stream](values))

    def reference_pose(self, body: Any) -> native.Pose:
        return _pose(body.position, body.orientation_wxyz).compose(self._root_to_reference)

    def _robot_state(self, snapshot: Any) -> dict[str, Any]:
        body = snapshot.body
        world_reference = self.reference_pose(body)
        offset = self._root_to_reference.translation
        omega = np.asarray(body.angular_velocity, float)
        velocity = np.asarray(body.linear_velocity, float) + np.cross(omega, offset)
        to_reference = self._root_to_reference.inverse()
        return {
            "sim": {"time": self._last_ros_ns},
            "reference_pose": {"position": np.asarray(world_reference.translation, float),
                               "orientation_wxyz": np.asarray(
                                   world_reference.orientation_wxyz, float)},
            "reference_body_velocity": _rotate(to_reference, velocity),
            "body_angular_velocity": _rotate(to_reference, omega),
        }

    # ------------------------------------------------------------ inbound

    def receive(self, stream: str, message: Any) -> list[Publication]:
        """Apply one inbound message; returns event publications it causes."""
        reader = self.readers[stream]
        try:
            if not reader.accepts(message):
                Counters.bump(self.counters.filtered_messages, stream)
                return []
            arguments = reader(message)
        except MappingError:
            Counters.bump(self.counters.rejected_commands, f"{stream}:malformed")
            return []
        endpoint = self.stream_config[stream]["native"]
        if endpoint == "command:thrusters.set_forces":
            self._command(stream, arguments["forces_n"])
            return []
        if endpoint == "command:robot.set_killed":
            return self.set_killed(arguments["killed"])
        self.latest_estimate = arguments
        return []

    def _command(self, stream: str, forces: np.ndarray) -> None:
        assert self._thruster_index is not None
        block = self.config["thrusters"]
        if forces.size != len(self._thruster_index):
            Counters.bump(self.counters.rejected_commands, f"{stream}:wrong_length")
            return
        if not np.all(np.isfinite(forces)):
            Counters.bump(self.counters.rejected_commands, f"{stream}:nonfinite")
            return
        if self.killed:
            if self.commands_while_killed == "rejected":
                Counters.bump(self.counters.rejected_commands, f"{stream}:killed")
                return
            forces = np.zeros_like(forces)
        native_forces = np.zeros(len(self._thruster_index))
        with np.errstate(over="ignore", invalid="ignore"):
            for position, index in enumerate(self._thruster_index):
                native_forces[index] = forces[position] * block["input_scales"][position]
        if not np.all(np.isfinite(native_forces)):  # finite input can overflow when scaled
            Counters.bump(self.counters.rejected_commands, f"{stream}:nonfinite_scaled")
            return
        self.runtime.command(native_forces)

    def set_killed(self, killed: bool) -> list[Publication]:
        self.killed = bool(killed)
        if self.killed and self.kill_stops_thrusters:
            self.runtime.stop_thrusters()
        values = {"sim": {"time": self._last_ros_ns}, "killed": self.killed}
        return [self._publish(stream, values)
                for stream in self._events.get("event:robot.kill_changed", [])]

    # ------------------------------------------------------------ services

    def call(self, service: str, request: Any) -> Any:
        reader, writer, options = self.services[service]
        Counters.bump(self.counters.service_calls, service)
        try:
            arguments = reader(request)
        except MappingError as error:
            Counters.bump(self.counters.rejected_commands, f"{service}:malformed")
            return writer({"accepted": False, "message": str(error)})
        if options["action"] == "command:robot.reset_to_start":
            accepted, message = self._place_state(self.start_state, keep_velocity=False,
                                                  becomes_start=False)
            trigger = "reset_to_start"
        elif options["pose_source"] == "estimate":
            if self.latest_estimate is None:
                accepted, message = False, "no estimate received yet"
            else:
                accepted, message = self.place_reference(self.latest_estimate, options)
            trigger = "placement"
        else:
            accepted, message = self.place_reference(arguments, options)
            trigger = "placement"
        if accepted and (options["action"] == "command:robot.reset_to_start"
                         or options.get("align_estimator", False)):
            self._request_alignment(trigger)
        return writer({"accepted": accepted, "message": message})

    def place_reference(self, pose: Mapping[str, Any], options: Mapping[str, Any]
                        ) -> tuple[bool, str]:
        """Place the robot reference frame at a pose given in any TF-resolvable frame."""
        wxyz = np.array([pose["orientation_w"], pose["orientation_x"], pose["orientation_y"],
                         pose["orientation_z"]], float)
        position = np.asarray(pose["position_m"], float)
        norm = float(np.linalg.norm(wxyz))
        if not (np.all(np.isfinite(position)) and math.isfinite(norm) and norm > 1e-10):
            return False, "pose must be finite with a nonzero quaternion"
        frame = pose["frame"] or self.world_frame
        world_to_frame = self._transform(self.world_frame, frame)
        if world_to_frame is None:
            return False, f"no transform from {frame!r} to {self.world_frame!r}"
        world_reference = world_to_frame.compose(_pose(position, wxyz / norm))
        com = world_reference.compose(self._root_to_reference.inverse())
        state = native.BodyState()
        state.position, state.orientation_wxyz = com.translation, com.orientation_wxyz
        if options["keep_velocity"]:
            body = self.runtime.observe().body
            state.linear_velocity, state.angular_velocity = (
                body.linear_velocity, body.angular_velocity)
        return self._place_state(state, keep_velocity=options["keep_velocity"],
                                 becomes_start=options["becomes_start_pose"])

    def _transform(self, target: str, source: str) -> native.Pose | None:
        if target == source:
            return _pose(np.zeros(3), [1.0, 0.0, 0.0, 0.0])
        return self.lookup(target, source) if self.lookup is not None else None

    def _place_state(self, state: Any, *, keep_velocity: bool, becomes_start: bool
                     ) -> tuple[bool, str]:
        target = native.BodyState()
        target.position, target.orientation_wxyz = state.position, state.orientation_wxyz
        if keep_velocity:
            target.linear_velocity, target.angular_velocity = (
                state.linear_velocity, state.angular_velocity)
        try:
            snapshot = self.runtime.place(target, clear_actuators=not keep_velocity)
        except ValueError as error:
            return False, str(error)
        if becomes_start:
            self.start_state = target
        reference = self.reference_pose(snapshot.body)
        x, y, z = (float(value) for value in reference.translation)
        return True, f"placed {self.reference_frame} at ({x:.3f}, {y:.3f}, {z:.3f}) m"

    # ------------------------------------------------------------ estimator alignment

    def _request_alignment(self, trigger: str) -> None:
        if self.alignment is not None and trigger in self.alignment["triggers"]:
            self.alignment_pending = trigger

    def pending_alignment(self) -> Alignment | None:
        """The seeding request due now, once the estimate frame and its transform exist."""
        if self.alignment_pending is None or self.latest_estimate is None:
            return None
        frame = self.latest_estimate["frame"]
        estimate_to_world = self._transform(frame, self.world_frame)
        if estimate_to_world is None:
            return None
        pose = estimate_to_world.compose(self.reference_pose(self.runtime.observe().body))
        request = self._alignment_request()
        stamped = request.pose
        stamped.header.frame_id = frame
        stamp = self._last_ros_ns
        stamped.header.stamp.sec, stamped.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        target = stamped.pose.pose
        target.position.x, target.position.y, target.position.z = (
            float(value) for value in pose.translation)
        w, x, y, z = (float(value) for value in pose.orientation_wxyz)
        target.orientation.w, target.orientation.x = w, x
        target.orientation.y, target.orientation.z = y, z
        covariance = np.zeros(36)
        covariance[::7] = self.alignment["covariance_diagonal"]
        stamped.pose.covariance = covariance
        trigger, self.alignment_pending = self.alignment_pending, None
        Counters.bump(self.counters.alignments, trigger)
        return Alignment(trigger, request)
