"""Simulation-time mechanism commands; no transport, projectile or grasp-physics ownership.

The caller supplies authoritative kill state on every operation and must deliver kill
changes through advance(0, killed=...). Calls are serialized by that same owner. Released
bodies belong to the caller: reload and reset never remove or mutate them.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

import numpy as np
from numpy.typing import NDArray

from . import _native as native


@dataclass(frozen=True)
class PayloadRelease:
    mechanism_id: str
    slot_id: str
    slot_index: int
    time_ns: int
    position_world_m: tuple[float, ...]  # visible mesh center, NOT center of mass
    orientation_wxyz: tuple[float, ...]
    velocity_com_world_m_s: tuple[float, ...]
    angular_velocity_world_rad_s: tuple[float, ...]
    projectile: Mapping[str, Any]


@dataclass(frozen=True)
class CommandResult:
    accepted: bool
    message: str
    release: PayloadRelease | None = None


@dataclass(frozen=True)
class ReleaseState:
    state: str
    available: int


@dataclass(frozen=True)
class ClawState:
    state: str
    gap_m: float
    target_gap_m: float
    joint_positions_m: tuple[float, float]
    grasp_supported: bool = False


@dataclass(frozen=True)
class MechanismState:
    time_ns: int
    armed: bool
    any_busy: bool
    releases: Mapping[str, ReleaseState]
    claws: Mapping[str, ClawState]


@dataclass
class _Claw:
    minimum: float
    maximum: float
    speed: float
    tolerance: float
    initial: float
    q: float
    target: float
    direction: int = 0
    remaining_ns: int | None = None

    @property
    def travel(self) -> float:
        return (self.maximum - self.minimum) / 2

    def stop(self) -> None:
        self.target, self.direction, self.remaining_ns = self.q, 0, None


def _number(value: Any, name: str, *, minimum: float = 0) -> float:
    if isinstance(value, (bool, str, bytes)):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (ValueError, TypeError, OverflowError):
        raise ValueError(f"{name} must be a finite number") from None
    if not math.isfinite(result) or result < minimum:
        raise ValueError(f"{name} must be finite and >= {minimum}")
    return result


def _boolean(value: Any, name: str) -> bool:
    if type(value) is not bool:
        raise ValueError(f"{name} must be a bool")
    return value


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({k: _freeze(v) for k, v in value.items()})
    if isinstance(value, (tuple, list)):
        return tuple(_freeze(v) for v in value)
    return value


def _pose(position: Sequence[float], quaternion: Sequence[float]) -> native.Pose:
    result = native.Pose()
    result.translation, result.orientation_wxyz = position, quaternion
    return native.Pose().compose(result)


def _vector(value: Sequence[float], name: str) -> NDArray[np.float64]:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain three finite values")
    return result


def _nanoseconds(seconds: float) -> int:
    value = seconds * 1e9
    if not math.isfinite(value) or value > 2**63 - 1:
        raise ValueError("duration exceeds signed 64-bit nanoseconds")
    return round(value)


class Mechanisms:
    """Compile validated robot data into sparse actuator state.

    Basic claw travel is supported; physical grasping, contact loads and movable objects
    are deliberately not implemented here. Passive magnets have no command state. There
    is no hardcoded robot, mechanism ID, task year or numeric transport status.
    """

    def __init__(self, robot: Mapping[str, Any]):
        config = robot["frames"]
        edges = []
        for entry in config["transforms"]:
            edge = native.FixedFrame()
            edge.parent, edge.child = entry["parent"], entry["child"]
            edge.pose = _pose(entry["position_m"], entry["orientation_wxyz"])
            edges.append(edge)
        self._frames = native.FixedFrames(config["root"], edges)
        self._release_configs: dict[str, Mapping[str, Any]] = {}
        self._mounts: dict[str, tuple[native.Pose, ...]] = {}
        self._cooldown_ns: dict[str, int] = {}
        self._claws: dict[str, _Claw] = {}
        safety = robot["safety"]
        arming = safety["arming"]
        self._initial_armed = _boolean(arming["initially_armed"], "initially_armed")
        self._kill_disarms = _boolean(safety["kill_disarms_mechanisms"], "kill_disarms_mechanisms")
        self._reject_arm_killed = _boolean(arming["arm_rejected_while_killed"],
                                          "arm_rejected_while_killed")
        self._guarded = frozenset(arming["applies_to"])
        ids = set()
        for entry in robot.get("mechanisms", []):
            identifier, kind, p = entry["id"], entry["type"], entry["parameters"]
            if identifier in ids:
                raise ValueError(f"duplicate mechanism {identifier!r}")
            ids.add(identifier)
            mount = self._frames.from_root(entry["frame"])
            if kind in ("launcher", "dropper"):
                slots = p["slots"]
                if (type(p["capacity"]) is not int or p["capacity"] < 1 or
                        p["capacity"] != len(slots) or
                        len({s["id"] for s in slots}) != len(slots)):
                    raise ValueError("capacity must match nonempty unique slots")
                self._cooldown_ns[identifier] = _nanoseconds(_number(p["cooldown_s"], "cooldown_s"))
                _number(p["launch"]["spring_energy_j"], "spring_energy_j")
                projectile = p["projectile"]
                for key in ("mass_kg", "added_mass_kg", "displaced_volume_m3"):
                    _number(projectile[key], key)
                if projectile["mass_kg"] + projectile["added_mass_kg"] <= 0:
                    raise ValueError("projectile inertial mass must be positive")
                _boolean(projectile["neutral_buoyancy"], "neutral_buoyancy")
                _number(projectile.get("center_of_mass_m", 0), "center_of_mass_m",
                        minimum=-math.inf)
                self._mounts[identifier] = tuple(mount.compose(_pose(
                    slot["position_m"], slot["orientation_wxyz"])) for slot in slots)
                self._release_configs[identifier] = _freeze(p)
            elif kind == "claw":
                minimum = _number(p["min_gap_m"], "min_gap_m")
                maximum = _number(p["max_gap_m"], "max_gap_m")
                speed = _number(p["jaw_speed_m_s"], "jaw_speed_m_s")
                tolerance = _number(p["completion_tolerance_m"], "completion_tolerance_m")
                if maximum <= minimum or speed <= 0 or tolerance >= (maximum - minimum) / 2:
                    raise ValueError("invalid claw travel, speed or completion tolerance")
                if p["initial_state"] not in ("open", "closed"):
                    raise ValueError("unsupported initial claw state")
                if p["timed_command"] != "signed_duration":
                    raise ValueError("unsupported timed claw command")
                initial = (maximum - minimum) / 2 if p["initial_state"] == "open" else 0.
                self._claws[identifier] = _Claw(minimum, maximum, speed, tolerance,
                                               initial, initial, initial)
            elif kind != "magnet" or p.get("actuated") is not False:
                raise ValueError(f"unsupported mechanism type {kind!r}")
        if not self._guarded <= ids:
            raise ValueError("arming references unknown mechanisms")
        self.reset(killed=_boolean(safety["initially_killed"], "initially_killed"))

    def reset(self, *, killed: bool) -> None:
        """Reset actuator state and local simulation time; released bodies are caller-owned."""
        _boolean(killed, "killed")
        self.time_ns = 0
        self.armed = self._initial_armed and not (killed and self._kill_disarms)
        self._available = {key: p["capacity"] for key, p in self._release_configs.items()}
        self._cooldowns: dict[str, tuple[int, str]] = {}
        for claw in self._claws.values():
            claw.q = claw.initial
            claw.stop()

    def _kill(self, killed: bool) -> None:
        _boolean(killed, "killed")
        if killed and self._kill_disarms:
            self.armed = False
        for key, claw in self._claws.items():
            if killed or (key in self._guarded and not self.armed):
                claw.stop()

    def _blocked(self, identifier: str, killed: bool) -> bool:
        return killed or (identifier in self._guarded and not self.armed)

    def set_armed(self, value: bool, *, killed: bool) -> CommandResult:
        _boolean(value, "armed")
        self._kill(killed)
        if value and killed and self._reject_arm_killed:
            return CommandResult(False, "Cannot arm while killed")
        self.armed = value
        self._kill(killed)
        return CommandResult(True, "Armed" if value else "Disarmed")

    def reload_all(self, *, killed: bool) -> CommandResult:
        self._kill(killed)
        self.armed = False
        self._available = {key: p["capacity"] for key, p in self._release_configs.items()}
        self._cooldowns.clear()
        self._kill(killed)
        return CommandResult(True, "Reloaded and disarmed")

    def fire(self, identifier: str, world_from_reference: native.Pose,
             linear_velocity_reference: Sequence[float], angular_velocity_reference: Sequence[float],
             *, reference_frame: str, water_density_kg_m3: float, killed: bool,
             insert: Callable[[PayloadRelease], None] | None = None) -> CommandResult:
        """Release using velocity at the named reference origin, expressed in its axes.

        Optional insert must synchronously/atomically create the caller-owned body. If it
        raises, ammunition and cooldown are unchanged. It must not reenter this runtime.
        """
        p = self._release_configs[identifier]
        world = native.Pose().compose(world_from_reference)
        velocity = _vector(linear_velocity_reference, "linear velocity")
        omega = _vector(angular_velocity_reference, "angular velocity")
        density = _number(water_density_kg_m3, "water_density_kg_m3")
        if density <= 0:
            raise ValueError("water density must be positive")
        root_from_reference = self._frames.from_root(reference_frame)
        self._kill(killed)
        if self._blocked(identifier, killed):
            return CommandResult(False, "Mechanism is disarmed or vehicle is killed")
        group = p["cooldown_group"]
        if self.time_ns < self._cooldowns.get(group, (0, ""))[0]:
            return CommandResult(False, "Release group is busy")
        if self._available[identifier] == 0:
            return CommandResult(False, "No ammunition; reload first")
        index = p["capacity"] - self._available[identifier]
        mount = root_from_reference.inverse().compose(self._mounts[identifier][index])
        pose = world.compose(mount)
        rotation = native.Pose()
        rotation.orientation_wxyz = world.orientation_wxyz
        axis_pose = native.Pose()
        axis_pose.orientation_wxyz = pose.orientation_wxyz
        axis = np.asarray(axis_pose.apply([1, 0, 0]))
        projectile = p["projectile"]
        mass = density * projectile["displaced_volume_m3"] if projectile["neutral_buoyancy"] \
            else projectile["mass_kg"]
        inertia = mass + projectile["added_mass_kg"]
        if inertia <= 0:
            raise ValueError("effective projectile inertial mass must be positive")
        speed = math.sqrt(2 * p["launch"]["spring_energy_j"] / inertia)
        world_omega = np.asarray(rotation.apply(omega))
        world_velocity = np.asarray(rotation.apply(velocity + np.cross(omega, mount.translation)))
        world_velocity += axis * speed + np.cross(
            world_omega, axis * projectile.get("center_of_mass_m", 0.))
        if not np.isfinite(world_velocity).all():
            raise ValueError("release velocity is not finite")
        deadline = self.time_ns + self._cooldown_ns[identifier]
        if deadline > 2**63 - 1:
            raise ValueError("cooldown exceeds simulation time range")
        release = PayloadRelease(identifier, p["slots"][index]["id"], index, self.time_ns,
                                 tuple(pose.translation), tuple(pose.orientation_wxyz),
                                 tuple(world_velocity), tuple(world_omega), projectile)
        if insert is not None:
            insert(release)
        self._available[identifier] -= 1
        self._cooldowns[group] = (deadline, identifier)
        return CommandResult(True, "Payload released", release)

    def command_claw(self, identifier: str, opened: bool, *, killed: bool) -> CommandResult:
        _boolean(opened, "opened")
        return self._command_claw(identifier, opened, None, killed)

    def move_claw(self, identifier: str, signed_duration_s: float, *, killed: bool) -> CommandResult:
        value = _number(signed_duration_s, "signed_duration_s", minimum=-math.inf)
        return self._command_claw(identifier, value > 0, _nanoseconds(abs(value)), killed)

    def _command_claw(self, identifier: str, opened: bool, duration_ns: int | None,
                      killed: bool) -> CommandResult:
        claw = self._claws[identifier]
        self._kill(killed)
        if self._blocked(identifier, killed):
            return CommandResult(False, "Mechanism is disarmed or vehicle is killed")
        if duration_ns == 0:
            claw.stop()
        else:
            claw.target = claw.travel if opened else 0.
            claw.direction, claw.remaining_ns = (1 if opened else -1), duration_ns
        return CommandResult(True, "Claw command accepted; grasp physics unavailable")

    def advance(self, dt_ns: int, *, killed: bool) -> None:
        if type(dt_ns) is not int or dt_ns < 0 or self.time_ns + dt_ns > 2**63 - 1:
            raise ValueError("dt_ns must be nonnegative and fit the simulation time range")
        self._kill(killed)
        for claw in self._claws.values():
            # Match the original timer-before-drive boundary: expiry stops this entire step.
            if claw.remaining_ns is not None:
                claw.remaining_ns -= dt_ns
                if claw.remaining_ns <= 0:
                    claw.stop()
            distance = claw.speed * dt_ns / 1e9
            claw.q += max(-distance, min(distance, claw.target - claw.q))
        self.time_ns += dt_ns

    def slot_count(self, identifier: str) -> int:
        return len(self._mounts[identifier])

    def slot_mount(self, identifier: str, index: int) -> native.Pose:
        """Pose of a launcher/dropper slot in the robot root frame (COM)."""
        return self._mounts[identifier][index]

    def snapshot(self, *, killed: bool) -> MechanismState:
        self._kill(killed)
        releases = {}
        busy = any(self.time_ns < deadline for deadline, _ in self._cooldowns.values())
        for key, p in self._release_configs.items():
            deadline, owner = self._cooldowns.get(p["cooldown_group"], (0, ""))
            state = "disarmed" if self._blocked(key, killed) else (
                "busy" if self.time_ns < deadline and owner == key else
                "loaded" if self._available[key] else "empty")
            releases[key] = ReleaseState(state, self._available[key])
        claws = {}
        for key, claw in self._claws.items():
            moving = not self._blocked(key, killed) and abs(claw.q - claw.target) > claw.tolerance
            busy = busy or moving
            state = "disarmed" if self._blocked(key, killed) else (
                ("opening" if claw.direction > 0 else "closing") if moving else
                "opened" if claw.q > claw.travel - claw.tolerance else "closed")
            claws[key] = ClawState(state, claw.minimum + 2 * claw.q,
                                   claw.minimum + 2 * claw.target, (claw.q, claw.q))
        return MechanismState(self.time_ns, self.armed, busy,
                              MappingProxyType(releases), MappingProxyType(claws))
