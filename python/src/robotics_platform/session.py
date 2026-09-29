"""One coordinated simulation: plant, sensors, mechanisms, payloads and task observation.

The session is the single owner that advances everything once per physics tick, in a
fixed order: plant step; mechanism actuation; payload propagation and task judging;
robot task observation. Transports (the ROS bridge, scripts, tests) call only this
object, so a full reset clears every component together. No wall time, ROS or viewer.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from . import _native as native
from .mechanisms import CommandResult, Mechanisms, MechanismState, PayloadRelease
from .pack_runtime import PackRuntime, _pose
from .packs import ResolvedScenario
from .task_runtime import TaskRuntime

_MODELS = {"finned_rigid_body": "FINNED", "fixed_axis_body": "FIXED_AXIS"}


def _payload_parameters(projectile: Mapping[str, Any]) -> native.PayloadParameters:
    p = native.PayloadParameters()
    model = _MODELS.get(projectile["model"])
    if model is None:
        raise ValueError(f"unsupported projectile model {projectile['model']!r}")
    p.model = getattr(native.PayloadModel, model)
    p.mass, p.displaced_volume = projectile["mass_kg"], projectile["displaced_volume_m3"]
    p.neutral_buoyancy, p.added_mass = projectile["neutral_buoyancy"], projectile["added_mass_kg"]
    p.length, p.radius = projectile["length_m"], projectile["radius_m"]
    p.drag_axial, p.drag_lateral = projectile["drag_axial"], projectile["drag_lateral"]
    p.center_of_mass = projectile.get("center_of_mass_m", 0.0)
    p.center_of_buoyancy = projectile.get("center_of_buoyancy_m", 0.0)
    p.center_of_drag = projectile.get("center_of_drag_m", 0.0)
    p.angular_damping = projectile.get("angular_damping_n_m_s", 0.0)
    return p


def _axis(orientation_wxyz: ArrayLike) -> NDArray[np.float64]:
    rotation = native.Pose()
    rotation.orientation_wxyz = orientation_wxyz
    return np.asarray(rotation.apply([1.0, 0.0, 0.0]), float)


@dataclass
class Payload:
    """A released body owned by the session until it stops (then kept for display)."""

    identifier: int
    mechanism_id: str
    mechanism_type: str
    dynamics: native.PayloadDynamics
    state: native.PayloadState
    released_ns: int
    max_age_ns: int
    active: bool = True
    outcome: str = ""


@dataclass(frozen=True)
class Step:
    snapshot: Any
    task_events: tuple[Mapping[str, Any], ...] = ()


@dataclass
class _Environment:
    density: float
    level: float
    current: NDArray[np.float64]
    amplitude: NDArray[np.float64]
    frequency: float
    water: native.PayloadEnvironment = field(default_factory=native.PayloadEnvironment)

    def at(self, time_s: float) -> native.PayloadEnvironment:
        self.water.water_density, self.water.water_level = self.density, self.level
        self.water.water_velocity = self.current + self.amplitude * math.sin(
            2 * math.pi * self.frequency * time_s
        )
        return self.water


class Session:
    """Owns and advances one scenario run; every mutation goes through its methods."""

    def __init__(
        self,
        resolved: ResolvedScenario,
        pack: PackRuntime,
        *,
        task_ids: Sequence[str] | None = None,
        tasks: bool | None = None,
    ) -> None:
        """``tasks=None`` observes tasks when the scenario selects any; robots without
        mechanisms and scenarios without tasks run as plain navigation plants."""
        self.resolved, self.pack = resolved, pack
        self.runtime = pack.runtime
        self.robot = resolved.robot
        self.timestep_ns = int(pack.parameters.timestep_ns)
        self.reference_frame = self.robot["reference_frame"]
        self._root_to_reference = pack.frames.from_root(self.reference_frame)
        self.mechanisms = Mechanisms(self.robot) if self.robot.get("mechanisms") else None
        self._mechanism_types = {
            item["id"]: item["type"] for item in self.robot.get("mechanisms", [])
        }
        self._projectiles = {
            item["id"]: item["parameters"]["projectile"]
            for item in self.robot.get("mechanisms", [])
            if item["type"] in ("launcher", "dropper")
        }
        self._dynamics = {
            key: native.PayloadDynamics(_payload_parameters(value))
            for key, value in self._projectiles.items()
        }
        if tasks is None:
            tasks = bool(getattr(resolved, "task_definitions", ()))
        self.tasks = TaskRuntime(resolved, task_ids=task_ids) if tasks else None
        # Tasks with a contact_world prop get their own rigid-body world (optional pybullet).
        self.prop_worlds: list[Any] = []
        if self.tasks is not None:
            selected = set(task_ids) if task_ids is not None else None
            for task in resolved.task_definitions:
                if (selected is None or task["id"] in selected) and any(
                    prop["type"] == "contact_world" for prop in task["props"]
                ):
                    from .prop_world import PropWorld

                    self.prop_worlds.append(PropWorld(resolved, task=task["id"]))
        self._environment = None
        if self._dynamics:
            pool = pack.parameters.pool
            self._environment = _Environment(
                float(pool.water_density),
                float(pool.water_level),
                np.asarray(pool.current_velocity, float),
                np.asarray(pool.current_oscillation_amplitude, float),
                float(pool.current_oscillation_frequency),
            )
        self.start_state = pack.initial
        self.seed = int(resolved.scenario["seed"])
        self._reset_owned_state()

    def _reset_owned_state(self) -> None:
        self.killed = bool(self.robot["safety"]["initially_killed"])
        if self.mechanisms is not None:
            self.mechanisms.reset(killed=self.killed)
        self.payloads: dict[int, Payload] = {}
        self._next_payload = 0
        self.last_step = Step(self.runtime.observe())

    # ------------------------------------------------------------------ time

    @property
    def time_ns(self) -> int:
        return int(self.last_step.snapshot.elapsed_ns)

    def advance(self) -> Step:
        """Advance exactly one physics tick and every model that depends on it."""
        snapshot = self.runtime.advance(1)
        now = int(snapshot.elapsed_ns)
        events: list[Mapping[str, Any]] = []
        if self.mechanisms is not None:
            self.mechanisms.advance(self.timestep_ns, killed=self.killed)
        if self._environment is not None and self.payloads:
            environment = self._environment.at(now / 1e9)
            for payload in self.payloads.values():
                if payload.active:
                    events += self._propagate(payload, now, environment)
        if self.prop_worlds:
            events += self._step_props(snapshot.body, now)
        if self.tasks is not None:
            events += self.tasks.observe(now, self.reference_pose(snapshot.body))
        self.last_step = Step(snapshot, tuple(events))
        return self.last_step

    def _propagate(
        self, payload: Payload, now: int, environment: native.PayloadEnvironment
    ) -> list[Mapping[str, Any]]:
        old = payload.state
        new = payload.dynamics.advance(old, environment, self.timestep_ns / 1e9)
        payload.state = new
        if now - payload.released_ns > payload.max_age_ns:
            payload.active, payload.outcome = False, "timeout"
        if self.tasks is None:
            return []
        step = self.tasks.step_projectile(
            now,
            payload.identifier,
            old.position.tolist(),
            new.position.tolist(),
            _axis(new.orientation_wxyz).tolist(),
            new.velocity.tolist(),
        )
        if step.position_world is not None:
            payload.state.position = step.position_world
        if step.velocity_world is not None:
            payload.state.velocity = step.velocity_world
        if step.stop:
            ended = [e["id"] for e in step.events]
            payload.active, payload.outcome = False, ended[-1] if ended else "stopped"
            payload.state.velocity = np.zeros(3)
            payload.state.angular_velocity = np.zeros(3)
        return list(step.events)

    def _step_props(self, body: Any, now: int) -> list[Mapping[str, Any]]:
        from .prop_world import Water

        assert self.tasks is not None and self.mechanisms is not None
        root = _pose(body.position, body.orientation_wxyz)
        rotation = native.Pose()
        rotation.orientation_wxyz = root.orientation_wxyz
        velocity = np.asarray(rotation.apply(body.linear_velocity), float).tolist()
        omega = np.asarray(rotation.apply(body.angular_velocity), float).tolist()
        state = self.mechanisms.snapshot(killed=self.killed)
        pool = self.pack.parameters.pool
        flow = np.asarray(pool.current_velocity, float) + np.asarray(
            pool.current_oscillation_amplitude, float
        ) * math.sin(2 * math.pi * float(pool.current_oscillation_frequency) * now / 1e9)
        water = Water((float(flow[0]), float(flow[1]), float(flow[2])), float(pool.water_density))
        events: list[Mapping[str, Any]] = []
        for world in self.prop_worlds:
            claw = state.claws[world.mechanism_id]
            produced = world.step(
                self.timestep_ns / 1e9,
                now,
                root,
                velocity,
                omega,
                list(claw.joint_positions_m),
                water,
                enabled=state.armed and not self.killed,
            )
            if produced:
                events += self.tasks.record(now, produced)
        return events

    def props(self) -> dict[str, Mapping[str, Any]]:
        """Current rigid-prop poses per task, for rendering and records."""
        return {world.task: world.props_state() for world in self.prop_worlds}

    # ------------------------------------------------------------------ robot state

    def reference_pose(self, body: Any) -> native.Pose:
        return _pose(body.position, body.orientation_wxyz).compose(self._root_to_reference)

    def _reference_velocities(self, body: Any) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Body-axis COM velocities -> velocity at the reference origin in reference axes."""
        offset = np.asarray(self._root_to_reference.translation, float)
        omega = np.asarray(body.angular_velocity, float)
        velocity = np.asarray(body.linear_velocity, float) + np.cross(omega, offset)
        rotation = native.Pose()
        rotation.orientation_wxyz = self._root_to_reference.inverse().orientation_wxyz
        return np.asarray(rotation.apply(velocity)), np.asarray(rotation.apply(omega))

    # ------------------------------------------------------------------ commands

    def command_thrusters(self, forces: Any) -> None:
        self.runtime.command(np.asarray(forces, float))

    def set_killed(self, killed: bool) -> None:
        self.killed = bool(killed)
        if self.killed and self.robot["safety"]["kill_stops_thrusters"]:
            self.runtime.stop_thrusters()
        if self.mechanisms is not None:
            self.mechanisms.advance(0, killed=self.killed)

    def _mechanism_type(self, identifier: str, *kinds: str) -> str | None:
        kind = self._mechanism_types.get(identifier)
        return None if kind in kinds else f"unknown {'/'.join(kinds)} mechanism {identifier!r}"

    def set_armed(self, armed: bool) -> CommandResult:
        if self.mechanisms is None:
            return CommandResult(False, "robot has no mechanisms")
        return self.mechanisms.set_armed(bool(armed), killed=self.killed)

    def reload_all(self) -> CommandResult:
        if self.mechanisms is None:
            return CommandResult(False, "robot has no mechanisms")
        return self.mechanisms.reload_all(killed=self.killed)

    def command_claw(self, identifier: str, opened: bool) -> CommandResult:
        error = self._mechanism_type(identifier, "claw")
        if error or self.mechanisms is None:
            return CommandResult(False, error or "robot has no mechanisms")
        return self.mechanisms.command_claw(identifier, bool(opened), killed=self.killed)

    def move_claw(self, identifier: str, signed_duration_s: float) -> CommandResult:
        error = self._mechanism_type(identifier, "claw")
        if error or self.mechanisms is None:
            return CommandResult(False, error or "robot has no mechanisms")
        return self.mechanisms.move_claw(identifier, float(signed_duration_s), killed=self.killed)

    def fire(self, identifier: str) -> CommandResult:
        """Release the next payload of a launcher/dropper at the current tick."""
        if identifier not in self._dynamics or self.mechanisms is None:
            return CommandResult(False, f"unknown release mechanism {identifier!r}")
        assert self._environment is not None
        body = self.last_step.snapshot.body
        velocity, omega = self._reference_velocities(body)
        now = self.time_ns
        released: list[PayloadRelease] = []
        result = self.mechanisms.fire(
            identifier,
            self.reference_pose(body),
            velocity.tolist(),
            omega.tolist(),
            reference_frame=self.reference_frame,
            water_density_kg_m3=self._environment.density,
            killed=self.killed,
            insert=released.append,
        )
        if not result.accepted:
            return result
        release = released[0]
        state = native.PayloadState()
        state.position, state.orientation_wxyz = (
            release.position_world_m,
            release.orientation_wxyz,
        )
        state.velocity = release.velocity_com_world_m_s
        state.angular_velocity = release.angular_velocity_world_rad_s
        projectile = self._projectiles[identifier]
        payload = Payload(
            self._next_payload,
            identifier,
            self._mechanism_types[identifier],
            self._dynamics[identifier],
            state,
            now,
            round(projectile.get("max_age_s", 30) * 1e9),
        )
        self._next_payload += 1
        self.payloads[payload.identifier] = payload
        if self.tasks is not None:
            tip = np.asarray(state.position) + _axis(state.orientation_wxyz) * (
                projectile["length_m"] / 2
            )
            events = self.tasks.release_projectile(
                now,
                payload.identifier,
                payload.mechanism_type,
                tip,
                projectile["radius_m"],
                projectile["length_m"],
            )
            self.last_step = Step(
                self.last_step.snapshot, self.last_step.task_events + tuple(events)
            )
        return result

    def mechanism_state(self) -> MechanismState | None:
        return None if self.mechanisms is None else self.mechanisms.snapshot(killed=self.killed)

    # ------------------------------------------------------------------ resets

    def place(self, state: Any, *, clear_actuators: bool) -> Any:
        snapshot = self.runtime.place(state, clear_actuators=clear_actuators)
        self.last_step = Step(snapshot)
        return snapshot

    def reset_tasks(self) -> tuple[bool, str]:
        """Original reset_tasks: payloads cleared, reload + disarm, scores/judges reset."""
        self.payloads.clear()
        if self.mechanisms is not None:
            self.mechanisms.reload_all(killed=self.killed)
        if self.tasks is not None:
            self.tasks.reset()
        for world in self.prop_worlds:
            world.reset()
        return True, "All tasks reset; ammunition reloaded and actuators disarmed"

    def full_reset(self, seed: int | None = None) -> Any:
        """Restore the scenario start: plant, sensors, noise, mechanisms, payloads, tasks."""
        if seed is not None:
            self.seed = int(seed)
        snapshot = self.runtime.reset(self.pack.initial, self.seed)
        self.start_state = self.pack.initial
        self._reset_owned_state()
        if self.tasks is not None:
            self.tasks.reset()
        for world in self.prop_worlds:
            world.reset()
        self.last_step = Step(snapshot)
        return snapshot
