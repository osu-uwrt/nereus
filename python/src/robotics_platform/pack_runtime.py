"""Construct the native plant and explicitly selected sensors from resolved data packs."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Iterable, TypeVar

import numpy as np

from . import _native as native

if TYPE_CHECKING:
    from .packs import ResolvedScenario

T = TypeVar("T")


def _assign(target: T, values: dict[str, Any], fields: dict[str, str]) -> T:
    for key, attribute in fields.items():
        if key in values:
            setattr(target, attribute, values[key])
    return target


def _pose(position: Iterable[float], quaternion: Iterable[float]) -> native.Pose:
    pose = native.Pose()
    pose.translation = list(position)
    pose.orientation_wxyz = list(quaternion)
    return pose


def _frames(config: dict[str, Any]) -> native.FixedFrames:
    edges = []
    for entry in config["transforms"]:
        edge = native.FixedFrame()
        edge.parent, edge.child = entry["parent"], entry["child"]
        pose = native.Pose()
        pose.translation = entry["position_m"]
        pose.orientation_wxyz = entry["orientation_wxyz"]
        edge.pose = pose
        edges.append(edge)
    return native.FixedFrames(config["root"], edges)


def _box(
    config: dict[str, Any], position: Iterable[float], quaternion: Iterable[float]
) -> native.BoxProxy:
    box = native.BoxProxy()
    box.id = config["id"]
    box.size = config["size_m"]
    pose = _pose(position, quaternion).compose(
        _pose(config["center_m"], config.get("orientation_wxyz", [1, 0, 0, 0]))
    )
    box.center = pose.translation
    box.orientation_wxyz = pose.orientation_wxyz
    return box


def _noise(config: dict[str, Any], enabled: bool, *, scalar: bool = False) -> Any:
    result = native.ScalarNoiseParameters() if scalar else native.NoiseParameters()
    if enabled:
        _assign(result, config, {key: key for key in ("bias", "white_stddev", "walk_stddev")})
    return result


def _reporting(config: dict[str, Any]) -> native.ImuReporting:
    return _assign(
        native.ImuReporting(),
        config,
        {
            "gravity_magnitude_m_s2": "gravity_magnitude",
            "force_variance": "force_variance",
            "angular_variance": "angular_variance",
        },
    )


def _attitude(config: dict[str, Any], noise: bool) -> native.AttitudeParameters:
    result = _assign(
        native.AttitudeParameters(),
        config,
        {
            "heading_axis_world": "heading_axis_world",
            "reported_variance": "reported_variance",
        },
    )
    if noise:
        _assign(
            result,
            config,
            {"angle_stddev_rad": "angle_stddev", "heading_drift_rad_s": "heading_drift_rate"},
        )
    return result


def _sensor(
    config: dict[str, Any],
    frames: native.FixedFrames,
    pool: native.Pool,
    surface_pressure: float,
    noise: bool,
) -> Any:
    pose = frames.from_root(config["mount_frame"])
    mount = native.Mount()
    mount.position_body, mount.orientation_wxyz = pose.translation, pose.orientation_wxyz
    p = config["parameters"]
    kind = config["type"]
    if kind == "imu":
        return native.Imu(
            mount,
            _noise(p.get("acceleration_noise", {}), noise),
            _noise(p.get("gyro_noise", {}), noise),
            _reporting(p.get("reporting", {})),
        )
    if kind == "attitude":
        return native.Attitude(mount, _attitude(p, noise))
    if kind == "ahrs":
        ahrs = native.AhrsParameters()
        inertial = p["inertial"]
        ahrs.acceleration_noise = _noise(inertial.get("acceleration_noise", {}), noise)
        ahrs.gyro_noise = _noise(inertial.get("gyro_noise", {}), noise)
        ahrs.inertial_reporting = _reporting(inertial.get("reporting", {}))
        ahrs.attitude = _attitude(p["attitude"], noise)
        return native.Ahrs(mount, ahrs)
    if kind == "fog":
        return native.Fog(
            mount, p["axes"], _noise(p.get("gyro_noise", {}), noise), p.get("reported_variance")
        )
    if kind == "reference_velocity":
        velocity = native.ReferenceVelocityParameters()
        velocity.mount, velocity.noise = mount, _noise(p.get("velocity_noise", {}), noise)
        _assign(
            velocity,
            p,
            {
                "reference_velocity_world_m_s": "reference_velocity_world",
                "reported_variance": "reported_variance",
            },
        )
        if "inclination_limit" in p:
            velocity.inclination_limit = _assign(
                native.InclinationLimit(),
                p["inclination_limit"],
                {
                    "axis": "sensor_axis",
                    "maximum_angle_rad": "maximum_angle",
                },
            )
        return native.ReferenceVelocity(velocity)
    if kind == "reference_altitude":
        altitude = native.ReferenceAltitudeParameters()
        altitude.mount, altitude.noise = mount, _noise(p.get("noise", {}), noise, scalar=True)
        _assign(
            altitude,
            p,
            {
                "target_position_body_m": "target_position_body",
                "reported_variance": "reported_variance",
            },
        )
        return native.ReferenceAltitude(altitude)
    if kind == "dvl":
        dvl = native.DvlParameters()
        dvl.mount, dvl.velocity_noise = mount, _noise(p.get("velocity_noise", {}), noise)
        _assign(
            dvl,
            p,
            {
                "bottom_axis": "bottom_axis",
                "minimum_range_m": "minimum_range",
                "maximum_range_m": "maximum_range",
            },
        )
        return native.Dvl(dvl, pool)
    if kind == "pressure":
        pressure = native.PressureParameters()
        pressure.mount, pressure.noise = mount, _noise(p.get("noise", {}), noise, scalar=True)
        _assign(
            pressure,
            p,
            {
                "reference_pressure_pa": "reference_pressure",
                "reference_density_kg_m3": "reference_density",
                "reference_gravity_m_s2": "reference_gravity",
                "minimum_pressure_pa": "minimum_pressure",
                "maximum_pressure_pa": "maximum_pressure",
            },
        )
        environment = native.HydrostaticPressure(
            pool.water_level, pool.water_density, surface_pressure
        )
        return native.Pressure(pressure, environment)
    raise ValueError(f"sensor {config['id']!r}: no native physics implementation for {kind!r}")


@dataclass(frozen=True)
class PackRuntime:
    runtime: native.Runtime
    parameters: native.PlantParameters
    frames: native.FixedFrames
    initial: native.BodyState
    streams: dict[str, Any]
    deferred_sensor_ids: tuple[str, ...]


def create_runtime(
    resolved: ResolvedScenario, *, sensor_ids: Iterable[str] | None = None
) -> PackRuntime:
    """Construct physics without ROS, a viewer, a wall clock, or task Python.

    A caller can explicitly select sensors for a partial/headless execution. Omitted
    sensors are reported in the result; unsupported requested sensors are errors.
    The caller owns stepping and must record this selection with the resolved run.
    """
    robot, world, scenario = resolved.robot, resolved.pool, resolved.scenario
    parameters = native.PlantParameters()
    body = robot["body"]["parameters"]
    parameters.body = _assign(
        native.BodyParameters(),
        body,
        {
            "mass_kg": "mass",
            "inertia_matrix": "inertia",
            "added_mass_matrix": "added_mass",
            "linear_damping_matrix": "linear_damping",
            "quadratic_damping": "quadratic_damping",
            "damping_center_m": "damping_center",
            "displaced_volume_m3": "displaced_volume",
            "buoyancy_center_m": "buoyancy_center",
            "buoyancy_radii_m": "buoyancy_radii",
        },
    )
    parameters.command_timeout = body["command_timeout_s"]
    parameters.timestep_ns = scenario["timestep_ns"]
    pool = _assign(
        native.Pool(),
        world["parameters"],
        {
            "length_m": "length",
            "width_m": "width",
            "depth_m": "depth",
            "water_level_m": "water_level",
            "water_density_kg_m3": "water_density",
            "current_m_s": "current_velocity",
            "current_oscillation_amplitude_m_s": "current_oscillation_amplitude",
            "current_oscillation_frequency_hz": "current_oscillation_frequency",
        },
    )
    placement = scenario["pool_placement"]
    pool.yaw_world = math.radians(placement["yaw_deg"])
    pool.origin_xy_world = placement["position_m"][:2]
    pool.water_level += placement["position_m"][2]
    parameters.pool = pool
    parameters.thrusters = [
        _assign(
            native.Thruster(),
            {**entry, **entry["parameters"]},
            {
                "id": "id",
                "position_m": "position",
                "direction": "direction",
                "delay_s": "delay",
                "rise_time_s": "rise_time",
                "fall_time_s": "fall_time",
                "slew_rate_n_s": "slew_rate",
                "forward_limit_n": "forward_limit",
                "reverse_limit_n": "reverse_limit",
                "deadband_n": "deadband",
                "forward_scale": "forward_scale",
                "reverse_scale": "reverse_scale",
                "efficiency": "efficiency",
                "propeller_radius_m": "propeller_radius",
            },
        )
        for entry in robot["thrusters"]
    ]
    contacts = native.ContactParameters()
    contacts.model = {
        "disabled": native.ContactModel.DISABLED,
        "sphere_pool": native.ContactModel.SPHERE_POOL,
        "box_scene": native.ContactModel.BOX_SCENE,
    }[scenario["contacts"]["model"]]
    contacts.restitution, contacts.friction = (
        scenario["contacts"]["restitution"],
        scenario["contacts"]["friction"],
    )
    contacts.body_boxes = [
        _box(box, np.zeros(3), np.array([1.0, 0, 0, 0])) for box in robot["collision_boxes"]
    ]
    yaw = pool.yaw_world / 2
    pool_quaternion = np.array([math.cos(yaw), 0, 0, math.sin(yaw)])
    boxes = [
        _box(box, np.asarray(placement["position_m"]), pool_quaternion)
        for box in world["collision_boxes"]
    ]
    tasks = {task["id"]: task for task in resolved.task_definitions}
    for instance in scenario["task_placements"]:
        yaw = math.radians(instance["yaw_deg"]) / 2
        quaternion = np.array([math.cos(yaw), 0, 0, math.sin(yaw)])
        for prop in tasks[instance["task"]]["props"]:
            if prop["type"] != "static_body":
                raise ValueError(
                    f"prop {prop['id']!r}: no native contact implementation for {prop['type']!r}"
                )
            for definition in prop["parameters"]["collision_boxes"]:
                box = _box(definition, np.asarray(instance["position_m"]), quaternion)
                box.id = f"{instance['task']}/{prop['id']}/{box.id}"
                boxes.append(box)
    contacts.world_boxes = boxes
    parameters.contacts = contacts
    frames = _frames(robot["frames"])
    initial = _assign(
        native.BodyState(),
        scenario["initial"],
        {
            "position_m": "position",
            "orientation_wxyz": "orientation_wxyz",
            "linear_velocity_m_s": "linear_velocity",
            "angular_velocity_rad_s": "angular_velocity",
        },
    )
    if scenario["initial"]["frame"] != frames.root:
        pose = frames.from_root(scenario["initial"]["frame"])
        com_pose = _pose(initial.position, initial.orientation_wxyz).compose(pose.inverse())
        initial.position = com_pose.translation
        initial.orientation_wxyz = com_pose.orientation_wxyz
    runtime = native.Runtime(parameters, initial, scenario["seed"])
    sensors = {entry["id"]: entry for entry in robot["sensors"]}
    selected = (
        [name for name, entry in sensors.items() if entry.get("enabled", True)]
        if sensor_ids is None
        else list(sensor_ids)
    )
    if len(selected) != len(set(selected)) or not set(selected) <= sensors.keys():
        raise ValueError("selected sensors must have unique ids from the robot pack")
    streams = {}
    for name in selected:
        config = sensors[name]
        if not config.get("enabled", True):
            raise ValueError(f"selected sensor {name!r} is disabled in the robot pack")
        model = _sensor(
            config,
            frames,
            pool,
            world["parameters"]["surface_pressure_pa"],
            scenario["sensor_noise"],
        )
        device = native.Device(
            name,
            config["frame"],
            config["period_ns"],
            config.get("latency_ns", 0),
            config.get("capacity", 64),
            {"fail": native.OverflowPolicy.FAIL, "drop_oldest": native.OverflowPolicy.DROP_OLDEST}[
                config.get("overflow", "fail")
            ],
        )
        streams[name] = runtime.add(device, model)
    return PackRuntime(
        runtime,
        parameters,
        frames,
        initial,
        streams,
        tuple(name for name in sensors if name not in selected),
    )
