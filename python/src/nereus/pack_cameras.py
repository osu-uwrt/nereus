"""Synchronous offscreen stereo capture composed from a resolved scenario's packs.

Standalone: needs only the optional camera extension (no simulation runtime, no ROS).
Robot poses are the robot frame root (``robot.frames.root``) in the scenario world frame;
camera eyes follow the robot pack's frame tree. Scene content, appearance, calibration and
noise come only from pack data; unsupported content is rejected rather than skipped.
Pose validation, composition and frame resolution are the native nereus_spatial ones.
"""

from __future__ import annotations

import copy
import hashlib
import math
import threading
from collections.abc import Iterable, Mapping
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from . import _camera

if TYPE_CHECKING:
    from .packs import ResolvedScenario

NEAR_PLANE_M = 0.05  # original camera_faker clipping planes
FAR_PLANE_M = 100.0
DEFAULT_JPEG_QUALITY = 93  # unused unless JPEG is requested
PANEL_TOLERANCE_M = 5e-4  # declared panel faces are sub-millimetre data
SEED_POLICY = ("uint32 = first 4 bytes, big-endian, of sha256(b'nereus.camera.v1'"
               " NUL decimal(scenario seed) NUL sensor id NUL eye)")
EYES = ("left", "right")
_NOISE = {  # pack depth.noise key -> native DepthNoise attribute
    "base_sigma_m": "base_sigma", "range_coefficient": "range_sigma",
    "range_exponent": "exponent", "bias_m": "bias", "dropout": "dropout",
    "range_dropout": "range_dropout", "edge_dropout": "edge_dropout",
    "outliers": "outliers", "correlation": "correlation", "patch_size_px": "patch_size",
}


@dataclass(frozen=True)
class CameraCapture:
    """One synchronous acquisition. Frames are top-down; None means not requested."""

    sensor_id: str
    left: _camera.Frame | None
    right: _camera.Frame | None


def derive_seed(seed: int, sensor_id: str, eye: str) -> int:
    """Stable 32-bit processor seed; independent of process hashing and camera order."""
    text = b"\0".join([b"nereus.camera.v1", str(seed).encode(), sensor_id.encode(),
                       eye.encode()])
    return int.from_bytes(hashlib.sha256(text).digest()[:4], "big")


def pose(position_m: Any, orientation_wxyz: Any) -> _camera.Pose:
    """Native pose; validated by the native operations that use it."""
    result = _camera.Pose()
    result.translation = position_m
    result.orientation_wxyz = orientation_wxyz
    return result


def _placed(item: Mapping[str, Any]) -> _camera.Pose:
    return pose(item["position_m"], item["orientation_wxyz"])


def _upright(item: Mapping[str, Any]) -> _camera.Pose:
    half = math.radians(item["yaw_deg"]) / 2
    return pose(item["position_m"], [math.cos(half), 0.0, 0.0, math.sin(half)])


class PackCameras:
    """Offscreen RGB/depth for a scenario's stereo cameras, one synchronous call per frame.

    The scene and EGL host are built once; GL capture is serialized. Different cameras may
    process depth/JPEG concurrently; each camera's stereo acquisition and noise stay ordered.
    Robot visuals follow the supplied root pose; pool and task content is static.
    No mechanism motion or lights.
    """

    def __init__(self, resolved: ResolvedScenario, sensor_ids: Iterable[str] | None = None, *,
                 shader_directory: Path | None = None) -> None:
        self._lock = threading.Lock()
        self._resolved: ResolvedScenario = resolved
        self._meshes: dict[tuple[str, str, str | None], _camera.Mesh] = {}
        self._files: dict[tuple[str, str], dict[str, Any]] = {}
        scenario, robot, pool = resolved.scenario, resolved.robot, resolved.pool
        self._cameras = self._select(robot, sensor_ids)
        self._capture_locks = {sensor: threading.Lock() for sensor in self._cameras}
        self._frames = _camera.FixedFrames(robot["frames"]["root"], [
            self._edge(item) for item in robot["frames"]["transforms"]])
        self._sensor_noise = bool(scenario["sensor_noise"])
        for camera in self._cameras.values():
            self._configure(camera)
        self._appearance, self._appearance_record = self._appearance_from(pool)
        self._scene, self._pool_record = self._pool_scene(pool, scenario["pool_placement"])
        self._cutouts: list[dict[str, Any]] = []
        self._unrendered: list[str] = []
        self._static = self._scene.instances + self._task_instances()
        self._robot_visuals = [
            (self._mesh("robot", item["asset"], item.get("texture")),
             self._frames.from_root(item["frame"]).compose(_placed(item)).matrix())
            for item in robot.get("visuals", [])
        ]
        # Importers may silently omit a missing sidecar. Check the resolved snapshot
        # as well as successfully imported files before accepting the scene.
        changed = resolved.changed_sources()
        if changed:
            raise ValueError("pack sources changed since resolution (missing file or sha256 "
                             "mismatch): " + ", ".join(str(path) for path in changed))
        shaders = shader_directory or Path(_camera.__file__).parent / "shaders"
        self._host = _camera.OffscreenRenderer(shaders)
        self.reset(scenario["seed"])

    # ------------------------------------------------------------------ public API

    @property
    def sensor_ids(self) -> tuple[str, ...]:
        return tuple(self._cameras)

    def capture(self, sensor_id: str, position_m: Any, orientation_wxyz: Any, time_s: float, *,
                jpeg_quality: int | None = None) -> CameraCapture:
        """Render and process the requested outputs with the robot root at the given pose.

        Everything is validated before rendering; a failed call consumes no noise state.
        """
        camera = self._get_camera(sensor_id)
        root = pose(position_m, orientation_wxyz)
        world_root = root.matrix()  # native validation
        time = float(time_s)
        if not math.isfinite(time):
            raise ValueError("time_s must be finite")
        if jpeg_quality is not None and (isinstance(jpeg_quality, bool) or
                                         not isinstance(jpeg_quality, int) or
                                         not 0 <= jpeg_quality <= 100):
            raise ValueError("jpeg_quality must be None or an integer in [0, 100]")
        outputs = camera["outputs"]
        wanted = {"left": ("rgb_left" in outputs, "depth_left" in outputs),
                  "right": ("rgb_right" in outputs, False)}
        views = {eye: self._view(camera, eye, root) for eye, (color, depth) in wanted.items()
                 if color or depth}
        robot = []
        for mesh, root_from_asset in self._robot_visuals:
            instance = _camera.Instance()
            instance.mesh = mesh
            instance.transform = (world_root @ root_from_asset).astype(np.float32)
            robot.append(instance)
        with self._capture_locks[sensor_id]:
            with self._lock:
                self._scene.instances = self._static + robot
                raw = {}
                for eye, view in views.items():
                    intrinsics = camera["intrinsics"][eye]
                    raw[eye] = self._host.capture(self._scene, view, self._appearance, time,
                                                  intrinsics.width, intrinsics.height, *wanted[eye])
            frames = {}
            # The right eye has no depth and so no random state; processing the left eye last
            # keeps a failed call from consuming the left eye's noise stream.
            for eye in ("right", "left"):
                if eye in raw:
                    jpeg = wanted[eye][0] and jpeg_quality is not None
                    frames[eye] = camera["processors"][eye].process(
                        camera["intrinsics"][eye], camera["noise"], raw[eye], jpeg=jpeg,
                        quality=DEFAULT_JPEG_QUALITY if jpeg_quality is None else jpeg_quality)
        return CameraCapture(sensor_id, frames.get("left"), frames.get("right"))

    def info(self, sensor_id: str, eye: str = "left") -> dict[str, Any]:
        """Rectified calibration: K and P row-major; right P carries Tx = -fx * baseline."""
        camera = self._get_camera(sensor_id)
        if eye not in EYES:
            raise ValueError(f"eye must be one of {EYES}")
        k = camera["intrinsics"][eye]
        tx = -k.fx * camera["baseline_m"] if eye == "right" else 0.0
        return {"width": k.width, "height": k.height,
                "k": [k.fx, 0.0, k.cx, 0.0, k.fy, k.cy, 0.0, 0.0, 1.0],
                "p": [k.fx, 0.0, k.cx, tx, 0.0, k.fy, k.cy, 0.0, 0.0, 0.0, 1.0, 0.0]}

    def reset(self, seed: int) -> None:
        """Restore every eye's noise stream from an independent seed derived from ``seed``."""
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 1 << 64:
            raise ValueError("seed must be an integer in [0, 2**64)")
        # Lock every complete acquisition before resetting any processor. Captures acquire
        # only their own camera lock, then the GL lock, so this order cannot form a cycle.
        with ExitStack() as locks:
            for lock in self._capture_locks.values():
                locks.enter_context(lock)
            for sensor_id, camera in self._cameras.items():
                for eye in EYES:
                    camera["seeds"][eye] = derive_seed(seed, sensor_id, eye)
                    camera["processors"][eye].reset(camera["seeds"][eye])
            self._seed = seed

    def describe(self) -> dict[str, Any]:
        """JSON-ready effective configuration for a run record; a detached deep copy."""
        with ExitStack() as locks:
            for lock in self._capture_locks.values():
                locks.enter_context(lock)
            return self._describe_locked()

    def _describe_locked(self) -> dict[str, Any]:
        cameras = {}
        for sensor_id, camera in self._cameras.items():
            noise = camera["noise"]
            cameras[sensor_id] = {
                "frame": camera["frame"], "right_frame": camera["right_frame"],
                "outputs": sorted(camera["outputs"]), "baseline_m": camera["baseline_m"],
                "left": self.info(sensor_id, "left"), "right": self.info(sensor_id, "right"),
                "depth_noise": {name: getattr(noise, name)
                                for name in ("enabled", "min_range", "max_range",
                                             *_NOISE.values())},
                "seeds": camera["seeds"],
            }
        return copy.deepcopy({
            "device": self._host.device,
            "clipping_m": {"near": NEAR_PLANE_M, "far": FAR_PLANE_M},
            "sensor_noise": self._sensor_noise,
            "seed": {"policy": SEED_POLICY, "scenario_seed": self._seed},
            "cameras": cameras,
            "appearance": self._appearance_record,
            "scene": {"pool": self._pool_record,
                      "static_instances": len(self._static),
                      "robot_visuals": len(self._robot_visuals),
                      "files": list(self._files.values()),
                      "cutouts": self._cutouts,
                      "props_without_visuals": self._unrendered},
        })

    # ------------------------------------------------------------------ construction

    @staticmethod
    def _select(robot: Mapping[str, Any],
                sensor_ids: Iterable[str] | None) -> dict[str, dict[str, Any]]:
        sensors = {item["id"]: item for item in robot["sensors"]}
        if sensor_ids is None:
            chosen = [key for key, item in sensors.items()
                      if item["type"] == "stereo_camera" and item.get("enabled", True)]
        else:
            if isinstance(sensor_ids, str):
                raise TypeError("sensor_ids must be an iterable of sensor ids, not a string")
            chosen = list(dict.fromkeys(sensor_ids))
        if not chosen:
            raise ValueError("no enabled stereo_camera sensors selected")
        result = {}
        for key in chosen:
            item = sensors.get(key)
            if item is None:
                raise ValueError(f"unknown robot sensor '{key}'")
            if item["type"] != "stereo_camera":
                raise ValueError(f"sensor '{key}' is a {item['type']}, not a stereo_camera")
            if not item.get("enabled", True):
                raise ValueError(f"sensor '{key}' is disabled")
            result[key] = {"source": item}
        return result

    @staticmethod
    def _edge(item: Mapping[str, Any]) -> _camera.FixedFrame:
        edge = _camera.FixedFrame()
        edge.parent, edge.child, edge.pose = item["parent"], item["child"], _placed(item)
        return edge

    def _configure(self, camera: dict[str, Any]) -> None:
        item = camera["source"]
        parameters = item["parameters"]
        width, height = parameters["resolution_px"]
        camera["intrinsics"] = {}
        for eye in EYES:
            k = _camera.Intrinsics()
            k.width, k.height = width, height
            values = parameters[f"intrinsics_{eye}"]
            k.fx, k.fy, k.cx, k.cy = values["fx"], values["fy"], values["cx"], values["cy"]
            k.near_plane, k.far_plane = NEAR_PLANE_M, FAR_PLANE_M
            k.projection()  # validates against the native calibration invariants
            camera["intrinsics"][eye] = k
        depth = parameters["depth"]
        noise = _camera.DepthNoise()
        noise.enabled = self._sensor_noise
        noise.min_range, noise.max_range = depth["min_range_m"], depth["max_range_m"]
        for key, name in _NOISE.items():
            setattr(noise, name, depth["noise"][key])
        right = parameters.get("right_frame")
        if right is None and "rgb_right" in parameters["outputs"]:
            raise ValueError(f"sensor '{item['id']}': rgb_right requires right_frame")
        camera.update(
            outputs=frozenset(parameters["outputs"]), noise=noise, frame=item["frame"],
            right_frame=right, baseline_m=parameters["baseline_m"],
            processors={eye: _camera.Processor() for eye in EYES}, seeds={},
            eyes={"left": self._frames.from_root(item["frame"])})
        if right is not None:
            camera["eyes"]["right"] = self._frames.from_root(right)

    @staticmethod
    def _appearance_from(pool: Mapping[str, Any]) -> tuple[_camera.Appearance, dict[str, Any]]:
        missing = [key for key in ("water_optics", "lighting") if key not in pool]
        if missing:
            raise ValueError(f"pool '{pool['id']}' lacks camera appearance: {', '.join(missing)}")
        optics, lighting = pool["water_optics"], pool["lighting"]
        water = _camera.WaterOptics()
        if "tint_rgb" in optics:
            water.tint = optics["tint_rgb"]
        water.absorption = optics["absorption_per_m_rgb"]
        water.scattering = optics["scattering"]
        water.distance_scale = optics["distance_scale"]
        water.distance_power = optics["distance_power"]
        water.clear_distance = optics["clear_distance_m"]
        appearance = _camera.Appearance()
        appearance.water = water
        appearance.outdoor = lighting["profile"] == "outdoor"
        appearance.direct_light = lighting["direct_light"]
        appearance.ambient_light = lighting["ambient_light"]
        appearance.sun_azimuth = lighting["sun_azimuth_deg"]
        appearance.sun_elevation = lighting["sun_elevation_deg"]
        appearance.glare = lighting["glare"]
        effective = appearance.water
        record = {
            "water_tint_rgb": effective.tint.tolist(),
            "water_absorption_per_m_rgb": effective.absorption.tolist(),
            "water_scattering": effective.scattering,
            "water_distance_scale": effective.distance_scale,
            "water_distance_power": effective.distance_power,
            "water_clear_distance_m": effective.clear_distance,
            "profile": lighting["profile"],
            **{name: getattr(appearance, name)
               for name in ("direct_light", "ambient_light", "sun_azimuth", "sun_elevation",
                            "glare", "caustics", "exposure", "surface", "shadows",
                            "reflections")},
        }
        return appearance, record

    @staticmethod
    def _pool_scene(pool: Mapping[str, Any],
                    placement: Mapping[str, Any]) -> tuple[_camera.Scene, dict[str, Any]]:
        if pool["type"] != "rectangular_pool":
            raise ValueError(f"pool type '{pool['type']}' has no camera scene")
        p = pool["parameters"]
        x, y, z = placement["position_m"]
        geometry = _camera.PoolGeometry()
        geometry.dimensions = [p["length_m"], p["width_m"], p["depth_m"]]
        # Native world placement: placement height raises the water level (pack_runtime).
        geometry.water_level = p["water_level_m"] + z
        geometry.deck_height = p["deck_height_m"]
        geometry.local_to_world = _upright({"position_m": [x, y, 0.0],
                                            "yaw_deg": placement["yaw_deg"]}).matrix()
        record = {"dimensions_m": [p["length_m"], p["width_m"], p["depth_m"]],
                  "water_level_world_m": geometry.water_level,
                  "deck_height_m": p["deck_height_m"],
                  "placement": {"position_m": [x, y, z], "yaw_deg": placement["yaw_deg"]}}
        return _camera.pool_scene(geometry), record

    def _task_instances(self) -> list[_camera.Instance]:
        resolved = self._resolved
        placements = {item["task"]: item for item in resolved.scenario["task_placements"]}
        result = []
        for task in resolved.task_definitions:
            world_task = _upright(placements[task["id"]])
            frames = {"task": _camera.Pose()}
            frames.update({item["id"]: _placed(item) for item in task["frames"]})
            regions = {item["id"]: item for item in task["regions"]}
            for prop in task["props"]:
                where = f"task '{task['id']}' prop '{prop['id']}'"
                if prop["type"] in ("rigid_body", "contact_world"):
                    # Moving props need per-capture poses; recorded as unrendered for now.
                    self._unrendered.append(f"{task['id']}/{prop['id']}")
                    continue
                if prop["type"] != "static_body":
                    raise ValueError(f"{where}: prop type '{prop['type']}' has no camera visuals")
                parameters = prop["parameters"]
                visuals = parameters.get("visuals", [])
                cutouts = parameters.get("cutouts")
                if not visuals:
                    if cutouts:
                        raise ValueError(f"{where}: cutouts declared without visuals")
                    self._unrendered.append(f"{task['id']}/{prop['id']}")
                    continue
                panel = self._panel(where, cutouts, regions) if cutouts else None
                counts = [0] * (len(panel[0]) if panel else 0)
                for visual in visuals:
                    task_asset = frames[visual["frame"]].compose(_placed(visual))
                    mesh = self._mesh("tasks", visual["asset"], visual.get("texture"))
                    if panel is not None:
                        mesh, selected = _camera.perforate_mesh(
                            mesh, task_asset.matrix().astype(np.float32), *panel,
                            PANEL_TOLERANCE_M)
                        counts = [a + b for a, b in zip(counts, selected)]
                    instance = _camera.Instance()
                    instance.mesh = mesh
                    instance.transform = world_task.compose(task_asset).matrix().astype(
                        np.float32)
                    # CLEAR/EMISSIVE/radiance need a native module built with them; older builds keep
                    # the mesh's own material (the C++ PackScene is the runtime authority).
                    material = visual.get("material", "asset")
                    kind = getattr(_camera.SurfaceMaterial, material.upper(), None)
                    if material != "asset" and kind is not None:
                        instance.material = kind
                    if material == "emissive" and hasattr(instance, "radiance"):
                        instance.radiance = float(visual.get("radiance", 60.0))
                        instance.casts_shadow = False
                    indicator = visual.get("indicator")
                    if indicator is not None:
                        # Initial state (reset); the latch is not part of the static scene.
                        color = regions[indicator["region"]]["parameters"]["indicator"]["initial"]
                        instance.tint = [*indicator["color_rgb"][color], 1.0]
                    result.append(instance)
                if panel is not None:
                    empty = [x for x, n in zip(panel[0], counts) if n == 0]
                    if empty:
                        raise ValueError(f"{where}: no visual geometry on cutout faces {empty}")
                    self._cutouts.append({"task": task["id"], "prop": prop["id"],
                                          "region": cutouts["region"],
                                          "faces_local_x_m": list(panel[0]),
                                          "face_triangles": counts})
        return result

    @staticmethod
    def _panel(where: str, cutouts: Mapping[str, Any],
               regions: Mapping[str, Any]
               ) -> tuple[list[float], float, list[tuple[float, float, float]]]:
        region = regions[cutouts["region"]]["parameters"]
        if region["plane"]["axis"] != "x":
            raise ValueError(f"{where}: cutout faces are local x offsets; region plane axis is "
                             f"'{region['plane']['axis']}'")
        holes = [(float(item["uv"][0]), float(item["uv"][1]), float(item["radius_uv"]))
                 for item in region["holes"]]
        return list(cutouts["faces_local_x_m"]), region["half_size_m"], holes

    def _mesh(self, role: str, asset_id: str, texture_id: str | None = None) -> _camera.Mesh:
        """Load a declared mesh after verifying it and every texture the renderer will open.

        ``texture_id`` (a visual's ``texture``) names a declared PNG asset of the same pack that
        replaces the diffuse texture of the mesh's textured submeshes for this placement.
        """
        key = (role, asset_id, texture_id)
        if key not in self._meshes:
            declared = {item["id"]: item for item in self._pack(role)["assets"]}
            item = declared.get(asset_id)
            if item is None:
                raise ValueError(f"{role} pack: unknown asset '{asset_id}'")
            path = self._verify(role, self._root(role) / item["path"], "mesh", asset_id)
            mesh = _camera.load_mesh(path)
            for dependency in mesh.dependencies:
                self._verify(role, dependency, "importer_dependency", asset_id)
            for texture in mesh.textures:
                self._verify(role, texture, "texture", asset_id)
            if texture_id is not None:
                override = declared.get(texture_id)
                if override is None:
                    raise ValueError(f"{role} pack: unknown texture asset '{texture_id}'")
                mesh = mesh.with_texture(self._verify(
                    role, self._root(role) / override["path"], "texture", asset_id))
            self._meshes[key] = mesh
        return self._meshes[key]

    def _verify(self, role: str, path: Path, kind: str, user: str) -> Path:
        """Require a present, declared, unchanged file inside the owning pack folder."""
        root = self._root(role)
        target = Path(path).resolve()
        what = f"{role} pack: {kind} '{Path(path).name}' used by asset '{user}'"
        if root not in target.parents:
            raise ValueError(f"{what} escapes the pack")
        declared = next((item for item in self._pack(role)["assets"]
                         if (root / item["path"]).resolve() == target), None)
        if declared is None:
            raise ValueError(f"{what} is not a declared pack asset")
        if declared["status"] != "present":
            raise ValueError(f"{what} is declared {declared['status']}")
        try:
            digest = hashlib.sha256(target.read_bytes()).hexdigest()
        except OSError as error:
            raise ValueError(f"{what} is unreadable: {error}") from error
        if digest != declared["sha256"]:
            raise ValueError(f"{what} sha256 mismatch")
        record = self._files.setdefault((role, declared["id"]), {
            "pack": role, "id": declared["id"], "kind": kind, "path": declared["path"],
            "sha256": digest, "used_by": []})
        if user not in record["used_by"]:
            record["used_by"].append(user)
        return target

    def _pack(self, role: str) -> Mapping[str, Any]:
        return {"robot": self._resolved.robot, "tasks": self._resolved.tasks}[role]

    def _root(self, role: str) -> Path:
        return (self._resolved.path.parent / Path(self._resolved.scenario[role])).resolve()

    # ------------------------------------------------------------------ helpers

    def _get_camera(self, sensor_id: str) -> dict[str, Any]:
        camera = self._cameras.get(sensor_id)
        if camera is None:
            raise ValueError(f"sensor '{sensor_id}' is not a selected camera")
        return camera

    @staticmethod
    def _view(camera: Mapping[str, Any], eye: str, root: _camera.Pose) -> _camera.View:
        world_eye = root.compose(camera["eyes"][eye])
        view = _camera.View()
        view.view = _camera.optical_view(world_eye)
        view.projection = camera["intrinsics"][eye].projection()
        view.eye = world_eye.translation.astype(np.float32)
        return view
