"""Bounded camera work outside the physics thread; poses and stamps are acquisition data."""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .core import Publication
from .images import ImageWriter, compile_image_writer, compile_info_writer
from .mapping import MappingError


@dataclass(frozen=True)
class _Acquisition:
    sensor: str
    native_ns: int
    ros_ns: int
    revision: int
    position: tuple[float, ...]
    orientation: tuple[float, ...]


class CameraBridge:
    """One worker owns capture, formatting and publication; the stepping thread queues poses.

    Each camera has a bounded pending queue (default one, drop_oldest) plus at most one
    in-flight capture across the whole worker. Placement invalidates pending captures but
    preserves noise state. A full reset requests new seeds and acquisition schedules.
    Shutdown joins the worker before its publisher/node or EGL host is destroyed.
    """

    def __init__(self, resolved: Any, provider: Any,
                 publish: Callable[[list[Publication]], None]) -> None:
        self.provider, self.publish = provider, publish
        self.sensors = {s["id"]: s for s in resolved.robot["sensors"]
                        if s["id"] in provider.sensor_ids}
        if len(self.sensors) != len(provider.sensor_ids) or any(
                s["type"] != "stereo_camera" or not s.get("enabled", True)
                for s in self.sensors.values()):
            raise MappingError("camera provider must select unique enabled robot cameras")
        self.streams: dict[str, list[tuple[str, str, Any]]] = {s: [] for s in self.sensors}
        self.quality: dict[str, int | None] = dict.fromkeys(self.sensors)
        for stream in resolved.bridge["streams"]:
            endpoint = stream["native"]
            if not endpoint.startswith("sensor:"):
                continue
            sensor, _, output = endpoint.removeprefix("sensor:").partition(".")
            if sensor not in self.sensors:
                continue
            config = self.sensors[sensor]
            if output not in config["parameters"]["outputs"]:
                raise MappingError(f"camera {sensor!r} has no configured output {output!r}")
            if abs(stream["rate_hz"] * config["period_ns"] / 1e9 - 1) > 1e-6:
                raise MappingError(f"camera stream {stream['id']!r} rate differs from its sensor")
            frame = (config["parameters"]["right_frame"] if output.endswith("right")
                     else config["frame"])
            expected = resolved.bridge.get("frame_names", {}).get(frame)
            if expected is not None and stream["frame_id"] != expected:
                raise MappingError(f"camera stream {stream['id']!r} differs from frame_names")
            writer = (compile_info_writer(stream) if output.startswith("camera_info")
                      else compile_image_writer(stream))
            if isinstance(writer, ImageWriter) and writer.jpeg_quality is not None:
                quality = self.quality[sensor]
                if quality is not None and quality != writer.jpeg_quality:
                    raise MappingError(f"camera {sensor!r} streams require conflicting JPEG qualities")
                self.quality[sensor] = writer.jpeg_quality
            self.streams[sensor].append((stream["id"], output, writer))
        self.stream_ids = frozenset(stream for streams in self.streams.values()
                                    for stream, _, _ in streams)
        for sensor, config in self.sensors.items():
            if config.get("latency_ns", 0) != 0:
                raise MappingError(f"camera {sensor!r}: nonzero delivery latency is not implemented")
            if config.get("capacity", 1) < 1 or config.get("overflow", "drop_oldest") not in {
                    "fail", "drop_oldest"}:
                raise MappingError(f"camera {sensor!r}: invalid pending queue policy")
        self._condition = threading.Condition()
        self._publication_lock = threading.Lock()
        self._pending: deque[_Acquisition] = deque()
        self._next = dict.fromkeys(self.sensors, 0)
        self._revision = 0
        self._seed: int | None = None
        self._failure: BaseException | None = None
        self._stopping = False
        self._thread: threading.Thread | None = None
        self._stats = {sensor: {"requested": 0, "captured": 0, "published": 0,
                               "dropped_pending": 0, "discarded_stale": 0,
                               "capture_wall_ns": 0} for sensor in self.sensors}

    def start(self) -> None:
        with self._condition:
            if self._thread is not None or self._stopping:
                raise RuntimeError("camera worker cannot be started twice or after close")
            self._thread = threading.Thread(target=self._run, name="camera-publisher", daemon=False)
            self._thread.start()

    def acquire(self, snapshot: Any, ros_ns: int) -> None:
        """Called by the physics owner after a tick; never waits for capture or publication."""
        now = int(snapshot.elapsed_ns)
        with self._condition:
            self._raise_failure()
            if self._stopping:
                raise RuntimeError("camera worker is closed")
            for sensor, config in self.sensors.items():
                if now < self._next[sensor]:
                    continue
                period = int(config["period_ns"])
                self._next[sensor] = (now // period + 1) * period
                stats = self._stats[sensor]
                stats["requested"] += 1
                queued = [i for i, job in enumerate(self._pending) if job.sensor == sensor]
                body = snapshot.body
                job = _Acquisition(sensor, now, ros_ns, self._revision,
                                   tuple(float(v) for v in body.position),
                                   tuple(float(v) for v in body.orientation_wxyz))
                if len(queued) >= config.get("capacity", 1):
                    if config.get("overflow", "drop_oldest") == "fail":
                        raise RuntimeError(f"camera {sensor!r} pending queue is full")
                    # Keep this sensor's queue positions: removing/re-appending in pack order
                    # would repeatedly move a slower camera behind the first camera.
                    for older, newer in zip(queued, queued[1:]):
                        self._pending[older] = self._pending[newer]
                    self._pending[queued[-1]] = job
                    stats["dropped_pending"] += 1
                else:
                    self._pending.append(job)
            self._condition.notify()

    def invalidate(self, *, seed: int | None = None) -> None:
        """Discard pre-placement work; a provided seed also resets camera schedules/noise.

        An operator placement/reset waits for an active publication to finish, never
        for rendering. Regular acquire() does not take this publication barrier.
        """
        with self._publication_lock, self._condition:
            self._revision += 1
            for job in self._pending:
                self._stats[job.sensor]["discarded_stale"] += 1
            self._pending.clear()
            if seed is not None:
                self._seed = seed
                self._next = dict.fromkeys(self.sensors, 0)
            self._condition.notify()

    def close(self) -> None:
        with self._condition:
            self._stopping = True
            self._revision += 1
            for job in self._pending:
                self._stats[job.sensor]["discarded_stale"] += 1
            self._pending.clear()
            self._condition.notify()
        if self._thread is not None:
            self._thread.join()
        self._raise_failure()

    def stats(self) -> dict[str, dict[str, int]]:
        with self._condition:
            return {key: dict(value) for key, value in self._stats.items()}

    def describe(self) -> dict[str, Any]:
        return {"capture": self.provider.describe(),
                "worker": {"count": 1, "in_flight_capacity": 1,
                           "pending": {sensor: {"capacity": config.get("capacity", 1),
                                                "overflow": config.get("overflow", "drop_oldest")}
                                       for sensor, config in self.sensors.items()},
                           "jpeg_quality": dict(self.quality)}}

    def _raise_failure(self) -> None:
        if self._failure is not None:
            raise RuntimeError(f"camera worker failed: {self._failure}") from self._failure

    def _run(self) -> None:
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._stopping or self._seed is not None
                                             or bool(self._pending))
                    if self._stopping:
                        return
                    seed, self._seed = self._seed, None
                    job = self._pending.popleft() if self._pending else None
                if seed is not None:
                    self.provider.reset(seed)
                if job is None:
                    continue
                started = time.monotonic_ns()
                frame = self.provider.capture(job.sensor, job.position, job.orientation,
                                              job.native_ns / 1e9,
                                              jpeg_quality=self.quality[job.sensor])
                publications = []
                for stream, output, writer in self.streams[job.sensor]:
                    eye = "right" if output.endswith("right") else "left"
                    if output.startswith("camera_info"):
                        values = {"sample": {"time": job.ros_ns},
                                  "info": self.provider.info(job.sensor, eye)}
                        message = writer(values)
                    else:
                        pixels = frame.right if eye == "right" else frame.left
                        if pixels is None:
                            raise RuntimeError(f"camera {job.sensor!r} omitted {output!r}")
                        message = writer(pixels, job.ros_ns)
                    publications.append(Publication(stream, message))
                with self._condition:
                    stats = self._stats[job.sensor]
                    stats["captured"] += 1
                    stats["capture_wall_ns"] += time.monotonic_ns() - started
                with self._publication_lock:
                    with self._condition:
                        stale = job.revision != self._revision or self._stopping
                        if stale:
                            stats["discarded_stale"] += 1
                    if not stale:
                        self.publish(publications)
                        with self._condition:
                            stats["published"] += len(publications)
        except BaseException as error:
            with self._condition:
                self._failure = error
                self._stopping = True
                self._condition.notify_all()
