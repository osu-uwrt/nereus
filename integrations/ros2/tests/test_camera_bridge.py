"""Camera work stays bounded and reset/placement cannot deliver old acquisitions."""

import copy
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import numpy as np

try:
    from robotics_platform_ros.camera_bridge import CameraBridge
    from robotics_platform_ros.mapping import MappingError
except ImportError as error:
    raise unittest.SkipTest(f"camera bridge dependencies unavailable: {error}") from None


class Provider:
    sensor_ids = ("cam",)

    def __init__(self):
        self.entered, self.release = threading.Event(), threading.Event()
        self.calls, self.seeds = [], []
        self.fail = False

    def capture(self, sensor, position, orientation, time_s, *, jpeg_quality):
        self.calls.append((sensor, position, orientation, time_s, jpeg_quality))
        self.entered.set()
        if self.fail:
            raise ValueError("capture failed")
        if not self.release.wait(5):
            raise TimeoutError("test capture was not released")
        frame = SimpleNamespace(width=3, height=2, depth=np.ones((2, 3), np.float32))
        return SimpleNamespace(left=frame, right=None)

    def reset(self, seed):
        self.seeds.append(seed)


class ParallelProvider(Provider):
    sensor_ids = ("cam", "other")

    def __init__(self):
        super().__init__()
        self.entered = {sensor: threading.Event() for sensor in self.sensor_ids}
        self.release = {sensor: threading.Event() for sensor in self.sensor_ids}
        self.reset_entered = threading.Event()
        self.reset_release = threading.Event()
        self.reset_release.set()
        self.active = set()
        self.lock = threading.Lock()

    def capture(self, sensor, position, orientation, time_s, *, jpeg_quality):
        with self.lock:
            if sensor in self.active:
                raise AssertionError("same camera acquired concurrently")
            self.active.add(sensor)
            self.calls.append((sensor, position, orientation, time_s, jpeg_quality))
        self.entered[sensor].set()
        if not self.release[sensor].wait(5):
            raise TimeoutError("test capture was not released")
        with self.lock:
            self.active.remove(sensor)
        frame = SimpleNamespace(width=3, height=2, depth=np.ones((2, 3), np.float32))
        return SimpleNamespace(left=frame, right=None)

    def reset(self, seed):
        with self.lock:
            if self.active:
                raise AssertionError("reset overlaps an old capture")
            self.seeds.append(seed)
        self.reset_entered.set()
        if not self.reset_release.wait(5):
            raise TimeoutError("test reset was not released")

    def unblock(self):
        for event in self.release.values():
            event.set()
        self.reset_release.set()


def resolved(overflow="drop_oldest"):
    config = {"id": "cam", "type": "stereo_camera", "frame": "optical", "enabled": True,
              "period_ns": 20_000_000, "capacity": 1, "overflow": overflow,
              "parameters": {"outputs": ["depth_left"]}}
    stream = {"id": "depth", "native": "sensor:cam.depth_left", "direction": "publish",
              "frame_id": "auv/optical", "message_type": "sensor_msgs/msg/Image", "rate_hz": 50,
              "image": {"encoding": "32FC1"}, "fields": {"header.stamp": {"from": "sample.time"}}}
    return SimpleNamespace(robot={"sensors": [config]},
                           bridge={"streams": [stream], "frame_names": {"optical": "auv/optical"}})


def two_cameras():
    config = resolved()
    other = copy.deepcopy(config.robot["sensors"][0])
    other["id"] = "other"
    config.robot["sensors"].append(other)
    stream = copy.deepcopy(config.bridge["streams"][0])
    stream.update(id="other_depth", native="sensor:other.depth_left")
    config.bridge["streams"].append(stream)
    return config


def snapshot(ns, x=1):
    return SimpleNamespace(elapsed_ns=ns, body=SimpleNamespace(
        position=np.array([x, 2., 3.]), orientation_wxyz=np.array([1., 0., 0., 0.])))


class CameraBridgeTests(unittest.TestCase):
    def test_each_camera_has_one_inflight_and_its_own_latest_pending_capture(self):
        provider = ParallelProvider()
        done = threading.Semaphore(0)
        worker = CameraBridge(two_cameras(), provider, lambda _: done.release())
        self.addCleanup(worker.close)
        self.addCleanup(provider.unblock)
        worker.start()
        worker.acquire(snapshot(2_000_000), 0)
        for event in provider.entered.values():
            self.assertTrue(event.wait(2))
        for ns in (20_000_000, 40_000_000, 60_000_000):
            worker.acquire(snapshot(ns), ns)
        for stats in worker.stats().values():
            self.assertEqual(stats["dropped_pending"], 2)
        provider.unblock()
        for _ in range(4):
            self.assertTrue(done.acquire(timeout=2))
        worker.close()
        for sensor in provider.sensor_ids:
            self.assertEqual([call[3] for call in provider.calls if call[0] == sensor], [.002, .06])

    def test_reset_waits_for_both_cameras_and_coalesces_new_seed_before_capture(self):
        provider = ParallelProvider()
        publications = []
        done = threading.Semaphore(0)

        def publish(values):
            publications.extend(values)
            done.release()

        worker = CameraBridge(two_cameras(), provider, publish)
        self.addCleanup(worker.close)
        self.addCleanup(provider.unblock)
        worker.start()
        worker.acquire(snapshot(2_000_000), 102_000_000)
        for event in provider.entered.values():
            self.assertTrue(event.wait(2))
        worker.invalidate(seed=98)
        worker.acquire(snapshot(2_000_000), 202_000_000)
        provider.release["cam"].set()
        self.assertFalse(provider.reset_entered.wait(.05))
        provider.reset_release.clear()
        provider.release["other"].set()
        self.assertTrue(provider.reset_entered.wait(2))
        worker.invalidate(seed=99)
        worker.acquire(snapshot(2_000_000), 302_000_000)
        self.assertEqual(len(provider.calls), 2)  # Neither camera can pass a pending reset.
        provider.reset_release.set()
        for _ in provider.sensor_ids:
            self.assertTrue(done.acquire(timeout=2))
        worker.close()
        self.assertEqual(provider.seeds, [98, 99])
        self.assertEqual(len(publications), 2)
        self.assertEqual({p.message.header.stamp.nanosec for p in publications}, {302_000_000})
        for stats in worker.stats().values():
            self.assertEqual(stats["discarded_stale"], 2)

    def test_close_joins_every_active_camera_and_discards_their_outputs(self):
        provider = ParallelProvider()
        publications = []
        worker = CameraBridge(two_cameras(), provider, publications.extend)
        self.addCleanup(worker.close)
        self.addCleanup(provider.unblock)
        worker.start()
        worker.acquire(snapshot(2_000_000), 0)
        for event in provider.entered.values():
            self.assertTrue(event.wait(2))
        with ThreadPoolExecutor(1) as closer:
            closing = closer.submit(worker.close)
            try:
                with worker._condition:
                    self.assertTrue(worker._condition.wait_for(lambda: worker._stopping, timeout=2))
                provider.release["cam"].set()
                self.assertFalse(closing.done())
            finally:
                provider.unblock()
            closing.result(timeout=2)
        self.assertEqual(publications, [])
        self.assertTrue(all(not thread.is_alive() for thread in worker._threads))

    def test_placement_invalidates_both_workers_without_reseeding(self):
        provider = ParallelProvider()
        publications = []
        done = threading.Semaphore(0)

        def publish(values):
            publications.extend(values)
            done.release()

        worker = CameraBridge(two_cameras(), provider, publish)
        self.addCleanup(worker.close)
        self.addCleanup(provider.unblock)
        worker.start()
        worker.acquire(snapshot(2_000_000), 102_000_000)
        for event in provider.entered.values():
            self.assertTrue(event.wait(2))
        worker.acquire(snapshot(20_000_000), 120_000_000)
        worker.invalidate()
        worker.acquire(snapshot(40_000_000), 140_000_000)
        provider.unblock()
        for _ in provider.sensor_ids:
            self.assertTrue(done.acquire(timeout=2))
        worker.close()
        self.assertEqual(provider.seeds, [])
        self.assertEqual(len(publications), 2)
        self.assertEqual({p.message.header.stamp.nanosec for p in publications}, {140_000_000})
        for stats in worker.stats().values():
            self.assertEqual(stats["discarded_stale"], 2)

    def test_publication_failure_stops_other_worker_and_reaches_owner(self):
        provider = ParallelProvider()
        calls = []

        def publish(values):
            calls.append(values)
            raise ValueError("publication failed")

        worker = CameraBridge(two_cameras(), provider, publish)
        worker.start()
        try:
            worker.acquire(snapshot(2_000_000), 0)
            for event in provider.entered.values():
                self.assertTrue(event.wait(2))
            provider.release["cam"].set()
            with worker._condition:
                self.assertTrue(worker._condition.wait_for(lambda: worker._failure is not None,
                                                          timeout=2))
            with self.assertRaisesRegex(RuntimeError, "publication failed"):
                worker.acquire(snapshot(20_000_000), 0)
        finally:
            provider.unblock()
            with self.assertRaisesRegex(RuntimeError, "publication failed"):
                worker.close()
        self.assertEqual(len(calls), 1)

    def worker(self, *, config=None):
        provider = Provider()
        publications = []
        delivered = threading.Semaphore(0)

        def publish(values):
            publications.extend(values)
            delivered.release()

        worker = CameraBridge(config or resolved(), provider, publish)
        self.addCleanup(worker.close)
        self.addCleanup(provider.release.set)
        worker.start()
        return worker, provider, publications, delivered

    def test_slow_capture_does_not_block_acquisition_and_pending_queue_keeps_latest(self):
        worker, provider, publications, delivered = self.worker()
        worker.acquire(snapshot(2_000_000), 102_000_000)
        self.assertTrue(provider.entered.wait(2))
        with ThreadPoolExecutor(1) as owner:
            for ns in (20_000_000, 40_000_000, 60_000_000):
                state = snapshot(ns, x=ns)
                owner.submit(worker.acquire, state, 100_000_000 + ns).result(timeout=2)
                state.body.position[:] = -10  # The queued pose must already be detached.
        stats = worker.stats()["cam"]
        self.assertEqual(stats["requested"], 4)
        self.assertEqual(stats["dropped_pending"], 2)
        provider.release.set()
        self.assertTrue(delivered.acquire(timeout=2))
        self.assertTrue(delivered.acquire(timeout=2))
        worker.close()
        self.assertEqual(len(provider.calls), 2)
        self.assertEqual(provider.calls[1][1][0], 60_000_000)
        self.assertEqual(publications[-1].message.header.stamp.nanosec, 160_000_000)

    def test_placement_discards_inflight_and_pending_without_resetting_noise(self):
        worker, provider, publications, delivered = self.worker()
        worker.acquire(snapshot(2_000_000), 102_000_000)
        self.assertTrue(provider.entered.wait(2))
        worker.acquire(snapshot(20_000_000), 120_000_000)
        worker.invalidate()
        worker.acquire(snapshot(40_000_000, x=7), 140_000_000)
        provider.release.set()
        self.assertTrue(delivered.acquire(timeout=2))
        worker.close()
        self.assertEqual(len(publications), 1)
        self.assertEqual(provider.seeds, [])
        self.assertEqual(provider.calls[-1][1][0], 7)
        self.assertEqual(worker.stats()["cam"]["discarded_stale"], 2)

    def test_reset_reseeds_before_the_first_new_capture_and_restarts_schedule(self):
        worker, provider, publications, delivered = self.worker()
        worker.acquire(snapshot(2_000_000), 102_000_000)
        self.assertTrue(provider.entered.wait(2))
        worker.invalidate(seed=99)
        worker.acquire(snapshot(2_000_000, x=8), 202_000_000)
        provider.release.set()
        self.assertTrue(delivered.acquire(timeout=2))
        worker.close()
        self.assertEqual(provider.seeds, [99])
        self.assertEqual(len(publications), 1)
        self.assertEqual(publications[0].message.header.stamp.nanosec, 202_000_000)

    def test_fail_overflow_is_explicit_and_callback_failure_reaches_owner(self):
        worker, provider, _, _ = self.worker(config=resolved("fail"))
        worker.acquire(snapshot(2_000_000), 0)
        self.assertTrue(provider.entered.wait(2))
        worker.acquire(snapshot(20_000_000), 0)
        with self.assertRaisesRegex(RuntimeError, "queue is full"):
            worker.acquire(snapshot(40_000_000), 0)
        provider.release.set()
        worker.close()
        broken = Provider()
        broken.fail = True
        worker2 = CameraBridge(resolved(), broken, lambda _: None)
        worker2.start()
        worker2.acquire(snapshot(2_000_000), 0)
        self.assertTrue(broken.entered.wait(2))
        with self.assertRaisesRegex(RuntimeError, "capture failed"):
            worker2.close()

    def test_invalid_rate_frame_and_latency_fail_before_start(self):
        for key, value in (("rate_hz", 10), ("frame_id", "wrong")):
            config = resolved()
            config.bridge["streams"][0][key] = value
            with self.assertRaises(MappingError):
                CameraBridge(config, Provider(), lambda _: None)
        config = resolved()
        config.robot["sensors"][0]["latency_ns"] = 1
        with self.assertRaisesRegex(MappingError, "latency"):
            CameraBridge(config, Provider(), lambda _: None)


if __name__ == "__main__":
    unittest.main()
