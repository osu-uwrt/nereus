"""Public contracts tested against the installed wheel, never a source-tree import."""

import gc
import math
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np
import robotics_platform as rp


def initial() -> rp.BodyState:
    state = rp.BodyState()
    state.position = [5, 5, -2]
    return state


def passive(seed: int = 42) -> rp.Runtime:
    return rp.Runtime(rp.PlantParameters(), initial(), seed)


class RuntimeTests(unittest.TestCase):
    def test_fixed_frame_values_are_detached_and_scenario_frames_survive_loading(self) -> None:
        pose = rp.Pose()
        pose.translation = [0.2, 0.3, -0.1]
        edge = rp.FixedFrame()
        edge.parent = "center"
        edge.child = "sensor"
        edge.pose = pose
        frames = rp.FixedFrames("center", [edge])
        pose.translation = [10, 20, 30]
        edge.pose = pose
        np.testing.assert_array_equal(frames.from_root("sensor").translation, [0.2, 0.3, -0.1])
        frames.edges[0].pose = pose
        detached = frames.from_root("sensor")
        detached.translation = [5, 5, 5]
        np.testing.assert_array_equal(
            frames.lookup("sensor", "center").translation, [-0.2, -0.3, 0.1]
        )
        with self.assertRaises(ValueError):
            frames.from_root("missing")
        with self.assertRaises(ValueError):
            rp.FixedFrames("center", [edge, edge])
        scenario = rp.load_scenario(rp.example_scenario())
        self.assertEqual(scenario.body_frames.root, "com")
        np.testing.assert_array_equal(scenario.body_frames.from_root("com").translation, [0, 0, 0])

    def test_explicit_box_contacts_and_disabled_contact_world(self) -> None:
        params = rp.PlantParameters()
        params.contacts.model = rp.ContactModel.BOX_SCENE
        hull = rp.BoxProxy()
        hull.id = "hull"
        hull.size = [0.4, 0.4, 0.4]
        floor = rp.BoxProxy()
        floor.id = "floor"
        floor.size = [10, 10, 1]
        floor.center = [0, 0, -0.5]
        params.contacts.body_boxes = [hull]
        params.contacts.world_boxes = [floor]
        state = initial()
        state.position = [0, 0, 0.1]
        state.linear_velocity = [0.1, 0.2, -0.3]
        runtime = rp.Runtime(params, state)
        result = runtime.advance(10)
        self.assertGreater(result.body.position[2], 0.1)
        runtime.reset(state, 0)
        np.testing.assert_array_equal(runtime.advance(10).body.position, result.body.position)
        params.contacts.model = rp.ContactModel.DISABLED
        params.body.collision_radius = -1
        state.position = [-2, -2, -10]
        runtime = rp.Runtime(params, state)
        self.assertEqual(runtime.advance().elapsed_ns, 2_000_000)

    def test_calibrated_thruster_stop_clears_delay_without_rewinding(self) -> None:
        params = rp.PlantParameters()
        thruster = rp.Thruster()
        thruster.id = "propeller"
        thruster.delay = 0.05
        thruster.rise_time = thruster.fall_time = thruster.slew_rate = 0
        thruster.deadband = 2
        thruster.forward_scale = 0.5
        thruster.reverse_scale = 0.8
        thruster.efficiency = 0.5
        params.thrusters = [thruster]
        runtime = rp.Runtime(params, initial())
        runtime.command([10])
        runtime.advance(5)
        runtime.stop_thrusters()
        self.assertEqual(runtime.observe().elapsed_ns, 10_000_000)
        self.assertEqual(runtime.observe().generation, 0)
        self.assertEqual(runtime.advance(50).thruster_forces[0], 0)
        runtime.command([10])
        self.assertEqual(runtime.advance(50).thruster_forces[0], 2.5)

    def test_optional_wet_propeller_and_current_configuration(self) -> None:
        params = rp.PlantParameters()
        thruster = rp.Thruster()
        thruster.id = "propeller"
        thruster.delay = thruster.rise_time = thruster.fall_time = thruster.slew_rate = 0
        self.assertIsNone(thruster.propeller_radius)
        params.thrusters = [thruster]
        state = initial()
        state.position = [5, 5, 1]
        unmodulated = rp.Runtime(params, state)
        thruster.propeller_radius = 0.05
        params.thrusters = [thruster]
        immersed = rp.Runtime(params, state)
        for runtime in (unmodulated, immersed):
            runtime.command([10])
            runtime.advance(5)
        self.assertAlmostEqual(unmodulated.observe().body.linear_velocity[0], 0.01, places=12)
        self.assertEqual(immersed.observe().body.linear_velocity[0], 0)
        params.pool.current_oscillation_amplitude = [0.2, 0, 0]
        params.pool.current_oscillation_frequency = 0.7
        moving_water = rp.Runtime(params, initial())
        self.assertGreater(moving_water.advance().body.linear_velocity[0], 0)
        moving_water.reset(initial(), 0)
        self.assertEqual(moving_water.observe().elapsed_ns, 0)

    def test_programmatic_force_matches_analytical_motion(self) -> None:
        params = rp.PlantParameters()
        thruster = rp.Thruster()
        thruster.id = "surge"
        thruster.delay = thruster.rise_time = thruster.fall_time = thruster.slew_rate = 0
        params.command_timeout = 0
        params.thrusters = [thruster]
        runtime = rp.Runtime(params, initial())
        params.body.mass = 200  # Construction owns its configuration.
        runtime.command([10])
        snapshot = runtime.advance(500)
        self.assertEqual(snapshot.elapsed_ns, 1_000_000_000)
        self.assertAlmostEqual(snapshot.body.position[0], 5.5, places=11)
        self.assertAlmostEqual(snapshot.body.linear_velocity[0], 1, places=11)
        self.assertEqual(runtime.reset(initial(), 0).generation, 1)
        self.assertEqual(runtime.observe().thruster_forces[0], 0)

    def test_observation_arrays_and_profile_values_are_detached(self) -> None:
        scenario = rp.load_scenario(rp.example_scenario())
        runtime = scenario.create_runtime()
        state = runtime.observe().body
        state.position = [100, 0, 0]
        values = runtime.observe().body.position
        values[:] = 400
        self.assertEqual(runtime.observe().body.position[0], 2)
        scenario.plant.body.mass = 300
        self.assertEqual(scenario.plant.body.mass, 10)
        runtime.advance(5)
        stream = runtime.imu_stream("imu")
        sample = stream.latest()
        assert sample is not None and sample.value is not None
        force = sample.value.specific_force
        force[:] = 0
        self.assertGreater(float(np.linalg.norm(sample.value.specific_force)), 9)
        latest = stream.latest()
        assert latest is not None and latest.value is not None
        self.assertGreater(float(np.linalg.norm(latest.value.specific_force)), 9)
        del runtime, stream, sample
        gc.collect()
        self.assertEqual(force.tolist(), [0, 0, 0])

    def test_all_models_work_without_profiles(self) -> None:
        runtime = passive()
        imu = runtime.add(rp.Device("imu", "imu"), rp.Imu())
        fog = runtime.add(rp.Device("fog", "fog"), rp.Fog(axes=[[1, 0, 0], [0, 0, 1]]))
        dvl = runtime.add(rp.Device("dvl", "dvl"), rp.Dvl(rp.DvlParameters(), rp.Pool()))
        pressure = runtime.add(
            rp.Device("pressure", "pressure"),
            rp.Pressure(rp.PressureParameters(), rp.HydrostaticPressure(0)),
        )
        runtime.advance(5)
        im = imu.latest()
        fo = fog.latest()
        dv = dvl.latest()
        pr = pressure.latest()
        assert im and im.value and fo and fo.value and dv and dv.value and pr and pr.value
        np.testing.assert_allclose(im.value.specific_force, [0, 0, 9.80665], atol=1e-12)
        self.assertEqual(fo.value.angular_rates.shape, (2,))
        self.assertAlmostEqual(dv.value.bottom_distance, 3)
        self.assertAlmostEqual(pr.value.depth, 2)
        self.assertEqual(len(imu.drain()), 1)
        self.assertEqual(imu.drain(), [])
        self.assertIsNotNone(imu.latest())
        with self.assertRaises(ValueError):
            runtime.fog_stream("imu")
        with self.assertRaises(ValueError):
            runtime.imu_stream("missing")

    def test_quaternion_order_and_mount_rotation_cross_the_binding_correctly(self) -> None:
        state = initial()
        state.orientation_wxyz = [math.sqrt(0.5), 0, math.sqrt(0.5), 0]
        runtime = rp.Runtime(rp.PlantParameters(), state)
        mount = rp.Mount()
        mount.orientation_wxyz = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
        stream = runtime.add(rp.Device("rotated", "sensor"), rp.Imu(mount))
        runtime.advance(5)
        sample = stream.latest()
        assert sample and sample.value
        np.testing.assert_allclose(sample.value.specific_force, [0, 9.80665, 0], atol=1e-12)
        np.testing.assert_allclose(
            runtime.observe().body.orientation_wxyz, state.orientation_wxyz, atol=1e-12
        )

    def test_integer_nanosecond_precision_and_latency(self) -> None:
        params = rp.PlantParameters()
        params.timestep_ns = 7
        runtime = rp.Runtime(params, initial())
        stream = runtime.add(rp.Device("imu", "imu", period_ns=11, latency_ns=3), rp.Imu())
        runtime.advance(5)
        samples = stream.drain()
        self.assertEqual([s.header.scheduled_ns for s in samples], [11, 22])
        self.assertEqual([s.header.acquired_ns for s in samples], [14, 28])
        self.assertEqual([s.header.delivered_ns for s in samples], [21, 35])
        self.assertEqual(runtime.observe().elapsed_ns, 35)
        device = rp.Device("slow", "slow", period_ns=2**53 + 1)
        self.assertEqual(device.period_ns, 2**53 + 1)
        with self.assertRaises(TypeError):
            rp.Device("bad", "bad", period_ns=1.5)  # type: ignore[arg-type]

    def test_seed_reset_partition_and_polling_independence(self) -> None:
        config = rp.load_scenario(rp.example_scenario())
        a, b = config.create_runtime(), config.create_runtime()
        sa, sb = a.imu_stream("imu"), b.imu_stream("imu")
        a.advance(50)
        for _ in range(50):
            b.advance()
            sb.latest()
            b.observe()
        left, right = sa.drain(), sb.drain()
        self.assertEqual(len(left), len(right))
        for x, y in zip(left, right):
            assert x.value and y.value
            np.testing.assert_array_equal(x.value.specific_force, y.value.specific_force)
        a.reset(config.initial, config.seed)
        self.assertIsNone(sa.latest())
        a.advance(50)
        for old, replay in zip(left, sa.drain()):
            assert old.value and replay.value
            np.testing.assert_array_equal(old.value.specific_force, replay.value.specific_force)
            self.assertEqual(replay.header.generation, 1)
        changed = config.create_runtime(seed=43)
        changed.advance(5)
        other = changed.imu_stream("imu").latest()
        assert other and other.value and left[0].value
        self.assertFalse(np.array_equal(other.value.specific_force, left[0].value.specific_force))

    def test_unavailable_is_none_and_fault_requires_reset(self) -> None:
        runtime = passive()
        params = rp.DvlParameters()
        params.maximum_range = 1
        dvl = runtime.add(rp.Device("dvl", "dvl", capacity=1), rp.Dvl(params, rp.Pool()))
        runtime.advance(5)
        sample = dvl.latest()
        assert sample is not None
        self.assertIsNone(sample.value)
        self.assertEqual(sample.unavailable_reason, "bottom out of range")
        with self.assertRaises(RuntimeError):
            runtime.advance(5)
        self.assertTrue(runtime.faulted)
        self.assertFalse(dvl.active)
        self.assertIsNone(dvl.latest())
        with self.assertRaises(RuntimeError):
            runtime.advance()
        runtime.reset(initial(), 42)
        self.assertTrue(dvl.active)
        self.assertFalse(runtime.faulted)
        runtime.advance(5)
        self.assertIsNotNone(dvl.latest())

    def test_invalid_inputs_fail_before_mutation(self) -> None:
        runtime = passive()
        with self.assertRaises(ValueError):
            runtime.command([1])
        with self.assertRaises(TypeError):
            runtime.advance(-1)
        with self.assertRaises(OverflowError):
            runtime.advance(2**64 - 1)
        state = initial()
        state.position = [math.nan, 0, 0]
        with self.assertRaises(ValueError):
            runtime.reset(state, 1)
        self.assertEqual(runtime.observe().tick, 0)
        self.assertFalse(runtime.faulted)
        noise = rp.NoiseParameters()
        noise.white_stddev = [-1, 0, 0]
        with self.assertRaises(ValueError):
            rp.Imu(acceleration_noise=noise)
        state = initial()
        with self.assertRaises(TypeError):
            state.orientation_wxyz = [1, 0]
        with self.assertRaises(ValueError):
            runtime.add(rp.Device("bad", "bad", period_ns=0), rp.Imu())

    def test_runtime_destruction_invalidates_handles_without_dangling_data(self) -> None:
        runtime = passive()
        stream = runtime.add(rp.Device("imu", "imu"), rp.Imu())
        runtime.advance(5)
        sample = stream.latest()
        assert sample and sample.value
        value = sample.value.specific_force
        del runtime
        gc.collect()
        self.assertFalse(stream.active)
        self.assertIsNone(stream.latest())
        self.assertEqual(stream.drain(), [])
        np.testing.assert_allclose(value, [0, 0, 9.80665])

    def test_profile_files_can_be_removed_after_loading(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "content"
            shutil.copytree(rp.example_scenario().parent.parent, root)
            config = rp.load_scenario(root / "examples/profile_pool.yaml")
            self.assertEqual(len(config.sources), 4)
            self.assertEqual({s.model for s in config.sensors}, {"imu", "fog", "dvl", "pressure"})
        runtime = config.create_runtime()
        runtime.advance(25)
        sample = runtime.pressure_stream("pressure").latest()
        assert sample and sample.value
        self.assertAlmostEqual(sample.value.depth, 2.1, delta=0.01)


if __name__ == "__main__":
    unittest.main()
