"""Pack-to-native translation must preserve the validated plant and sensor behavior."""

import copy
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import robotics_platform as rp
from robotics_platform.pack_runtime import create_runtime
from robotics_platform.packs import resolve_scenario
from test_packs_fixtures import write_generic_packs

ROOT = Path(__file__).resolve().parents[2]
PACKS = ROOT / "content/packs"
SENSORS = ["imu", "fog", "dvl", "depth"]


def resolved_data():
    # The factory consumes resolved values; tests edit this copy before building a runtime.
    resolved = resolve_scenario(PACKS / "scenarios/talos_uwrt")
    return SimpleNamespace(
        robot=copy.deepcopy(resolved.robot),
        pool=copy.deepcopy(resolved.pool),
        scenario=copy.deepcopy(resolved.scenario),
        task_definitions=copy.deepcopy(resolved.task_definitions),
    )


class GenericPackRuntimeTests(unittest.TestCase):
    def test_generic_robot_uses_its_own_parameters_and_sensor_schedule(self):
        with tempfile.TemporaryDirectory() as directory:
            resolved = resolve_scenario(write_generic_packs(Path(directory)))
            result = create_runtime(resolved)
            self.assertEqual(len(result.parameters.thrusters), 4)
            self.assertEqual(result.parameters.body.mass, 20)
            self.assertEqual(result.deferred_sensor_ids, ())
            result.runtime.command([3, 3, 0, 0])
            for _ in range(50):
                result.runtime.advance()
                for stream in result.streams.values():
                    stream.drain()
            self.assertEqual(result.streams["imu"].stats.acquired, 10)
            self.assertEqual(result.streams["altitude"].stats.acquired, 2)


class PackRuntimeTests(unittest.TestCase):
    def test_resolved_scenario_constructs_native_runtime(self):
        resolved = resolve_scenario(PACKS / "scenarios/talos_uwrt")
        result = create_runtime(resolved, sensor_ids=SENSORS)
        self.assertEqual(result.runtime.advance().tick, 1)
        self.assertEqual(result.deferred_sensor_ids, ("ffc", "dfc"))

    @unittest.skipUnless(
        (ROOT / "content/examples/talos_navigation_pool.yaml").is_file(),
        "requires repository native reference fixture",
    )
    def test_matches_existing_native_talos_under_commands(self):
        reference = rp.load_scenario(ROOT / "content/examples/talos_navigation_pool.yaml")
        data = resolved_data()
        # The pack writes the pool corner as exact zero; the earlier importer
        # retained a 1e-15 m rotation residual. Compare identical numeric inputs
        # because initial wall contact is sensitive to last-bit point ordering.
        data.scenario["pool_placement"]["position_m"][:2] = list(
            reference.plant.pool.origin_xy_world
        )
        result = create_runtime(data, sensor_ids=SENSORS)
        old = reference.create_runtime()
        old_streams = {
            "imu": old.ahrs_stream("imu"),
            "fog": old.fog_stream("fog"),
            "dvl": old.velocity_stream("dvl"),
            "depth": old.altitude_stream("depth"),
        }
        counts = dict.fromkeys(SENSORS, 0)
        for tick in range(500):
            if tick % 100 == 0:
                force = np.array([4, 4, 2, 2, -2, -2, 4, 4]) * (1 if tick < 300 else -1)
                result.runtime.command(force)
                old.command(force)
            actual, expected = result.runtime.advance(), old.advance()
            np.testing.assert_allclose(
                actual.body.position, expected.body.position, atol=1e-11, rtol=0
            )
            np.testing.assert_allclose(
                actual.body.orientation_wxyz, expected.body.orientation_wxyz, atol=1e-11, rtol=0
            )
            np.testing.assert_allclose(
                actual.thruster_forces, expected.thruster_forces, atol=1e-12, rtol=0
            )
            for name, stream in result.streams.items():
                samples, prior = stream.drain(), old_streams[name].drain()
                self.assertEqual(len(samples), len(prior))
                counts[name] += len(samples)
                for a, b in zip(samples, prior):
                    self.assertEqual(a.header.acquired_ns, b.header.acquired_ns)
                    self.assertEqual(a.header.sequence, b.header.sequence)
                    if name == "imu":
                        np.testing.assert_allclose(
                            a.value.inertial.specific_force,
                            b.value.inertial.specific_force,
                            atol=1e-10,
                            rtol=0,
                        )
                        np.testing.assert_allclose(
                            a.value.attitude.orientation_wxyz,
                            b.value.attitude.orientation_wxyz,
                            atol=1e-11,
                            rtol=0,
                        )
                    elif name == "fog":
                        np.testing.assert_allclose(
                            a.value.angular_rates, b.value.angular_rates, atol=1e-11, rtol=0
                        )
                    elif name == "dvl":
                        np.testing.assert_allclose(
                            a.value.reference_relative_velocity,
                            b.value.reference_relative_velocity,
                            atol=1e-11,
                            rtol=0,
                        )
                    else:
                        self.assertAlmostEqual(
                            a.value.target_world_z, b.value.target_world_z, places=10
                        )
        self.assertEqual(counts, {"imu": 50, "fog": 500, "dvl": 8, "depth": 20})
        self.assertEqual(result.deferred_sensor_ids, ("ffc", "dfc"))

    def test_requested_unsupported_sensor_fails_instead_of_disappearing(self):
        with self.assertRaisesRegex(ValueError, "ffc.*stereo_camera"):
            create_runtime(resolved_data())

    def test_selection_rejects_unknown_duplicate_and_disabled_ids(self):
        for ids in (["missing"], ["fog", "fog"]):
            with self.assertRaisesRegex(ValueError, "unique ids"):
                create_runtime(resolved_data(), sensor_ids=ids)
        data = resolved_data()
        next(s for s in data.robot["sensors"] if s["id"] == "fog")["enabled"] = False
        with self.assertRaisesRegex(ValueError, "disabled"):
            create_runtime(data, sensor_ids=["fog"])

    def test_reference_frame_initial_pose_preserves_com_pose(self):
        data = resolved_data()
        a = create_runtime(data, sensor_ids=[])
        initial = data.scenario["initial"]
        initial["frame"] = "base_link"
        initial["position_m"] = [0.017, -0.01, -1.042]
        b = create_runtime(data, sensor_ids=[])
        np.testing.assert_allclose(a.initial.position, b.initial.position, atol=1e-15)
        np.testing.assert_allclose(
            a.initial.orientation_wxyz, b.initial.orientation_wxyz, atol=1e-15
        )

    def test_sensor_queue_policy_is_applied(self):
        data = resolved_data()
        sensor = next(s for s in data.robot["sensors"] if s["id"] == "fog")
        sensor.update(capacity=1, overflow="drop_oldest")
        runtime = create_runtime(data, sensor_ids=["fog"])
        runtime.runtime.advance(10)
        self.assertEqual(runtime.streams["fog"].stats.dropped_delivered, 9)
        self.assertEqual(len(runtime.streams["fog"].drain()), 1)

    def test_contact_selection_is_honored(self):
        for name, model in (
            ("disabled", rp.ContactModel.DISABLED),
            ("sphere_pool", rp.ContactModel.SPHERE_POOL),
            ("box_scene", rp.ContactModel.BOX_SCENE),
        ):
            with self.subTest(model=name):
                data = resolved_data()
                data.scenario["contacts"]["model"] = name
                data.scenario["initial"]["position_m"] = [2, 0, -1]
                result = create_runtime(data, sensor_ids=[])
                self.assertEqual(result.parameters.contacts.model, model)
                result.runtime.advance()

    def test_native_pose_operations_validate_and_invert(self):
        pose = rp.Pose()
        pose.translation = [2, 3, 4]
        pose.orientation_wxyz = [np.sqrt(0.5), 0, 0, np.sqrt(0.5)]
        np.testing.assert_allclose(pose.apply([1, 0, 0]), [2, 4, 4], atol=1e-15)
        np.testing.assert_allclose(
            pose.inverse().apply(pose.apply([1, 2, 3])), [1, 2, 3], atol=1e-14
        )
        invalid = rp.Pose()
        invalid.orientation_wxyz = [2, 0, 0, 0]
        for operation in (
            invalid.inverse,
            lambda: pose.compose(invalid),
            lambda: invalid.apply([0, 0, 0]),
        ):
            with self.assertRaises(ValueError):
                operation()
        with self.assertRaises(ValueError):
            pose.apply([np.inf, 0, 0])


if __name__ == "__main__":
    unittest.main()
