"""Cross-language contract against a separately built C++ runner."""

import csv
import io
import math
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np
import robotics_platform as rp


@unittest.skipUnless(os.environ.get("RP_REFERENCE_RUNNER"), "C++ runner not supplied")
class ReferenceTests(unittest.TestCase):
    def test_complete_profile_matches_cpp_trajectory_and_sensor_fields(self) -> None:
        scenario_path = rp.example_scenario()
        with tempfile.TemporaryDirectory() as directory:
            samples_path = Path(directory) / "sensors.csv"
            native = subprocess.run(
                [
                    os.environ["RP_REFERENCE_RUNNER"],
                    str(scenario_path),
                    "--sensors",
                    str(samples_path),
                ],
                check=True,
                capture_output=True,
                text=True,
                timeout=60,
            )
            expected = list(csv.DictReader(io.StringIO(native.stdout)))
            records = list(csv.DictReader(io.StringIO(samples_path.read_text())))
        scenario = rp.load_scenario(scenario_path)
        runtime = scenario.create_runtime()
        streams = (
            runtime.imu_stream("imu"),
            runtime.fog_stream("fog"),
            runtime.dvl_stream("dvl"),
            runtime.pressure_stream("pressure"),
        )
        schedule = {command.tick: command.forces for command in scenario.commands}
        observed: dict[tuple[str, int, str], tuple[float, int, int]] = {}
        for tick in range(scenario.ticks + 1):
            state = runtime.observe()
            row = expected[tick]
            self.assertEqual(state.tick, int(row["tick"]))
            self.assertEqual(state.elapsed_ns, int(row["time_ns"]))
            np.testing.assert_allclose(
                state.body.position,
                [float(row[k]) for k in ("x_m", "y_m", "z_m")],
                rtol=1e-12,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                state.body.orientation_wxyz,
                [float(row[k]) for k in ("qw", "qx", "qy", "qz")],
                rtol=1e-12,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                state.body.linear_velocity,
                [float(row[k]) for k in ("u_m_s", "v_m_s", "w_m_s")],
                rtol=1e-12,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                state.body.angular_velocity,
                [float(row[k]) for k in ("p_rad_s", "q_rad_s", "r_rad_s")],
                rtol=1e-12,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                state.thruster_forces,
                [float(row[f"thrust_{i}_n"]) for i in range(4)],
                rtol=1e-12,
                atol=1e-12,
            )
            for stream in streams:
                for sample in stream.drain():
                    value = sample.value
                    assert value is not None
                    fields: dict[str, float] = {}
                    matrices: dict[str, np.typing.NDArray[np.float64]] = {}
                    vectors: dict[str, np.typing.NDArray[np.float64]] = {}
                    if isinstance(value, rp.ImuReading):
                        vectors = {
                            "specific_force": value.specific_force,
                            "angular_velocity": value.angular_velocity,
                        }
                        matrices = {
                            "force_covariance": value.force_covariance,
                            "angular_covariance": value.angular_covariance,
                        }
                    elif isinstance(value, rp.FogReading):
                        fields = {
                            f"angular_rate.{i}": float(rate)
                            for i, rate in enumerate(value.angular_rates)
                        }
                        matrices = {"angular_covariance": value.covariance}
                    elif isinstance(value, rp.DvlReading):
                        vectors = {"bottom_relative_velocity": value.bottom_relative_velocity}
                        matrices = {"velocity_covariance": value.covariance}
                        fields = {"bottom_distance": value.bottom_distance}
                    else:
                        fields = {
                            "absolute_pressure": value.absolute_pressure,
                            "pressure_variance": value.pressure_variance,
                            "depth": value.depth,
                            "depth_variance": value.depth_variance,
                        }
                    for name, vector in vectors.items():
                        for axis, component in zip("xyz", vector):
                            fields[f"{name}.{axis}"] = float(component)
                    for name, matrix in matrices.items():
                        for i in range(matrix.shape[0]):
                            for j in range(matrix.shape[1]):
                                fields[f"{name}.{i}.{j}"] = float(matrix[i, j])
                    for name, number in fields.items():
                        observed[(sample.header.device_id, sample.header.sequence, name)] = (
                            number,
                            sample.header.acquired_ns,
                            sample.header.delivered_ns,
                        )
            if tick == scenario.ticks:
                break
            if tick in schedule:
                runtime.command(schedule[tick])
            runtime.advance()
        self.assertEqual(len(observed), len(records))
        for record in records:
            self.assertEqual(record["valid"], "1")
            key = (record["device"], int(record["sequence"]), record["field"])
            actual, acquired, delivered = observed[key]
            self.assertTrue(
                math.isclose(actual, float(record["value"]), rel_tol=1e-12, abs_tol=1e-12), key
            )
            self.assertEqual(
                (acquired, delivered), (int(record["acquired_ns"]), int(record["delivered_ns"]))
            )


if __name__ == "__main__":
    unittest.main()
