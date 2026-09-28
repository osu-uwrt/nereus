"""Run the installed synthetic profile; no ROS, viewer, or source checkout required."""

import json

import robotics_platform as rp


def main() -> None:
    scenario = rp.load_scenario(rp.example_scenario())
    runtime = scenario.create_runtime()
    imu = runtime.imu_stream("imu")
    fog = runtime.fog_stream("fog")
    dvl = runtime.dvl_stream("dvl")
    pressure = runtime.pressure_stream("pressure")
    commands = {command.tick: command.forces for command in scenario.commands}
    for tick in range(scenario.ticks):
        if tick in commands:
            runtime.command(commands[tick])
        runtime.advance()
        for stream in (imu, fog, dvl, pressure):
            stream.drain()  # Explicit consumption; latest remains available.
    latest = pressure.latest()
    assert latest is not None and latest.value is not None
    print(
        json.dumps(
            {
                "tick": runtime.observe().tick,
                "depth_m": latest.value.depth,
                "pressure_samples": pressure.stats.delivered,
            }
        )
    )
    runtime.reset(scenario.initial, scenario.seed)
    assert runtime.observe().elapsed_ns == 0 and pressure.latest() is None


if __name__ == "__main__":
    main()
