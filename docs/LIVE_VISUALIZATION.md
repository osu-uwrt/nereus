# Live visualization boundary

`RoboticsPlatform::visualization` provides `LivePoseSource`, a neutral source for
one timestamped rigid-body pose stream and its world-to-body transform history.
It requires neither simulation nor graphics nor middleware. This is the initial
live payload; images, clouds, joints and aligned multi-source scenes remain planned.

The producer publishes a value with an explicit generation and nanosecond clock.
Time must increase within a generation; increase the generation before rewinding.
Old generations and duplicate/reversed timestamps are rejected with a counter.
A producer generation change clears queued samples and the presentation history.

One producer thread calls `publish`; one presentation thread calls
`snapshot`, `disconnect`, and `reconnect`. Stop/join callers before destruction.
Queue allocation happens at construction. A short mutex protects bounded copies;
no I/O, rendering, or history reconstruction occurs while holding it. This is not
a lock-free or hard-real-time API. Overflow replaces oldest pending samples;
history is separately bounded. Both losses are reported on snapshots, along with
stale-update rejection. Retained snapshots own immutable data.

Disconnect affects presentation only. Accepted producer updates still advance the
latest value. Reconnect starts a fresh display generation seeded from that value,
without replaying obsolete history. Null snapshot data means disconnected or
waiting for the first update. Live sources expose no playback capability.

The optional `RoboticsPlatform::simulation_view` target supplies
`publishSimulationPose(source, snapshot)`. It receives a simulation observation by
const reference and maps COM position/orientation and simulation time to the source.
It owns no runtime, cannot step or command it, and does not invent a base-link or
CAD-frame offset. Application composition owns the runtime, scheduling, control,
and any required frame conversion.

Build this adapter without graphics:

```sh
python3 tools/check.py --preset simulation-view --install-check
```

`examples/simulation_view` is a downstream installed-package consumer. The native
contracts compare every trajectory state and noisy IMU reading with and without
publication across fast/slow polling, dropped updates, disconnect, reset, and
reconnect. This proves transport independence for those exercised cases; it does
not establish Talos physics fidelity or provide a connected simulation application.

## Composed simulation application

```sh
cmake --preset simulator-viewer
cmake --build --preset simulator-viewer
build/simulator-viewer/robotics-sim-view content/examples/profile_pool.yaml
```

The optional `robotics-sim-view` application composes the same desktop interface
with a live source and a worker-owned scenario runtime. It runs the configured
finite command schedule at wall-paced fixed steps, consumes sensor samples every
tick, and retains the completed state for inspection. Falling behind wall time
never drops physics steps. Closing the application cancels and joins execution;
worker exceptions reach the owner and result in a failed exit status.

The viewer can disconnect/reconnect, change displays, save/reopen its workspace,
or open a local recording while the scenario runs. Playback affects recordings
only. Simulation pause/reset/placement controls are a future separate provider.
Saved simulation bindings refer to the startup scenario and reopen in the composed
application started with that same scenario. The standalone viewer reports an
unregistered simulation source rather than loading simulator code implicitly.

`--hidden --frames N --screenshot OUTPUT.ppm` exercises the desktop path for
validation. It requires a graphics display/context; this is not the future
headless camera-rendering backend. The application currently displays pose/lines;
water rendering, robot meshes, and task content are separate outstanding work.

Static source-owned frames can now accompany the live body history through
`LivePoseOptions.fixed_frames`. The simulation application passes the scenario's
resolved COM-rooted mounts; neither the live source nor the shared spatial library
requires simulation. See [FRAMES.md](FRAMES.md) for construction and reset semantics.
