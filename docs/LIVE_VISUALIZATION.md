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
