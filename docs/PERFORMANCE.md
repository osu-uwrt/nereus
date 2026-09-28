# Performance acceptance and measurement

The user requires practical operation on mid-grade hardware and selected the
current machine as the provisional baseline. Preserve the Fossen-style 6-DOF
rigid-body/added-mass/Coriolis/damping/hydrostatic model; optimize measured costs
without silently changing the physical behavior or acquisition schedule.

The replacement gate is real-time operation at the declared physics/sensor rates
with the same Talos/course/renderer settings as the original reference, including
animated thrusters, LEDs, cameras, mechanisms and task interactions. The physics
clock must not become viewer-driven. Report frame/sensor latency and backlog,
not just an average observer frame rate. Actual full-scene targets/comparisons
remain pending original/new fixed-workload measurements.

## Provisional headless benchmark

```sh
cmake -S . -B build/benchmark -DCMAKE_BUILD_TYPE=Release \
  -DRP_BUILD_BENCHMARKS=ON -DBUILD_TESTING=OFF
cmake --build build/benchmark --parallel 2
python3 tools/record_benchmark.py build/benchmark/robotics-benchmark \
  content/examples/profile_pool.yaml build/headless-performance.json
```

The benchmark runs three warmups and then 1,000 repetitions of the bundled
three-second scenario. It measures 1.5 million 2ms ticks per mode: plant alone,
then plant plus scheduled IMU/FOG/DVL/pressure and per-tick consumption. Seed,
command schedule and starting state repeat each run. Final trajectory checksums
must match. Runtime construction, profile parsing, CSV output, graphics, ROS,
competition tasks and camera generation are outside the timed interval.

Reported tick percentiles include command application, advancement, and sensor
consumption. Reported throughput also includes measurement-loop overhead. These
are local wall-clock measurements affected by scheduling, power state and load,
not worst-case execution guarantees. The snapshot values/checksum ensure the work
remains observable. Benchmark storage is bounded to ten million measured ticks.
The recording helper is for bundled repository scenarios and build-tree executables;
it records content and executable hashes, revision/patch identity, compiler/build
settings and selected machine information.

The initial record is
[synthetic-headless-baseline.json](reference/performance/synthetic-headless-baseline.json).
It is evidence for the synthetic headless slice only. It must not be advertised as
Talos/full-stack throughput or cross-hardware performance.

The [compound contact record](reference/performance/compound-headless-baseline.json)
uses `content/examples/contact_pool.yaml`: one hull proxy, five finite pool boxes,
wall contact, and four sensor families. On the same machine, plant p99 was 3.25
microseconds; plant plus sensors p99 was 4.75 microseconds and completed 3,000
simulated seconds in 5.262 measured seconds. This is still a synthetic workload,
not Talos, dynamic props, rendering or full-stack acceptance.

## Repeat as the replacement grows

- Compound contacts: representative floor/wall/corner impacts and task geometry;
  preserve collision response while measuring pair queries and solve cost.
- Native Talos: eight thrusters, actual inertia/added mass, mounts and acquisition
  schedules, both free motion and contact-heavy paths.
- Rendering: the original pool, Talos mesh/rotors/LEDs, 2026 props, identical viewport,
  water/shadow/bloom settings and camera products. Measure CPU scene preparation,
  GPU work, observer presentation and camera acquisition separately.
- Full robot stack: declared detector/backend and entire mission, including payloads,
  claw/props, score events and task resets. Record sustained real-time factor, tail
  latencies, CPU/GPU/memory use and any backlog, rather than masking skipped work.

Performance data informs implementation and configurable presentation quality.
Any sensor/physical-fidelity setting change must be explicit and compared against
its declared model, never hidden as an optimization. Broad RViz features and
speculative framework work remain behind simulator-replacement acceptance.
