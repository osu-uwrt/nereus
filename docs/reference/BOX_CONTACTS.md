# Static box contact reference

The static box contact solver ports the original separating-axis test,
contact point selection, positional correction, coupled-mass impulse, restitution
and friction response. It accepts ordered body-local and static world proxy lists.
It contains no task, robot, pool, ROS or renderer names. Static world geometry is
prepared once; resolving a state uses fixed-size temporary matrices without heap
allocation. Plant selects this solver explicitly through `ContactModel::BoxScene`;
`SpherePool` remains the default and `Disabled` supports contact-free experiments.

`tools/capture_contact_reference.py` reads pinned simulator revision
`07647eebe706f96ea7b76db3cc9802735a146698` through `git show`. It compiles the original
collisionBox implementation and marine mass kernel, and extracts the actual
`handleCollisions` and `computeCollision` method bodies. Only the optional dynamic
task-contact call is removed; the logging macro is disabled. The wrapper normalizes
state quaternions exactly as the original `state2quat` utility does.

```sh
python3 tools/capture_contact_reference.py /path/to/riptide_simulator
```

The checked `tests/fixtures/legacy_box_contacts.csv` contains 11 single-resolution
cases, with source/driver/output hashes alongside it. Cases cover floor/wall
penetration, compound proxies, rotated body/world geometry, local proxy rotation,
coupled inverse mass, nonunit integration-stage quaternion, isolated separating
contact, empty scene and disjoint nonempty scene. The separating case must correct
position without applying an impulse. Numerical state comparisons use 2e-12 absolute
tolerance on this local build. Proxy order and SAT tie-breaking remain significant.

Coordinates: body proxy center/orientation are relative to COM; world proxy poses
are in the simulation world frame. Inputs require positive dimensions, valid poses,
unique IDs within each ordered list, nonnegative friction, and restitution in [0,1].
The caller supplies physically validated state and positive-definite inverse mass.
Nonunit but valid state quaternions are normalized for contact geometry/impulses;
the raw state quaternion remains unchanged by resolution, matching the original.

Robot profiles own body boxes; world profiles own static world boxes. Scenario
`contacts.model` chooses `disabled`, `sphere_pool`, or `box_scene`. Sphere radius
and initial containment apply only to the sphere model. Box scenes permit initial
intersections and resolve them during advancement. See `content/examples/contact_pool.yaml`.

Box advancement preserves half actuator step → pre-contact → raw RK4 endpoint →
post-contact → quaternion normalization → second half actuator step. Full reset
has no retained contact history. The capture tool also extracts the original RK4
expression and records 50 whole steps for each of the 11 cases in
`legacy_box_steps.csv`. These use zero propulsion/current/displaced volume; the
stage-forcing fixtures provide complementary coverage. GCC 11.4 `-O0` and `-O2`
builds of the original itself diverge at case 10, ticks 48–50 (maximum state-field
difference about 1.05e-4), due to rounding near discrete contact decisions. Both
original captures are retained with compiler flags. The test requires all 550
states to match one complete candidate within 1e-9; it never mixes candidates per
field or widens the tolerance. On this machine the new Release matches optimized
original output and the new ASan/Debug matches unoptimized output. Dynamic task/prop contacts
remain separate runtime-owned work and are not established by these fixtures.

BoxScene kinematics expose the post-impulse free-motion acceleration, not impact
acceleration integrated over a sensor interval. The existing DVL floor query still
uses the separately placed finite Pool floor: arbitrary collision boxes do not automatically
become sensor-query geometry.

Work remains proportional to body-proxy count times world-proxy count. The 4,096
item limit per list bounds storage; it is not a real-time workload guarantee.
Measure the representative Talos/course scene before choosing further acceleration.
