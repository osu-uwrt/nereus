# Static box contact reference

The initial private box contact solver ports the original separating-axis test,
contact point selection, positional correction, coupled-mass impulse, restitution
and friction response. It accepts ordered body-local and static world proxy lists.
It contains no task, robot, pool, ROS or renderer names. Static world geometry is
prepared once; resolving a state uses fixed-size temporary matrices without heap
allocation. This increment is a tested numerical component, not yet the selected
Plant contact model.

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

Next integration must select contact policy explicitly, keep geometry ownership in
robot/world content, permit initial depenetration for box scenes, and preserve the
original pre/post-RK4 contact ordering and quaternion normalization timing. Do not
apply existing sphere-pool containment validation to disabled or box contact modes.
Dynamic props/task contacts and whole-step contact trajectories require additional
fixtures; these single-response fixtures do not establish those behaviors.

Work remains proportional to body-proxy count times world-proxy count. The 4,096
item limit per list bounds storage; it is not a real-time workload guarantee.
Measure the representative Talos/course scene before choosing further acceleration.
