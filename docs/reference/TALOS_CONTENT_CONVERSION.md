# Talos content conversion map

This is the reviewed offline-conversion specification. The bounded native
[dynamics/mount pack](TALOS_PHYSICS_PACK.md) is delivered; full sensors, mechanisms,
visual assets and task/stack acceptance remain open.
Pin simulator `07647eebe706f96ea7b76db3cc9802735a146698` and vehicle-description repo
`src/riptide_core/riptide_descriptions` at `7f37bdd62ab90113844137a23806c89b23a8ab9b`.
The platform must load checked native content after both source trees are absent.

## Independent content ownership

| Owner | Source relative to simulator, except noted vehicle repo |
|---|---|
| Robot manifest | c_simulator/robots/talos/robot.yaml |
| Mass, CAD-relative frames and ordered thrusters | vehicle repo: config/talos.yaml |
| Plant inertia, hydrodynamics and actuator calibration | c_simulator/robots/talos/config/hydrodynamics.yaml |
| Sensor reporting defaults | vehicle repo: config/simulator.yaml; c_simulator/robots/talos/config/sensors.yaml |
| Water and pool dimensions | c_simulator/worlds/competition_pool.yaml |
| Scenario selections | c_simulator/tasks/2026/scenarios/default.yaml |
| Pool/map and course placements | c_simulator/tasks/2026/config/mapping.yaml |
| Robot collision proxies | c_simulator/collision_files/robots/talos.urdf |
| Robot mechanisms | c_simulator/robots/talos/config/equipment.yaml |
| Task interactions and scoring | c_simulator/tasks/2026/competition.yaml and config/* |

Hydrodynamics source SHA256:
`bf80b5d6215a7d81fe1017b52bed31befb1749a7bec2feeceaa977909a9f0c88`.
Vehicle source SHA256:
`7f0c5a703e3974401a23ce316dbedde1327e974b16105d39a97c7a64837bd664`.
Use the manifest-selected hydro file, not similarly named historical configurations.
`profiles.py` applies world water properties over hydro defaults. Controller feedforward,
legacy damping/volume in simulator.yaml, and nominal controller properties are not
substitutes for resolved plant values.

## Numerical mapping

Default mass is 31.998 kg; CAD COM is `[-.157,.040,-.048]` m and CAD base_link
`[-.140,.030,-.090]` m. Thus COM-to-base translation is `[.017,-.010,-.042]` m.
If a plant-only COM override is selected, let delta = plant_COM - nominal_COM:
subtract delta from hydro damping/COB centers. The original does not apply a
parallel-axis inertia correction for that override; preserve/document that choice.

Rigid inertia (row-major):
`[1.6315,-.0599,-.0330, -.0599,.7341,.0427, -.0330,.0427,1.6332]`.
Copy full added-mass and linear damping matrices, quadratic damping vector, and
actuator response from the resolved hydro profile. Volume is `.0323621667640687`
m^3; buoyancy radii `[.175,.415,.275]` m; default COB `[.00775,0,.010]` m relative
to COM and damping center `[0,0,0]`.

Thruster sequence is VUS,VUP,HUS,HUP,HLS,HLP,VLS,VLP. Vehicle `type` does not flip
force sign in the original physics. Position is CAD mount minus plant COM; direction
is `Rz(yaw)*Ry(pitch)*Rx(roll)*UnitX`. Preserve original radians `.785` and `1.571`
exactly; replacing them with pi/4 and pi/2 changes allocation.

COM-relative positions, in sequence:
`[.023,-.409,.290]`, `[.023,.398,.290]`, `[.017,-.300,.189]`,
`[.017,.288,.189]`, `[.017,-.300,-.093]`, `[.017,.288,-.093]`,
`[.023,-.412,-.196]`, `[.023,.400,-.196]`.
All default actuator delays are .1 s, rise .08 s, fall .06 s, slew 300 N/s,
forward/reverse limits 28 N, deadband zero, scales/efficiency one, disk radius .05 m,
and command timeout .5 s.

World dimensions are 50×22.86×2.1336 m; water level zero, density 998.2 kg/m^3,
deck height .305288888 m. Mean current and oscillation amplitude are zero;
frequency is .1 Hz. Current vectors already use world/map axes.

## Legacy frame conventions that must stay explicit

`map_origin_pool: [19.5136,0,90]` defines
`mapToPool = Translation(19.5136,0,0)*Rz(90 degrees)` and poolToMap is its inverse.
Apply poolToMap to collision/render geometry, independently of hydrodynamic current.
Course yaw values use degrees and parent frames; vehicle mount angles use radians.

The absent scenario start pose resolves to `[0,0,-1,0,0,0]`. Startup assigns that
translation directly to COM; the placement service instead accepts base_link and
subtracts its rotated COM offset. Do not silently treat both inputs as base poses.

URDF collision origins are copied directly into COM-following proxies by the old
loader, unlike CAD-relative mounts. Preserve their effective COM-local values:
chassis size `[.35,.83,.55]`, center `[0,0,.07]`; poker size `[.2,.05,.05]`, center
`[.23,-.2,-.1]`. Subtracting CAD COM again would change old contact behavior.

## Prerequisites for full native Talos content acceptance

- Selectable compound box collision proxies and collision-disabled reference mode.
- Transformed pool geometry with finite walls/deck, independently selected from tasks.
- Named COM/CAD/base/mount frame data for rendering, acquisition and adapters.
- Task-owned collision geometry and independent mechanisms/scoring composition.
- Distinct placement/task/full-reset semantics.

A coefficients-only Talos example with reduced contacts must be labeled as such.
It cannot stand in for the default competition configuration or its acceptance test.

The companion [visual content audit](VISUAL_CONTENT_CONVERSION.md) identifies the
exact body/rotor/LED/mechanism/course resources, transform equivalence, import
repairs and provenance requirements for replacing the original appearance.
