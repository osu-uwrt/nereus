# Tasks and scoring

A task set is one folder, `content/packs/tasks/<id>/`: one file per task (geometry, what can happen), a
`tasks.yaml` that lists them and says how the run is scored and shown, and the meshes. The RoboSub 2026 set in
`tasks/robosub_2026/` covers every feature below; copy from it.

Geometry and events are data. Turning events into points is code: a rules class selected by name (see
[Scoring rules](#scoring-rules)).

## A task file

Everything in a task file is in the task's **local frame**; the [scenario](course.md) places that frame in the
pool with `task_placements`.

```yaml
kind: task
id: gate
description: Start gate with two role halves.
frames:                               # named points in the task, e.g. what mapping estimates
- {id: gate_repair, position_m: [0, -0.75, 0.5], orientation_wxyz: [1, 0, 0, 0]}
props:                                # physical things
- id: gate_structure
  type: static_body
  parameters:
    collision_boxes:                  # the robot hits these
    - {id: top_bar, size_m: [0.076, 3.05, 0.076], center_m: [0, 0, 0.61], orientation_wxyz: [1, 0, 0, 0]}
    visuals:                          # what the viewer and cameras draw
    - {asset: gate_mesh, frame: task, position_m: [0, -1.5, 0], orientation_wxyz: [1, 0, 0, 0]}
regions:                              # judged volumes
- id: gate_opening
  type: rectangular_portal
  parameters:
    plane: {axis: x, offset_m: 0.0}
    bounds_local: {abs_y_lt_m: 1.5, z_lt_m: 0.75}
    approach_radius_m: 3.0
    # ...see robosub_2026/gate.yaml for the full set
events:                               # what the runtime reports
- {id: forward_pass, type: pass_through, parameters: {region: gate_opening, from_side: positive, to_side: negative}}
- {id: structure_contact, type: contact, parameters: {prop: gate_structure, with: robot}}
scoring: []
```

### Props

| Type | Use |
| --- | --- |
| `static_body` | fixed structures: `collision_boxes`, `visuals`, optional `collision_meshes` (triangle meshes for props to rest on) and `cutouts` (holes in a panel mesh) |
| `rigid_body` | loose objects the claw can pick up: collision mesh, visual mesh, mass, volume, friction, `expected_region` |
| `contact_world` | physics settings for a task with `rigid_body` props (Bullet), and which mechanism type grasps them |

### Regions and events

| Region | Events it produces |
| --- | --- |
| `rectangular_portal` (gate, slalom rows) | `pass_through` from one side to the other |
| `perforated_panel` (torpedo board) | `hit` with an outcome per hole |
| `open_crate` (bins) | dropped markers landing in it |
| `proximity_target` (magnet lights) | `activate` when the robot's probe point dwells close enough |
| `surface` (octagon) | `surface_reached` / `surface_lost`, `facing_reached` / `facing_lost` |
| `turn_zone` | whole yaw turns made in place |
| box regions (baskets) | `drop_into` for props coming to rest inside or elsewhere |

`contact` fires when the robot touches a named prop. Each event can limit the data it reports with `emits`.

## tasks.yaml

```yaml
kind: tasks
id: robosub_2026
assets:                               # every mesh/texture the task files use
- {id: gate_mesh, path: assets/gate/model.dae}
tasks: [gate.yaml, torpedo.yaml, table.yaml]
requires:                             # the robot must have these mechanisms
- {task: table, mechanism_type: claw, min_count: 1}
run_options:                          # chosen in the viewer before "Start run"
- {key: role, type: choice, choices: [repair, rescue], default: repair}
- {key: heading_coin, type: bool, default: true}
- {key: bin_vinyl1_class, type: choice, choices: [blood, fire], default: blood, fixed: true}   # set by the scenario
scoring_rules:
- {name: robosub_2026, parameters: {...points and thresholds...}}
score_rows:                           # scorecard rows, in order
- {key: gate, label: Gate passage / heading / role / style}
outcome_counters: [success, wrong_target, blocked, miss]
run_messages: {start: "Run started; pass the gate first", stop: "Run stopped"}
ui:                                   # how the viewer presents the run
  title: RoboSub 2026 scorecard
  run_options:                        # labels and widgets for run_options
  - {key: role, label: Coin-flip role, type: choice, default: repair,
     choices: [{value: repair, label: Survey & Repair}, {value: rescue, label: Search & Rescue}]}
  status_fields: [{key: role, label: Role (gate side taken)}]      # values from the rules' describe()
  score_fields: [{key: basket_count, label: Objects in both baskets}]
  manual_adjustment: true             # the Add/Clear score adjustment
  focus: [Course, Vehicle, gate, torpedo, table]                   # viewer focus list
```

### Fixed run options

A `fixed: true` option is set by the scenario's `run.options` when it resolves and holds for the whole session:
a run Start can't change it. A task file can bind a value to a fixed choice option, so one task file serves
every layout of the course. Bindings are allowed for a visual's `texture` and an `open_crate` region's `class`:

```yaml
# bins.yaml: the crate's class is the choice itself; its vinyl texture maps each choice to an asset
- {asset: bin_vinyl_mesh, frame: bin_vinyl1, position_m: [0, 0, 0], orientation_wxyz: [1, 0, 0, 0],
   texture: {option: bin_vinyl1_class, values: {blood: bin_vinyl_blood_texture, fire: bin_vinyl_fire_texture}}}
- id: bin_vinyl1
  type: open_crate
  parameters: {frame: bin_vinyl1, class: {option: bin_vinyl1_class}, ...}
```

`values` must map every choice. Resolving writes the plain value in, so the simulator, cameras and datasets
never see a binding.

## Scoring rules

`scoring_rules[].name` names a rules class compiled into the simulator (`extensions/rules/`, registered in
`extensions/rules/registry.cpp`). A rules class gets the run state and the ordered events and returns score
rows; `describe()` fills the scorecard's status fields and `feed()` the event list. `parameters` is passed through
unchanged, so point values and thresholds stay in YAML.

To score a new competition, add a class next to `robosub_2026/`, register it under a new name and point
`name:` at it.
