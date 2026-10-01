# Synthetic datasets

Nereus can render labelled training images straight from the course: no sim, no viewer, no ROS. Images come
through the robot's own cameras (intrinsics, mount, resolution), are generated per task, and are labelled by a
team's label pack, then exported as YOLO detect, segment or OBB datasets.

```
task pack     parts.yaml + part-map PNGs   named pieces of the artwork and props; no team classes
label pack    labels.yaml                   parts -> classes per task, class order per model, range limit
dataset spec  dataset.yaml                  scenarios, model, per-task samplers, counts, randomization
planner       nereus-dataset plan           the three above + resolved scenarios -> job.json
renderer      nereus-dataset-render         job.json -> images, id maps, records (EGL only)
exporter      nereus-dataset export         records + label pack -> YOLO
```

The label pack is applied **at export**. Renders are team-neutral (every visible part is recorded), so a new
class, a renamed class or a dropped class is a re-export, not a re-render. The renderer only uses the label pack
to reject views: it never generates an image with a labelled object farther away than the model's range, or a
task image without its target in view.

## Parts (task pack)

`parts.yaml` sits next to the tasks pack's `tasks.yaml` (the simulator never reads it) and names the pieces of the
course a team might label. It has no classes: it is a fact of the printed course and any team can reuse it.

```yaml
kind: parts
id: robosub_2026_parts
tasks: robosub_2026                 # must be the tasks pack id
textures:                           # part maps: pixel value -> part, same size and UVs as the texture
- texture: torpedo_texture          # tasks-pack asset id of the texture
  mask: assets/torpedo/Task4_ver1_parts.png
  parts:
  - {value: 1, part: icon_fire, seed_px: [376, 378], threshold: 40}
  - {value: 5, part: ring, seed_px: [372, 679], fill: all}   # values 5..8 are the four rings: one part name
visuals:                            # whole visuals or materials of a mesh
- {task: slalom, asset: slalom_mesh, materials: {Material.001: pole_red, Material.002: pole_white}}
- {task: bins, asset: magnet_cover_mesh, frame: magnet_target1, part: magnet_cover, indicator: magnet_target1}
```

- **Texture parts**: every visual that draws the texture (from its DAE or as a `texture:` override) gets the map,
  so the four bin vinyls, both octagon sets and the table props need one entry per texture. Each value is one
  instance; several values may share a part name. `task:` limits an entry to one task's visuals.
- **Visual parts**: select visuals by `task` and `asset` (and optionally `prop`, `frame`), then either `part`
  (the whole visual) or `materials` (importer material name -> part; unlisted materials have no part).
  `split: connected` makes each connected piece of a mesh its own instance. `indicator: <region>` records that
  region's indicator colour (`red` / `green`) on the instance, for labels that depend on it.
- Part names are lower-case `[a-z0-9_]` and scoped by task.

Part-map PNGs are built from the artwork by `tools/generate_part_masks.py` (re-run it after changing a seed or a
texture; `--check` fails if a committed map is stale). A seed is a pixel on the emoji (texture pixels, top-left
origin); the tool takes the connected non-background component under it, fills holes (`fill: small` default,
`all` for rings, `none`) and paints it with the value. `threshold`, `close_px` and `background` tune the
background test per part.

## Label pack (team)

```yaml
kind: labels
id: uwrt_robosub_2026
guide: https://...                  # the human labeling guide this encodes
tasks_pack: robosub_2026            # part patterns are checked against this pack's parts.yaml
models:                             # class order = the deployed model's class ids
  ffc: {camera: ffc, max_range_m: 5.0, classes: [blood, buoy, compass, circle, fire, ...]}
  dfc: {camera: dfc, max_range_m: 3.0, classes: [bandage, blood, fire, helmet, ...]}
classes:                            # optional per-class export options
  circle: {shape: outer}            # fill holes in the instance mask (default: visible pixels)
tasks:                              # per task: class <- part patterns (* ? [..] globs)
  torpedo: {fire: [icon_fire], circle: [ring]}
  slalom:  {slalom: [pole_red]}     # pole_white is never labelled
  bins:    {magnet: {parts: [magnet_cover], when: {indicator: red}}}   # only while its LEDs are red
export:
  min_visible_px: 25                # smaller instances (at the output resolution) are dropped
```

- A model is a camera, a class list in id order and `max_range_m`: views with a labelled instance farther than
  that (median depth) are never generated.
- A class mapped in `tasks` but missing from a model's `classes` is not exported for that model (the dfc model
  has no `circle`, so the torpedo rings are unlabelled there).
- A part may map to only one class per task (indicator-gated mappings aside).
- With the torpedo board, the label pass gives the see-through hole the ring's value, so `circle` covers ring
  plus opening without `shape: outer`.

Keep the class order in step with the detector: `check-classes` compares every model with the
`<camera>_class_id_map` strings of a ROS parameter file.

```bash
nereus-dataset check-classes content/packs/labels/uwrt \
    --yolo-config ../src/riptide_perception/tensor_detector/config/yolo_orientation.yaml
```

## Dataset spec

`content/packs/datasets/<id>/dataset.yaml` is one experiment: what to render, from where, how many, how varied.

```yaml
kind: dataset
id: uwrt_ffc_2026
labels: ../../labels/uwrt           # label pack folder or file
model: ffc                          # camera + classes + range from the label pack
scenarios: [../../scenarios/talos_uwrt]   # samples go round-robin across these (one tasks pack)
seed: 1
image: {resolution_px: native, crop: center, format: jpg, jpeg_quality: 92}   # or [w, h]
robot_visuals: true                 # draw the robot (claw / hull in view)
split: {train: 0.8, val: 0.2, test: 0.0}
tasks:
  torpedo:
    count: 500
    sampler: {type: approach, frame: task, facing: [1, 0, 0], range_m: [0.8, 4.5], bearing_deg: 50,
              elevation_deg: [-10, 20], aim_jitter_deg: 12, roll_deg: 4, pitch_deg: 6}
  slalom:
    count: 400
    sampler: {type: approach, frame: [slalom_front, slalom_middle, slalom_back], both_sides: true, ...}
background: {count: 280, sampler: {type: free, depth_m: [0.3, 1.8]}}   # images with nothing labelled
randomize: {water: {scattering: [0.05, 0.2]}}   # any key left out takes its default
acceptance: {min_target_px: 150}                # optional renderer acceptance overrides
```

A non-native resolution keeps the camera's field of view: intrinsics are scaled to cover the new size and the
centre is cropped (never stretched).

### Samplers

Samplers place the robot; the camera follows its mount from the robot pack. `frame` is `task`, a frame id of
the task, or a list of frame ids (one is picked per attempt). `offset_m` moves the target point in that frame;
`roll_deg` / `pitch_deg` jitter the robot's attitude (uniform ±).

| Type | Use | Keys |
| --- | --- | --- |
| `approach` | forward cameras | `facing` (target-frame direction the camera comes from), `range_m`, `bearing_deg` (± about `facing`), `elevation_deg` (positive = camera above), `both_sides`, `aim_jitter_deg`, `max_aim_pitch_deg` (default 20) |
| `overhead` | down cameras | `altitude_m`, `radius_m` (horizontal offset disc), `yaw_deg` (default: any) |
| `free` | backgrounds | `depth_m` below the surface; anywhere in the pool 0.5 m from the walls |

Every attempt is checked before the colour image is rendered: camera under water and inside the pool, not
pressed against geometry, no labelled instance beyond `max_range_m`, and for task samples at least one labelled
instance of the task with `min_target_px` pixels (backgrounds: no labelled pixels). Rejected attempts are drawn
again up to `max_attempts`; a sample that never passes is skipped and logged.

### Randomization defaults

| Key | Default | Meaning |
| --- | --- | --- |
| `water.tint_scale` | `[0.85, 1.15]` | pool tint × U(range) |
| `water.absorption_scale` | `[0.7, 1.4]` | pool absorption × U(range) |
| `water.scattering` | `[0.05, 0.18]` | scattering = U(range) |
| `lighting.caustics`, `.exposure` | `[0, 1.3]`, `[0.75, 1.25]` | absolute |
| `lighting.direct_light_scale`, `.ambient_light_scale` | `[0.8, 1.2]` | × the pool's values |
| `lighting.sun_azimuth_deg`, `.sun_elevation_deg` | `[0, 360]`, `[35, 80]` | absolute |
| `time_s` | `[0, 600]` | caustic phase |
| `placement.task_yaw_deg`, `.task_offset_m` | `0`, `0` | rigid jitter of each task about its origin (±) |
| `placement.groups` | `[]` | lists of task ids that share one jitter, e.g. `[[surface, table]]` (the octagon over the table) |
| `indicators.latched_probability` | `0.2` | chance each indicator shows its latched colour |
| `image.noise_sigma`, `.blur_px` | `[0, 4]`, `[0, 0.8]` | Gaussian RGB noise (8-bit units) and blur |

## Command line

Install the tools with the `datasets` extra (OpenCV for export and previews; planning works without it) and
build the renderer once:

```bash
pip install -e '.[datasets]'
cmake --preset datasets && cmake --build --preset datasets
```

The renderer is found from `--renderer`, then `$NEREUS_DATASET_RENDERER`, then
`build/datasets/libraries/datasets/nereus-dataset-render`.

```bash
# Everything: plan, render (2 parallel shards), export YOLO segment, write a preview sheet.
nereus-dataset generate content/packs/datasets/uwrt_ffc_2026 --out ~/datasets/ffc

# A quick look at one task: 4 torpedo images at half resolution.
nereus-dataset generate content/packs/datasets/uwrt_ffc_2026 --out /tmp/try \
    --task torpedo --count 4 --resolution 960x600

# Several formats from one render.
nereus-dataset generate content/packs/datasets/uwrt_dfc_2026 --out ~/datasets/dfc \
    --formats yolo-seg,yolo-bbox,yolo-obb --workers 3

# Step by step.
nereus-dataset plan content/packs/datasets/uwrt_ffc_2026 --out ~/datasets/ffc/render
nereus-dataset render ~/datasets/ffc/render --workers 2
nereus-dataset export ~/datasets/ffc/render --format yolo-bbox --out ~/datasets/ffc/yolo-bbox
nereus-dataset preview ~/datasets/ffc/render --count 24
```

| Command | Does |
| --- | --- |
| `generate DATASET --out DIR` | `plan` into `DIR/render`, `render`, `export` each of `--formats` (default `yolo-seg`) into `DIR/<format>`, and write `DIR/preview.jpg`; `--no-export` stops after rendering |
| `plan DATASET --out DIR` | validate everything and write `DIR/job.json` (renders nothing) |
| `render DIR` | run `nereus-dataset-render DIR/job.json --shard i/N` for `--workers` N (default 2) in parallel, streaming their progress; fails if any shard fails |
| `export DIR --format F` | records -> one YOLO dataset (default `DIR/<format>`); `--labels` / `--model` re-export with another label pack or model |
| `preview DIR` | contact sheet of `--count` samples (default 16): class-coloured masks, outlines, boxes, names |
| `check-classes LABELS --yolo-config PATH` | compare model class orders with a detector parameter file |

`generate` and `plan` take the same overrides, applied to every selected task: `--task T` (repeatable; `background`
selects the background block, which is otherwise dropped when `--task` is given), `--count N` (per block),
`--range-m A B`, `--bearing-deg X`, `--elevation-deg A B` (approach samplers), `--altitude-m A B` (overhead
samplers), `--resolution native|WxH` and `--seed S`. Sample `k` always draws from its own random stream, so the
same spec, seed and sample index give the same image whatever the shard count.

## Output

```
DIR/render/
  job.json                    the renderer job (absolute paths)
  job/scenario_<i>.json       resolved scenarios, as python -m nereus.packs resolve writes them
  job/export.json             label pack, model and split the exporter defaults to
  images/<name>.jpg           RGB, e.g. torpedo_000042.jpg (task or background, global sample index)
  ids/<name>.png              16-bit instance ids: 0 = no part, k = the record's instances[k-1]
  records/<name>.json         camera, poses, randomization and every visible instance (task, part,
                              indicator, pixels, box, depth); label-pack neutral
  logs/shard_<i>_of_<n>.jsonl accepted / skipped per sample, attempts, rejection reasons, timings
DIR/yolo-seg/
  data.yaml                   train, val, [test], nc, names (model class order); no absolute path, so
                              the folder can be copied elsewhere (Ultralytics resolves it next to data.yaml)
  images/{train,val,test}/    hard links to render/images when possible
  labels/{train,val,test}/    one .txt per image, empty for backgrounds
DIR/preview.jpg
```

Export applies the label pack: instance -> class by its task's patterns and indicator state (a record from a
camera other than the model's is an error); classes the model lacks are dropped; `shape: outer` fills holes; instances under `min_visible_px` are dropped; images with a
labelled instance beyond `max_range_m` are skipped and counted. The split is a stable hash of the sample name.
Coordinates are normalized to [0, 1] with 6 decimals:

- `yolo-bbox`: `cls cx cy w h` around the mask's pixels (pixel edges: a box from x = 3 to 7 covers pixels 3..6).
- `yolo-seg`: `cls x1 y1 ...`, the outer contour simplified to 0.75 px. Polygon vertices are pixel centres in
  pixel-index coordinates (pixel (x, y) is the point x / width, y / height), as Ultralytics' own mask converters
  write them and as `cv2.fillPoly` reads them back, so a polygon lies half a pixel inside the box. A piece only
  1 px wide, which would collapse to a line, is written as its pixel-edge rectangle, so seg labels the same
  instances as bbox and obb. A mask split into pieces (a pole in front of an emoji, the image border) becomes
  one polygon joined at the pieces' closest points, as Ultralytics does with multi-part COCO masks.
- `yolo-obb`: `cls x1 y1 ... x4 y4`, the minimum-area rectangle around the mask's pixel edges, clockwise from
  the top corner.

The exporter prints per-class instance counts and warns when a class is more than 20 % from the mean (the team
training guide's balance rule). Re-exporting into the same folder replaces its images and labels.

Always look at a preview sheet before training on a new spec or label pack: a wrong part map, a mapping to the
wrong class or a sampler that only sees the edge of a board is obvious there.

## Adding a team label pack

1. Create `content/packs/labels/<team>/labels.yaml` with `tasks_pack` set to the course's tasks pack id.
2. Under `models`, list each deployed model: its camera (a robot pack camera sensor id), its range limit and its
   classes in id order. Run `check-classes` against your detector's config if it has `<camera>_class_id_map`
   strings.
3. Under `tasks`, map part names (see the course's `parts.yaml`) to your classes. Parts you do not list are
   never labelled.
4. Point a dataset spec's `labels:` at the folder, or re-export an existing render with
   `nereus-dataset export DIR --format yolo-seg --labels content/packs/labels/<team> --model <model>`.

`nereus-dataset plan` checks the pack against the scenario's `parts.yaml`: every pattern must match a part of its
task, a part may belong to one class, every class in `tasks` and `classes` must be in some model (a misspelt
class would otherwise label nothing), and `when.indicator` colours must be ones the task's indicators show. A
model class that no task maps is a warning. `export --labels` runs the same checks against the rendered course.

## Adding a task

1. Add the task to the tasks pack as usual ([Tasks and scoring](tasks.md)).
2. Name its parts in `parts.yaml`: a `textures` entry with seeds for printed artwork (then run
   `tools/generate_part_masks.py`), or a `visuals` entry for whole meshes or materials. `nereus-dataset-render
   JOB --describe` prints every scene instance with its material names, textures and resolved parts, which is
   the quickest way to find a material name or check that a part map landed.
3. Map the new parts to classes in each label pack that should label them.
4. Add a block to a dataset spec with a sampler aimed at the task's frames, then generate a few images
   (`--task <id> --count 8`) and check the preview.
