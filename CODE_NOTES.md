# Code notes from the comment + reformat pass

Working document from the `comment-reformat` branch. That pass only added comments, docstrings and
blank lines (plus a formatter run); no code changed. Every one of the 316 code files was read; 311
were edited and 5 were already clear (`bridge/src/camera_sink.cpp`, `session/src/run_score.hpp`, and
the three `__init__.py` re-export files).

**How "no code changed" was checked:** each edited file was compared against `main`.
C++/GLSL: the token stream with comments and whitespace removed must be identical (string literals
and raw strings verbatim, preprocessor line ends kept, no `//` comment ending in `\`). Python: the
AST must be identical apart from newly added docstrings; existing docstrings and `# noqa` /
`# type:` / shebang comments must be unchanged, and no module docstring was added where `__doc__`
feeds argparse. CMake/shell: non-comment lines identical. Result: 311/311 files pass.

**No syntax errors were found.** Below are things noticed while reading — likely bugs, stale or
misleading comments, dead code and duplication — left untouched for you to triage. Line numbers
are approximate (files grew with the new comments); search for the named function.

## Most worth a look

- `integrations/uwrt/launch/sim_supervisor.py` `switch`: a failing `simulator.start()` kills the
  switcher thread and every later `load_scenario` is ignored as "a switch is in progress".
- `integrations/ros2/viewer/panels/uwrt_autonomy.cpp` destructor: `async_cancel_all_goals` cancels
  other clients' trees too, and its callback captures `this`.
- `integrations/ros2/viewer/panels/uwrt_electrical.cpp` / `uwrt_mapping.cpp`: goal-response
  timeouts leave goals running while the UI shows idle, or leave `calibrating` stuck true.
- `integrations/ros2/viewer/src/prior_map_editor.cpp`: switching robot namespace keeps the
  undo/redo stacks of the previous document; failed edits wipe the redo stack.
- `libraries/session/src/session.cpp` vs `tasks/runtime.cpp`: a pack projectile `max_age_s` above
  30 s is silently capped by a hard-coded `kMaxPayloadAgeS = 30.0`.
- `libraries/rendering/src/renderer.cpp`: the `ledRadiance` fallback can never be taken.
- Data races: `pack_scene` `warnings_` read without `mutex_`; `session_cameras` `reseedAll()`
  writes seeds outside `mutex_` while `describe()` reads them.
- `integrations/ros2/bridge/src/core.cpp`: `kill_stops_thrusters_` is parsed but never used.
- `integrations/ros2/viewer/panels/theme.cpp` `mix()`: always returns alpha 1, so adjusted
  translucent colours come out opaque.

## ROS 2 bridge — tests
- `integrations/ros2/bridge/tests/core_test.cpp` (~636): `Placement.MalformedRequestIsRejectedWithoutPlacing` is misnamed — it tests the SetBool "arm" service mapping CommandResult to success/message; nothing about malformed placement requests.
- `integrations/ros2/bridge/tests/real_pack_test.cpp` (~85): `Talos::send<T>` is never called (the claw test uses its own local `send` lambda) — dead code.
- `integrations/ros2/bridge/tests/core_test.cpp` (~107): template parameter `T` of `makeMessage<T>` is unused; the type comes from the string argument.
- `bridgeError` helper is duplicated verbatim in `core_test.cpp` and `visual_test.cpp` (could live in `fake_port.hpp`).

## Viewer — prior map editor, ros_side
- `integrations/ros2/viewer/src/prior_map_editor.cpp` `drawFileBar` robot/namespace switch: replaces `doc_` but keeps undo/redo stacks and `selected_`; undo after a switch restores the other namespace's objects. Also skips `loadState()`, so lock/hide flags reset to `load()` defaults.
- `prior_map_editor.cpp` `drawObjects` `row` lambda: a row shows only if it or a *direct* child matches the filter; a matching grandchild under non-matching ancestors is never shown.
- `prior_map_editor.cpp` `drawInspector`: `lock_orientation_to_config` only gets an undo step via `IsItemActivated`; `point_yaw_at_parent` gets none; neither calls `changed()`.
- `prior_map_editor.cpp` rename / reparent / swap: call `record()` first and `undo_.pop_back()` on failure, but `record()` already cleared `redo_`, so a failed edit wipes the redo stack.
- `prior_map_editor.cpp` `drawInspector`: `static int other` is function-static, shared across editor instances and selections.
- `prior_map.hpp` / `prior_map.cpp` `scalarEnd`: comment says it returns "the [start, end)" but it returns only `end` (stale comment, left as is).
- `ros_side.hpp` `MarkerRecord::attached`: says the pose is relative to the truth base link, but `receiveMarkers` also sets it for markers on the estimate base frame.
- `ros_side.cpp` `captureMpc`: local `truth` holds the lookup of `poseFrame()`, which is the estimate base link when the estimate is active — misleading name.
- `ros_side.cpp` `attach()`: `mechanismCommands_` is never cleared, so publishers from an earlier scenario persist.
- Missing direct includes (work transitively): `<cctype>`, `<string_view>` in `prior_map.cpp`; `<cmath>` in `ros_side.cpp`.
- Note: three existing comments in `prior_map_editor.{hpp,cpp}` sat above the wrong declaration and were moved (wording unchanged): `takeFocus()`, `robotStart()`, `labelCorner`.

## Viewer — panels (run, simulation, telemetry, theme, ROS/UWRT providers)
- `integrations/ros2/viewer/panels/theme.cpp` `mix()`: always returns alpha 1, so `pushUntil` / `readable` / `fillFor` drop the alpha passed in `toward` — translucent colours they adjust come out opaque.
- `theme.cpp` (~603): comment says "5:1 rather than 4.5" but the next line uses 5.5 for `muted` (partly stale comment).
- `panels/run_panel.cpp` (~251): `ImGui::TableNextRow(0, 30)` uses a raw 30 px row height instead of `ui(30)`; ignores interface scale.
- `panels/uwrt_electrical.cpp` (~397, ~405): on goal-response timeout, mag cal and tare are marked not running but no cancel is sent; a late acceptance leaves a goal running while the UI shows idle (mag cal even flips to "Calibrating"). Autonomy does cancel in the same case.
- `panels/uwrt_mapping.cpp` (~209): calibration timeout sets `canceling = true`; if the goal response/result never arrives, `calibrating` stays true forever because the timeout check requires `!canceling`.
- `panels/uwrt_autonomy.cpp` (~77): destructor `stop()` uses `async_cancel_all_goals`, which also cancels other clients' trees (contradicts "must not cancel a tree started by another client"); its reply callback captures `this` and can outlive the object (electrical/mapping destructors avoid callbacks).

## Python — `python/src/nereus`
- `python/src/nereus/datasets/__main__.py:1`: module docstring lists `{generate,plan,render,export,preview,check-classes}` but omits the `environments` subcommand.
- `python/src/nereus/packs/_semantics.py` (~660): the `# ---- bridge` section header sits above `equipment()`, not `bridge()`; there's no equipment header.
- `_semantics.py` `task()` regions loop: a `box` region's `frame` is checked twice (via `named` and again via `_known(...)`), so an unknown box frame yields two identical "unknown frame" problems.
- `_semantics.py` `bridge_binding`: `data.get("frame_names", {})` read twice (harmless redundancy).
- `python/src/nereus/datasets/compare.py` `settings_panel`: `not value or label in ("water", "image") or (label == "lighting")` — redundant parenthesised clause (behaviour looks intended).
- `datasets/_documents.py` vs `packs/_definitions.py`: `_load`, `_schema`, `schema_problems` are near-duplicates.

## ROS 2 bridge — sources
- `integrations/ros2/bridge/src/core.cpp` `steppedPeriod`: rejects `rate_hz <= 0` (or NaN) with "is too high" — misleading message.
- `bridge/src/core.cpp`: `kill_stops_thrusters_` is read from the safety block and stored but never used (killing goes through `commands_while_killed_`).
- `bridge/src/ros_types.hpp` (~117): stale comment refers to a nonexistent `readonly` parameter; bounds checks are done by `locateConst`, `locate` does none.
- `bridge/src/ros_types.cpp` (~197): a token with more than one index (`a[0][1]`) is rejected with "is not an array" — misleading.
- `bridge/src/native.cpp` `SourceRef::compile`: unknown-size (-1) dimensions treated as size 0; indexing a variable-size dimension other than the first would compute a wrong offset, and a trailing -1 dimension gives `count_ = 0`. Probably not hit by current specs.
- `bridge/src/mapping.cpp` `constantSpec`: `ints` flag computed but unused (`(void)ints;`).
- `bridge/src/visual.cpp` (~360): `try { ... } catch (const MappingError &) { throw; }` is a no-op.
- `bridge/src/visual.hpp`: `VisualContext::scenario_json` is set but never read.
- Duplicated helpers: `listRepr` in both `mapping.cpp` and `visual.cpp`; sec/nanosec split repeated in `mapping.cpp`, `visual.cpp`, `node.cpp` (`stampOf`).
- `bridge/src/main.cpp` (~167): an unknown sensor id reports "selected sensors must have unique ids" — misleading. With `--no-cameras --sensors ...`, unknown ids are silently dropped before that check.
- `bridge/src/session_camera_sink.cpp` (~315): comma operator joins two assignments in a loop body (`rgb = ..., depth = ...`) — works, easy to misread.
- `session_camera_sink.cpp` (~144): comment says "projectiles stream" but the code looks for `state:payloads` (stale name?).
- `session_camera_sink.cpp`: uses `offsetof` without `<cstddef>` (transitive include).

## Datasets + pack_scene libraries
- `libraries/datasets/src/output.cpp` (~35): `errno` is saved before `::close(fd)`, so a failing `close` reports a stale errno; same stale value if `write` returns 0.
- `libraries/pack_scene/src/pack_scene.cpp` (end of `PackScene` ctor): comment "Reject content that silently vanished from a strict scene (mesh() throws when strict)." has no code after it — dangling/stale.
- `pack_scene.cpp:1-3`: `<algorithm>`/`<array>` included before the file's own header (breaks the include order used elsewhere).
- `pack_scene.hpp` `warnings()` and `pack_scene.cpp` `describe()`: read `warnings_` without `mutex_`, while `mesh()`/`warn()` may append from other threads (header says multi-thread callable) — data race.
- `pack_scene.cpp` `mesh()`: two threads missing the cache for the same key both load the mesh (harmless duplicate work).
- `pack_scene.cpp` `buildTasks`: in non-strict mode a missing task mesh is warned twice (inside `mesh()` and again here).
- `libraries/datasets/tests/datasets_test.cpp` `makeGenerator`: `find("EGL")` redundant given `find("GL")`; any exception message containing "GL" turns the end-to-end tests into silent skips.
- `datasets_test.cpp` (supersample test): comment says id maps "are those of the single-sample run" but the test compares against the non-supersampled `plain` run.
- `libraries/datasets/src/main.cpp`: comment says sorting by scenario keeps "one scenario's meshes resident", but `Generator` caches every scenario and never frees one; benefit (if any) would be GPU-side.
- `libraries/datasets/src/job.cpp` `parseSampler` (fixed quaternion): element errors report the sampler's path, not `.world_from_root.orientation_wxyz`.
- `libraries/pack_scene/tests/pack_scene_test.cpp`: `ProfiledFloorDrapesStripesAndMeetsTheWalls` and `RecessesOpenTheirWallAndBoxesJoinTheirGroup` mostly test `rendering::makePoolScene` (arguably belong in rendering tests).
- Note: existing `resolveParts` comment in `mesh_parts.hpp` moved (unchanged) to sit directly above `resolveParts`.

## Sensors library + rendering/session tests
- `libraries/rendering/tests/renderer_image_test.cpp` (~56): comment says "Writes an 8-bit PNG" but the helper takes `bit_depth` and is used with 16 for `deep.png` — stale comment.
- `libraries/rendering/tests/offscreen_test.cpp` (~193): `writePng` uses `ASSERT_NE` in a `void` helper, so a failed open only returns from the helper and the test continues; no `setjmp` error handling (unlike `renderer_image_test.cpp`).
- `libraries/session/tests/prop_world_test.cpp`: uses `std::map` without `<map>` (transitive include).
- `libraries/session/tests/task_runtime_test.cpp` (~231): hard-coded pi literal where sibling tests use `M_PI` (cosmetic).
- `libraries/sensors/src/models.cpp` `PoolBottom::operator()`: always reports zero bottom velocity, so the moving-bottom path in `Dvl::sample` is reachable only via a custom `BottomQuery` (looks intentional; commented as stationary floor).
- `integrations/ros2/viewer/panels/bagging_panel.cpp` `topicList` (~290): `std::binary_search` over `rows` while selected-but-unpublished topics are appended to it; once the tail is unsorted, a published selected topic can be missed and shown as a duplicate row.
- `panels/bagging_panel.cpp` (~180): Record tooltip says a local bag "stops when the viewer closes" — true for the `stop_on_exit` default, wrong if configured `false`.
- `panels/motion_panel.cpp` `toolbar()` (~131): `enableKillButton({90, ...})` passes unscaled 90 while `sameLineIfFits(ui(90))` uses the scaled width.
- `panels/motion_panel.cpp` (~359): `dive_max_depth_z` option is accepted and validated but never read anywhere.
- `panels/motion_panel.cpp` (~218): existing comment "...shortens; the tooltip names it the first head names the frame and units..." reads as two sentences run together (left as is).
- `panels/motion_panel.cpp` (~23): first (name) parameter of `numericValue` unused.
- `panels/electrical_panel.cpp` (~183, ~200): IVC log child height (`{-1, 130}`) and Send button width (`send = 70`) not scaled with `ui()`.
- `panels/composition.cpp` (~225): ownership check uses `kinds.at(...)`; an unknown provider id throws `std::out_of_range` with an unhelpful message.
- `panels/ros_runtime.cpp` `RosMotion::complete` (~205): if the mode request succeeds after the switch was disabled / `ready()` went false, it returns silently; `pending` clears but "Changing control mode..." stays up.
- `viewer/include/nereus/ros_viewer/pins.hpp` (~71): comment says ini section `[Nereus][Pins]`, but `pins.cpp` writes `[NereusPins][Toolbar]`.

## Simulation + session_cameras libraries
- `libraries/session_cameras/tests/capture_tool.cpp:1-2`: header points to `tests/compare_pack_cameras.py`, which doesn't exist — stale.
- `libraries/session_cameras/src/session_cameras.cpp` `reseedAll()`: writes `camera->seeds`, `processors`, `seed_` on a worker thread after releasing `mutex_` (see `run()`), while `describe()` reads `seeds`/`seed_` under `mutex_` — data race if they overlap.
- `libraries/session_cameras/tests/session_cameras_test.cpp` (~121, ~251): second `make()` result (`again`, `cameras`) not null-checked before use.
- `session_cameras_test.cpp` (~228-238): `same` is counted but never checked; `second`/`id` silenced with `(void)` — leftover dead code.
- `session_cameras_test.cpp` (~168): comment "first request >= 200000001 ns" only holds for `period_ns == 66666667`.
- `libraries/simulation/tests/model_reference_test.cpp`: `#include "detail/thruster_dynamics.hpp"` and `<limits>` sit mid-file after several TESTs; file also hosts ThrusterDynamics tests despite its name.
- `libraries/simulation/tests/contact_reference_test.cpp` (~94-100): comment says non-aarch64 agreement is "about 1e-4" but the tolerance is `1e-3` (not a bug).
- `capture_tool.cpp`: uses `std::condition_variable` and `Eigen::Quaterniond` without including their headers (transitive).

## Session library
- `libraries/session/src/pack_runtime.cpp` (`addSensor`): comment "Adds one sensor to the runtime and returns its type-erased handle name check." is stale/garbled — `addSensor` returns void.
- `libraries/session/src/session.cpp` `Session::Impl`: `Json robot_safety;` never read or written.
- `session.cpp` vs `src/tasks/runtime.cpp`: session times a payload out after the projectile's `max_age_s` (default 30), but `TaskRuntime::stepProjectile` separately stops it at hard-coded `kMaxPayloadAgeS = 30.0` — a pack `max_age_s` > 30 is silently capped.
- `session.cpp` `stepProps` vs `waterAt()`: oscillating current computed twice with the same formula (could drift apart).
- `libraries/session/src/mechanisms.cpp` (~239): `water_density < 0` check followed by `water_density <= 0` — first is redundant except for its message.
- `mechanisms.cpp` `Impl::kill(killed)` applies kill/arm state rather than killing; `snapshot() const` mutates through `impl_` (existing comment notes this).
- `libraries/session/src/prop_world.cpp`: `pool_from_world = yawMatrix(pool_placement)` is actually world-from-pool (maps pool boxes into the world) — inverted name.
- `libraries/session/src/tasks/trackers.cpp` `PerforatedPanel`: `world_from_task_` from validated/renormalised `ownedPose`, but `task_from_world_` from the raw `world_from_task` (minor; `intersect` renormalises).
- `libraries/session/src/tasks/runtime.cpp` (~145): `score_rules` vs `scoring` easy to confuse (comment added).

## Python tests + tools
- `tests/python/test_datasets_export.py` `write_render` (~74): `for folder in (...)` shadows the `folder` parameter (harmless today, confusing).
- `tests/python/test_packs.py` (~65): `before = load_pack(pool).plain()` is compared to a second load of the same untouched pool — the assertion can't fail; leftover or meant to check something else.
- `tests/python/test_part_masks.py` (~93-98): `used` gets `{"asset": None}` for every prop without `visual_asset` (harmless, sloppy).
- `tools/check.py` (~65): error text "Rendering depends on simulation" is also raised for `libraries/spatial/` files — misleading for spatial.
- `tools/record_benchmark.py` (~67): records `c++ --version`, which may not be the `CMAKE_CXX_COMPILER` used; assumes `CMakeCache.txt` sits next to the executable. (Also hashes `applications/benchmark/main.cpp`, which doesn't exist in the repo.)
- `tools/generate_course_geometry.py` (~176): nested `wall()` redefined each loop iteration, capturing `axis` by closure (correct, odd style).
- `tools/generate_part_masks.py` (~235): `Image.fromarray(mask, mode="L")` — the `mode` argument is deprecated in recent Pillow; confirm before upgrading.
- `tests/python/test_sim_supervisor.py`: no `if __name__ == "__main__"` block unlike its siblings (cosmetic).

## Viewer — `app.cpp`
- `integrations/ros2/viewer/src/app.cpp`: member `std::vector<glm::mat4> lastLoaded_` declared but never used.
- `app.cpp` `viewFor(..., bool &)`: last parameter unused; caller passes a throwaway `bool unused`.
- `app.cpp` `step(double t)` and `drawMapToolbar(float width)`: ignore their arguments via `(void)`.
- Stale "View > Pool" references — pool switching lives in File > Switch pool (`drawFileMenu`): `PoolSwitch` struct comment, a comment in `drawPoolView`, and the runtime tooltip `"The pool (View > Pool)"` in `drawMapToolbar` (a string, left alone).
- `app.cpp` "Local camera cards" comment says each card refreshes every 0.1 s; `cardPeriod` actually uses `--card-rate` / `cards.rate_hz`, else the camera rate (15 Hz fallback). The comment also sits above `cardPeriod` rather than `renderLocalCards`.
- `app.cpp` `setupPriorMap` duplicates `loadMappingMarkers`' `<package>/<path>` share-directory lookup.
- `app.cpp` mesh-list `BeginChild` width is a fixed `260` while the adjacent search box uses `ui(260)` — doesn't scale.
- `app.cpp` closes and reopens `namespace nereus::ros_viewer::host` three times (harmless; merge leftover).

## UWRT integration, launch files, rules extension, misc
- `integrations/uwrt/launch/sim_supervisor.py` `switch` (~240): if `simulator.start()` raises (Popen fails), the exception kills the switcher thread; the queued request is never taken off, `requests.empty()` stays false, and every later `load_scenario` is ignored as "a switch is in progress".
- `integrations/uwrt/acceptance/stack.launch.py`: nothing launches it any more (was started by the removed `acceptance/compare.py`; `hold.py` expects the stack to already be running).
- `integrations/ros2/viewer/launch/robotics_pool_viewer.launch.py:6`: docstring usage names `nereus_viewer.launch.py`.
- `integrations/uwrt/launch/sim.launch.py` / `robot.launch.py`: `scenario` argument says it "overrides pool", but `_scenario()` raises if both are given.
- `sim.launch.py` / `robot.launch.py`: `_scenario` and `_gpu_env` copied verbatim in both; `POOLS` exists in both `robot.launch.py` and `sim_supervisor.py`.
- `integrations/uwrt/acceptance/mission.py:4`: docstring says the kill switch is released "once the tree waits for it"; code releases after a fixed `--kill-delay` of wall time.
- `integrations/uwrt/acceptance/hold.py` (~155-165): node/publisher count checks repeat what the discovery loop above already waited for.
- `extensions/rules/robosub_2026/robosub_2026.cpp` `selectRole`: role/side choice duplicated inline in `evaluate()`'s `handle` lambda.
- `robosub_2026.cpp`: uses `std::tuple` (`seen` set in `feed`) without `<tuple>` (transitive).
- `robosub_2026.cpp` `describe`: `key.get<std::string>()` throws if no role was selected and `options.role` is null/missing (possibly intended).
- `integrations/ros2/scripts/viewer_interfaces_smoke.sh`, `mission.py`, `robot.launch.py`, `sim_supervisor.py`: still prepend `python/src` to `PYTHONPATH` although setup is now uv/`.venv` (harmless; possibly stale — may be deliberate for ROS-launched processes).

## Rendering, spatial, cameras libraries
- `libraries/rendering/src/renderer.cpp` (~1003): `o.radiance < 0 ? ledRadiance : o.radiance` can never take the `ledRadiance` branch because `instance()` rejects negative radiance; `Resources::ledRadiance` is stuck at 60, and the per-frame `ledRadiance` uniform set at the top of `drawScene` is always overwritten per object.
- `libraries/cameras/src/camera.cpp` (~100): inside the JPEG block `const cv::Mat pixels` shadows the outer `pixels` (`size_t` count).
- `libraries/rendering/src/scene.cpp` (~379, ~381): comma operator makes two assignments in one statement (`yMin = ..., yMax = ...`).
- `libraries/rendering/src/assets.cpp` `perforatePanel` (~287): when a submesh is split, the `kept` remainder still carries all original vertices, including ones now only used by the moved panel triangles (harmless, wastes GPU memory).

## Viewer — other sources
- `integrations/ros2/viewer/src/main.cpp` (~113): the `--workspace` check throws `std::runtime_error` outside the `try` around `run()`; `std::stof`/`std::stoi` for `--inject-f`/`--orbit` can also throw there — bad input ends in `std::terminate` rather than a clean error.
- `viewer/src/main.cpp` `--card-rate` help says default is "each camera's own rate; 0 = camera rate", while `app.hpp` says `<0` means the host yaml `cards.rate_hz` — one is stale.
- `viewer/src/scene_model.hpp` (~18): `struct ExtraVisual` unused (`SceneModel` reads `extra_visuals` straight from YAML).
- `viewer/src/overlay_draw.cpp` `drawTfAxes`: dereferences `tf.font->FontSize` without a null check though `TfOverlay::font` defaults to `nullptr` (only caller sets it — latent).
- `overlay_draw.cpp` `drawTfTree`: child size `{620, 320}` and column widths 76/112 not wrapped in `ui()`.
- `overlay_draw.cpp`: `drawMpcPath` and `drawPlannedPath` are near-duplicate loops.
- `viewer/src/scenario_packs.cpp` (~118): trailing comment `// <source>/content/packs` on `root = packContent().parent_path().parent_path()`; `root` is `<source>`.
- `viewer/src/scenario.cpp`: `<regex>`, `<set>`, `<fstream>`, `<sstream>` look unused; `std::snprintf` used without `<cstdio>`.
- `viewer/src/window.cpp`: if `glewInit` fails (or anything throws after `glfwCreateWindow`), the constructor throws without destroying the window / `glfwTerminate` (minor leak).
- `viewer/src/top_down.cpp` `halve`: odd sizes drop the last row/column but `low`/`high` bounds are copied unchanged — smaller levels stretched by up to one source pixel (negligible).

## Viewer — tests
- `integrations/ros2/viewer/tests/host_viewer_input.cpp:7-8`: `<glm/gtc/matrix_transform.hpp>` and `<limits>` included a second time, after the `using namespace`.
- `viewer/tests/host_detection_pose.cpp:7`: `<iostream>` included twice.
- `viewer/tests/panels_panel_ros.cpp`: includes come after `using namespace nereus::ros_viewer::panels;`.
- `panels_panel_ros.cpp` (~80): comment "Send deferred replies from the test loop, after the callback has returned." sits before the `Registry` setup; the code it describes is in the `spin` lambda further down (misplaced).
- `viewer/tests/panels_operator_panels.cpp` (~48): alias `using Reset = std_srvs::srv::Trigger;` is also used for sync, capture and SVO-stop services — misleading name.
