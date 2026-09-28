# ADR 0005: Independent native viewer and local-data slice

Status: accepted for the initial viewer increment.

## Decision

Use C++17 neutral visualization contracts, an optional yaml-cpp composition layer,
and an independently selected ImGui/GLFW/OpenGL 3.3 host. Preserve the UI/rendering
stack the user finds responsive in the old viewer, while giving the new host no
dependency on its ROS nodes, robot assets, runtime, or source tree. Reuse only the
identified upstream ImGui distribution with its license. Use system GLEW for GL
entry-point loading. The small line renderer owns GPU objects but no window.

Sources and optional playback capabilities are separate interfaces. Displays are
functions registered at construction. The host composes explicitly supplied
source factories and display functions; it does not inherit simulation behavior.
A source snapshot owns immutable, bounded histories. Rendering recomputes the
small initial scenes instead of introducing cache invalidation complexity before
there is a measured need. Frame lookup uses source-local integer time, nearest
common ancestors, and bracketing interpolation with no extrapolation.

Support one selected source per view initially. Multiple sources are independently
loadable, but equal names do not justify overlaying their frames/clocks. Future
overlay must introduce explicit alignment. The local recording proves seek,
rewind, disconnect, and reconnect. A full recording subsystem, live transport,
and bounded asynchronous ingestion are separate increments.

## Consequences

The viewer can be installed from a source copy without any simulation library,
Python package, robot profile, or ROS environment. Neutral contracts can also be
built without graphics. The renderer consumes only neutral geometry, and graphics
state stays on the UI thread. Compiled source/display extensions work against
installed headers; dynamic binary plugin ABI stability is not promised.

The initial GPU renderer draws lines and pose glyphs, not meshes or sensor images.
Sharing it later does not authorize using UI overlays or camera settings for
sensor acquisition. Mesh/image/cloud rendering and camera products require their
own explicit data/resource contracts. No simulation adapter or training work is
included. This begins phase 5A; it does not satisfy that phase's live ROS gate.

Window and context lifecycle follows the
[GLFW documentation](https://www.glfw.org/docs/3.3/quick.html), and the host uses
the upstream [ImGui GLFW/OpenGL backends](https://github.com/ocornut/imgui/tree/v1.91.9b/backends).
See [viewer contracts and checks](../VIEWER.md) for units, limits, ownership,
serialization, extension examples, and validation scope.
