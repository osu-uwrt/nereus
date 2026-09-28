Dear ImGui v1.91.9b, unmodified upstream core and GLFW/OpenGL3 backends.

Upstream: https://github.com/ocornut/imgui/tree/v1.91.9b
License: MIT; see LICENSE.txt. Includes upstream stb headers and their notices.

Copied from the existing riptide_simulator camera_faker/vendor/imgui distribution,
whose README.riptide identifies this exact upstream release. No legacy application
or ROS code is included. Kept in-tree for reproducible, offline CMake builds.
Project formatting/static-analysis rules exclude these upstream sources. Review
and record dependency updates separately from application changes.

The 16 vendored files were byte-compared against the upstream tag archive:
`https://codeload.github.com/ocornut/imgui/tar.gz/refs/tags/v1.91.9b`.
Archive SHA-256: `8e1bbc76c71d74fef2fb85db7e7ca8eba13d6a86623c54992b60162db554ffdb`.
