# Contributing

This is an early local-development project. See README.md for supported builds
and LICENSE.md before redistributing. The local repository has no upstream remote;
review through local branches/commits until maintainers establish hosting.

## Workflow

Install the README dependencies and `clang-format==23.1.1` (Python package) for
consistent formatting. `clang-tidy` is optional for the quick check and required
for the full check; the initial analysis configuration is exercised with LLVM 14.
Python tooling uses the standard library and Python 3.9+.

```sh
python3 -m pip install --user clang-format==23.1.1
python3 tools/check.py --install-check --tidy
python3 tools/check.py --preset asan --install-check
```

The helper removes sourced ROS/workspace variables from the build/test environment,
checks C++ formatting and forbidden includes, builds with warnings as errors,
runs the behavior tests, optionally runs static analysis, and tests a relocated
installation with a separate C++ consumer. It does not install dependencies or
change your shell environment. Tests never require the original simulator.
To reformat C++, run `clang-format -i` on the files changed.

Use composition, explicit ownership, and small typed contracts. Keep calculations
free of IO and global mutable state. Core libraries may not depend on adapters or
UI. Config validation belongs at input boundaries; don't pass YAML or a runtime
service locator through numerical code. Implementing a genuine substitutable
interface may use inheritance; sharing stateful implementation should not.
See architecture section 11 for the full design rules and justified-exception policy.

Test behavior and invariants, not private code structure. Include appropriate
analytical cases or an independently stated expected result for numerical changes.
Check reset, instance isolation, units, and error handling when touching stateful
code. Public API changes need an installed-example/documentation update.

Each change should explain the problem, resulting behavior, verification, and
material limitations. Keep changes reviewable. Record decisions affecting public
contracts, ownership, dependencies, or numerical semantics in docs/decisions.
Avoid broad unrelated cleanup and speculative abstractions.

Commit coherent, reviewable increments with descriptive messages that identify
the resulting behavior. Keep the tests and documentation needed for a change in
the same commit; separate unrelated changes. Verify the staged diff and relevant
checks before committing. Keep build outputs, local environment settings, and
unrelated workspace edits out of the repository. Record unfinished capabilities
honestly in docs/STATUS.md. Work remains local; do not configure a remote or push
without an explicit request from the project owner.

## Python binding checks

Use a virtual environment and `pip install '.[dev]'` for the binding development
tools. The optional extension is built by the wheel backend; C++-only builds remain
unchanged. Run `tools/check_python.py` with that environment's Python to verify an
sdist-built wheel outside the source tree. Pass `--reference-runner
build/release/robotics-sim` for cross-language comparisons and `--sanitizers` for
ASan/UBSan under CPython. See [Python verification](docs/PYTHON.md) for commands,
system prerequisites, and sanitizer limitations. Keep public stubs aligned with
bindings; observation buffers must never alias live native state.

## Review and releases

The project owner acts as maintainer until named owners are established. Public
API/numerical changes require maintainer review; a contributor should not approve
their own release. Before public hosting, name maintainers and a private security
contact, resolve licensing for all imported files, and document supported targets.
Do not claim hardware/platform support from unexecuted CI configuration.

The API is experimental in 0.x. Record breaking changes in CHANGELOG.md. Before
1.0, define the stable surface and deprecation policy; internal kernel headers are
never part of that surface. Code releases, content schemas, and eventual extension
contracts have explicit versions. No compatibility to the old simulator is promised.
