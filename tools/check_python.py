#!/usr/bin/env python3
"""Build a wheel from an sdist and test it in a disposable environment outside the checkout."""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import venv
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run(args: list[str | Path], *, cwd: Path, env: dict[str, str]) -> None:
    print("+ " + " ".join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), cwd=cwd, env=env, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sanitizers", action="store_true")
    parser.add_argument("--tidy", action="store_true")
    args = parser.parse_args()
    interpreter = Path(sys.executable).absolute()
    env = {
        "HOME": os.environ["HOME"],
        "PATH": f"{interpreter.parent}:/usr/local/bin:/usr/bin:/bin",
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "PIP_DISABLE_PIP_VERSION_CHECK": "1",
    }
    paths = ["python", "tests/python", "tools/check_python.py"]
    run([interpreter, "-m", "ruff", "check", *paths], cwd=ROOT, env=env)
    run([interpreter, "-m", "ruff", "format", "--check", *paths], cwd=ROOT, env=env)
    run(
        [
            interpreter,
            "-m",
            "mypy",
            "python/src",
            "tests/python",
            "tools/check_python.py",
        ],
        cwd=ROOT,
        env=env,
    )
    with tempfile.TemporaryDirectory(prefix="nereus-python-") as directory:
        temp = Path(directory)
        artifacts = temp / "artifacts"
        run(
            [interpreter, "-m", "build", "--sdist", "--no-isolation", "--outdir", artifacts],
            cwd=ROOT,
            env=env,
        )
        (archive,) = artifacts.glob("*.tar.gz")
        # This archive was just built from this checkout, not supplied by a third party.
        shutil.unpack_archive(archive, temp / "source")
        (source,) = (temp / "source").iterdir()
        build = temp / "native-build"
        settings = [f"-Cbuild-dir={build}", "-Ccmake.define.CMAKE_EXPORT_COMPILE_COMMANDS=ON"]
        if args.sanitizers:
            settings += ["-Ccmake.define.NEREUS_ENABLE_SANITIZERS=ON", "-Ccmake.build-type=Debug"]
        run(
            [
                interpreter,
                "-m",
                "build",
                "--wheel",
                "--no-isolation",
                "--outdir",
                artifacts,
                *settings,
            ],
            cwd=source,
            env=env,
        )
        if args.tidy:
            tidy = shutil.which("clang-tidy")
            if tidy is None:
                raise RuntimeError("--tidy requires clang-tidy")
            for path in sorted((source / "bindings/python").glob("*.cpp")):
                run([tidy, path, "-p", build], cwd=source, env=env)
        shutil.copytree(source / "tests/python", temp / "tests")
        # Runtime must not depend on the source copy or build outputs.
        shutil.rmtree(source)
        shutil.rmtree(build)
        venv.EnvBuilder(with_pip=True).create(temp / "environment")
        python = temp / "environment/bin/python"
        (wheel,) = artifacts.glob("*.whl")
        run([python, "-m", "pip", "install", wheel], cwd=temp, env=env)
        runtime_env = dict(env)
        if args.sanitizers:
            # CPython isn't an ASan executable. Preload the sanitizer and C++ runtime
            # before importing the extension so exception interception is available.
            compiler = shutil.which("c++")
            if compiler is None:
                raise RuntimeError("sanitizer checks require c++")
            libraries = [
                subprocess.check_output(
                    [compiler, f"-print-file-name={name}"], text=True, env=env
                ).strip()
                for name in ("libasan.so", "libstdc++.so")
            ]
            if not all(Path(path).is_file() for path in libraries):
                raise RuntimeError("could not locate sanitizer runtime libraries")
            runtime_env["LD_PRELOAD"] = ":".join(libraries)
            runtime_env["ASAN_OPTIONS"] = "detect_leaks=0"
            runtime_env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
        run(
            [python, "-I", "-m", "unittest", "discover", "-s", temp / "tests", "-v"],
            cwd=temp,
            env=runtime_env,
        )
    print("Python wheel, installed API, and requested checks passed.")


if __name__ == "__main__":
    main()
