#!/usr/bin/env python3
"""Lint, type-check and test the pack tools, then check an installed wheel can validate the packs."""

import os
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
        [interpreter, "-m", "mypy", "python/src", "tests/python", "tools/check_python.py"],
        cwd=ROOT,
        env=env,
    )
    tests_env = dict(env, PYTHONPATH=str(ROOT / "python/src"))
    run(
        [interpreter, "-m", "pytest", "-q", "-p", "no:cacheprovider"],
        cwd=ROOT / "tests/python",
        env=tests_env,
    )
    with tempfile.TemporaryDirectory(prefix="nereus-python-") as directory:
        temp = Path(directory)
        run(
            [interpreter, "-m", "build", "--wheel", "--no-isolation", "--outdir", temp],
            cwd=ROOT,
            env=env,
        )
        (wheel,) = temp.glob("*.whl")
        venv.EnvBuilder(with_pip=True).create(temp / "environment")
        python = temp / "environment/bin/python"
        run([python, "-m", "pip", "install", wheel], cwd=temp, env=env)
        scenario = ROOT / "content/packs/scenarios/talos_uwrt"
        run([python, "-I", "-m", "nereus.packs", "validate", scenario], cwd=temp, env=env)
    print("Pack tools, installed wheel and tests passed.")


if __name__ == "__main__":
    main()
