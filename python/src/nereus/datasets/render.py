"""Run ``nereus-dataset-render`` on a planned job as parallel shards, streaming their output."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
from pathlib import Path
from types import FrameType
from typing import IO

from nereus.packs import PackError

# Environment variable naming the renderer executable (second in the lookup order)
RENDERER_ENV = "NEREUS_DATASET_RENDERER"
# Source checkout: python/src/nereus/datasets/render.py -> repository root.
REPOSITORY = Path(__file__).resolve().parents[4]
# Presets that build the renderer, in lookup order (./build.sh picks ros-viewer when ROS is sourced).
BUILT = tuple(
    Path(f"build/{preset}/libraries/datasets/nereus-dataset-render")
    for preset in ("datasets", "ros-viewer")
)


def find_renderer(explicit: Path | None = None) -> Path:
    """``--renderer``, then ``$NEREUS_DATASET_RENDERER``, then the datasets / ros-viewer builds."""
    # An explicitly given renderer must work; never fall back past it
    given = [
        (explicit, "--renderer"),
        (Path(os.environ[RENDERER_ENV]) if os.environ.get(RENDERER_ENV) else None, RENDERER_ENV),
    ]
    for candidate, origin in given:
        if candidate is None:
            continue
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
        raise PackError(f"{candidate}: renderer from {origin} is not an executable file")

    for built in BUILT:
        candidate = REPOSITORY / built
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    raise PackError(
        f"no renderer: run ./build.sh at {REPOSITORY}, pass --renderer or set {RENDERER_ENV}"
    )


def _pump(stream: IO[str], prefix: str, lock: threading.Lock) -> None:
    """Copy a shard's output to stdout line by line; the lock keeps lines from interleaving."""
    for line in stream:
        with lock:
            sys.stdout.write(f"{prefix}{line}" if line.endswith("\n") else f"{prefix}{line}\n")
            sys.stdout.flush()


def _interrupt(signum: int, frame: FrameType | None) -> None:
    """SIGTERM handler: unwind like Ctrl-C so ``render`` terminates its shards."""
    raise KeyboardInterrupt(f"signal {signum}")


def render(folder: Path, *, workers: int = 2, renderer: Path | None = None) -> None:
    """Render ``folder/job.json`` as ``workers`` shards; raise if any shard fails."""
    job = Path(folder).resolve() / "job.json"
    if not job.is_file():
        raise PackError(f"{job}: no job (run 'nereus-dataset plan' first)")
    binary = find_renderer(renderer)

    workers = max(1, workers)
    lock = threading.Lock()
    processes: list[subprocess.Popen[str]] = []
    threads: list[threading.Thread] = []
    # SIGTERM (a job scheduler, kill) stops the shards like Ctrl-C instead of orphaning them.
    handle_term = threading.current_thread() is threading.main_thread()
    previous = signal.signal(signal.SIGTERM, _interrupt) if handle_term else None
    try:
        # One process per shard ("--shard i/n"), each with a thread streaming its output
        for index in range(workers):
            command = [str(binary), str(job), "--shard", f"{index}/{workers}"]
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
            processes.append(process)
            assert process.stdout is not None
            prefix = f"[{index}/{workers}] " if workers > 1 else ""
            thread = threading.Thread(
                target=_pump, args=(process.stdout, prefix, lock), daemon=True
            )
            thread.start()
            threads.append(thread)
        codes = [process.wait() for process in processes]
    except BaseException:
        # Interrupted or failed to start: stop every shard still running, then re-raise
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            process.wait()
        raise
    finally:
        if handle_term:
            signal.signal(signal.SIGTERM, previous)

    # Drain the remaining output before reporting failed shards
    for thread in threads:
        thread.join()
    failed = [f"shard {index}/{workers} exit {code}" for index, code in enumerate(codes) if code]
    if failed:
        raise PackError([f"{binary.name} failed: {item}" for item in failed])
