"""Shard runner: renderer lookup, parallel shards, streamed output and failure reporting."""

import contextlib
import io
import os
import signal
import stat
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from nereus.datasets.__main__ import main
from nereus.datasets.render import RENDERER_ENV, find_renderer, render
from nereus.packs import PackError

FAKE = """\
#!/bin/sh
# Fake renderer: echoes its shard; shard 1 fails when FAIL_SHARD_1 is set.
echo "rendering $1 shard $3"
echo "progress $3" 1>&2
if [ -n "$FAIL_SHARD_1" ] && [ "$3" = "1/3" ]; then exit 4; fi
exit 0
"""


class RenderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.directory = tempfile.TemporaryDirectory()
        self.root = Path(self.directory.name)
        self.fake = self.root / "nereus-dataset-render"
        self.fake.write_text(FAKE)
        self.fake.chmod(self.fake.stat().st_mode | stat.S_IXUSR)
        (self.root / "job.json").write_text("{}")

    def tearDown(self) -> None:
        self.directory.cleanup()

    def test_lookup_order(self) -> None:
        self.assertEqual(find_renderer(self.fake), self.fake.resolve())
        with mock.patch.dict(os.environ, {RENDERER_ENV: str(self.fake)}):
            self.assertEqual(find_renderer(), self.fake.resolve())
            with self.assertRaises(PackError):
                find_renderer(self.root / "missing")
        with mock.patch.dict(os.environ, {RENDERER_ENV: str(self.root / "job.json")}):
            with self.assertRaises(PackError):
                find_renderer()  # not executable

    def test_shards_stream_and_succeed(self) -> None:
        output = io.StringIO()
        with mock.patch("sys.stdout", output):
            render(self.root, workers=3, renderer=self.fake)
        lines = sorted(output.getvalue().splitlines())
        job = self.root.resolve() / "job.json"
        for index in range(3):
            self.assertIn(f"[{index}/3] rendering {job} shard {index}/3", lines)
            self.assertIn(f"[{index}/3] progress {index}/3", lines)

    def test_a_failed_shard_fails_the_render(self) -> None:
        with mock.patch.dict(os.environ, {"FAIL_SHARD_1": "1"}):
            with mock.patch("sys.stdout", io.StringIO()):
                with self.assertRaises(PackError) as caught:
                    render(self.root, workers=3, renderer=self.fake)
        self.assertEqual(
            caught.exception.problems, ["nereus-dataset-render failed: shard 1/3 exit 4"]
        )
        with mock.patch.dict(os.environ, {"FAIL_SHARD_1": "1"}):
            with mock.patch("sys.stdout", io.StringIO()):
                with contextlib.redirect_stderr(io.StringIO()):
                    code = main(
                        ["render", str(self.root), "--workers", "3", "--renderer", str(self.fake)]
                    )
        self.assertEqual(code, 1)

    def test_sigterm_stops_the_shards(self) -> None:
        sleeper = self.root / "sleeper"
        sleeper.write_text('#!/bin/sh\necho $$ > "$PID_DIR/pid_${3%%/*}"\nexec sleep 60\n')
        sleeper.chmod(sleeper.stat().st_mode | stat.S_IXUSR)
        script = "import sys; from pathlib import Path; from nereus.datasets.render import render; "
        script += "render(Path(sys.argv[1]), workers=2, renderer=Path(sys.argv[2]))"
        source = Path(__file__).resolve().parents[2] / "python/src"
        environment = dict(os.environ, PID_DIR=str(self.root), PYTHONPATH=str(source))
        runner = subprocess.Popen(
            [sys.executable, "-c", script, str(self.root), str(sleeper)],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        pids = [self.root / "pid_0", self.root / "pid_1"]
        deadline = time.monotonic() + 20
        while not all(path.is_file() and path.read_text().strip() for path in pids):
            self.assertLess(time.monotonic(), deadline, "shards never started")
            time.sleep(0.05)
        runner.send_signal(signal.SIGTERM)
        self.assertNotEqual(runner.wait(timeout=20), 0)
        for path in pids:
            pid = int(path.read_text())
            deadline = time.monotonic() + 5
            while self._alive(pid):
                self.assertLess(time.monotonic(), deadline, f"shard {pid} survived SIGTERM")
                time.sleep(0.05)

    @staticmethod
    def _alive(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        status = Path(f"/proc/{pid}/stat")
        return not (status.is_file() and status.read_text().split(") ")[-1].startswith("Z"))

    def test_missing_job(self) -> None:
        (self.root / "job.json").unlink()
        with self.assertRaises(PackError):
            render(self.root, renderer=self.fake)


if __name__ == "__main__":
    unittest.main()
