"""Stops what ros2 launch started and left running when it exits (sim.launch.py, robot.launch.py).

ros2 launch (Humble) exits without stopping the processes it already started when it fails partway, e.g. on
"package 'complete_controller' not found": they keep running, in a process group Ctrl-C no longer reaches.
register() (from a launch file, which runs inside the ros2 launch process) stops them at exit: SIGINT, so a
simulator still writes its run records, then SIGKILL after `timeout` seconds. After a normal shutdown nothing
is left and it does nothing. It cannot help when ros2 launch itself is killed with SIGKILL.
"""

import atexit
import os
import signal
import sys
import time
from pathlib import Path


def _children(pid):
    """Live (not zombie) child processes of `pid`."""
    children = []
    for entry in Path("/proc").iterdir():
        try:
            if entry.name.isdigit():
                state, ppid = (entry / "stat").read_text().rsplit(")", 1)[1].split()[:2]
                if int(ppid) == pid and state != "Z":
                    children.append(int(entry.name))
        except OSError:  # exited
            continue
    return children


def stop_leftovers(launch, timeout):
    """Stops the children of `launch` (SIGINT, then SIGKILL after `timeout` seconds)."""
    left = _children(launch)
    if not left:
        return
    print(
        f"launch_cleanup: stopping {len(left)} processes ros2 launch left running", file=sys.stderr
    )
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGKILL, 1.0)):
        for pid in left:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + wait
        while (left := _children(launch)) and time.monotonic() < deadline:
            time.sleep(0.2)
        if not left:
            return


def register(timeout=20.0):
    """Stops this process's leftover children when it exits; call from a launch file."""
    atexit.register(stop_leftovers, os.getpid(), timeout)
