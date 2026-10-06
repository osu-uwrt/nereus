"""Run a UWRT mission tree (default RepairCompTree) against the simulator end to end.

Private ROS domain. Resolves the scenario, starts nereus-sim and the UWRT stack (mission_stack.launch.py),
releases the kill switch once the tree waits for it, runs the tree through
/talos/autonomy/run_tree, then stops the bridge so it writes tasks.json (scores + every task
event). Writes mission.json with the tree result, the visited subtree stack and the scores.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
# The mission trees live in the surrounding UWRT workspace (nereus is checked out inside it).
DEFAULT_TREE = ROOT.parent / "src/riptide_autonomy/trees/RepairCompTree.xml"


def stop(process: subprocess.Popen) -> None:
    """Stops a process group: SIGINT (12 s grace), then SIGTERM (5 s), then SIGKILL."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def write(path: Path, value: object) -> None:
    """Writes `value` as indented JSON (NaN/inf rejected)."""
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def drive(args: argparse.Namespace, log: list[dict]) -> dict:
    """Runs the tree as the operator would; appends each new subtree stack to `log`."""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from riptide_msgs2.action import ExecuteTree
    from riptide_msgs2.msg import KillSwitchReport
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    # Driver node: sim clock, software kill, run control and the tree action.
    rclpy.init()
    node = Node("mission_driver", namespace="/talos")
    clock = {"sim_s": 0.0}
    node.create_subscription(
        Clock, "/clock", lambda m: clock.update(sim_s=m.clock.sec + m.clock.nanosec * 1e-9), 10
    )
    kill = node.create_publisher(KillSwitchReport, "command/software_kill", 10)
    run = node.create_publisher(String, "simulator/run_command", 10)
    client = ActionClient(node, ExecuteTree, "autonomy/run_tree")
    start = time.monotonic()
    result: dict = {"tree": str(args.tree)}
    try:
        # Wait for autonomy, then let the stack settle.
        if not client.wait_for_server(timeout_sec=args.startup):
            result["error"] = "autonomy/run_tree never became available"
            return result
        settle = time.monotonic()
        while time.monotonic() - settle < args.settle:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Operator starts the scored run (Run panel "Start run").
        for action in ("stop", "start"):  # fresh judge after the killed float-up
            run.publish(String(data=json.dumps({"action": action})))
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)

        # Record the subtree stack whenever it changes.
        def feedback(message) -> None:
            stack = list(message.feedback.stack.stack)
            if not log or log[-1]["stack"] != stack:
                log.append(
                    {
                        "wall_s": round(time.monotonic() - start, 3),
                        "sim_s": round(clock["sim_s"], 3),
                        "stack": stack,
                    }
                )

        # Send the tree, release the kill after --kill-delay, and wait for the result or the timeout.
        goal = ExecuteTree.Goal(tree=str(args.tree))
        future = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        handle = future.result()
        if handle is None or not handle.accepted:
            result["error"] = "tree goal rejected"
            return result
        released_at = time.monotonic() + args.kill_delay
        released = False
        outcome = handle.get_result_async()
        while not outcome.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not released and time.monotonic() >= released_at:
                # Operator inserts the kill switch: firmware reports state/kill false.
                kill.publish(KillSwitchReport(kill_switch_id=1, switch_asserting_kill=False))
                released = True
                result["kill_released_sim_s"] = clock["sim_s"]
            if time.monotonic() - start > args.timeout:
                result["error"] = f"tree did not finish within {args.timeout} s wall time"
                handle.cancel_goal_async()
                break
        if outcome.done():
            value = outcome.result().result
            result.update(error_flag=bool(value.error), returncode=int(value.returncode))
        result["sim_s"] = clock["sim_s"]
        result["wall_s"] = time.monotonic() - start
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--scenario", type=Path, default=ROOT / "content/packs/scenarios/talos_uwrt"
    )
    parser.add_argument("--tree", type=Path, default=DEFAULT_TREE)
    parser.add_argument(
        "--sim", type=Path, default=ROOT / "build/ros-viewer/integrations/ros2/bridge/nereus-sim"
    )
    parser.add_argument("--domain", type=int, default=221)
    parser.add_argument("--startup", type=float, default=120, help="wall s for the stack")
    parser.add_argument("--settle", type=float, default=15, help="wall s before the goal")
    parser.add_argument("--kill-delay", type=float, default=8, help="wall s after the goal")
    parser.add_argument("--timeout", type=float, default=1800, help="wall s for the tree")
    args = parser.parse_args()
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    args.tree = args.tree.resolve()

    # Private, localhost-only ROS domain for the children and for this process's own node.
    env = dict(
        os.environ,
        ROS_DOMAIN_ID=str(args.domain),
        ROS_LOCALHOST_ONLY="1",
        RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
    )
    env["PYTHONPATH"] = os.pathsep.join([str(ROOT / "python/src"), env.get("PYTHONPATH", "")])
    os.environ.update(
        {k: env[k] for k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION")}
    )

    # Resolve the scenario pack, then start the bridge and the stack, each in its own process group.
    resolved = args.output / "resolved.json"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "nereus.packs",
            "resolve",
            str(args.scenario.resolve()),
            "-o",
            str(resolved),
        ],
        cwd=ROOT,
        env=env,
        check=True,
    )
    commands = [
        [str(args.sim.resolve()), str(resolved), "--output", str(args.output / "bridge")],
        ["ros2", "launch", str(HERE / "mission_stack.launch.py")],
    ]
    write(args.output / "commands.json", commands)
    processes, logs = [], []
    log: list[dict] = []
    result: dict = {}
    try:
        for i, command in enumerate(commands):
            handle = (args.output / f"process-{i}.log").open("w")
            logs.append(handle)
            processes.append(
                subprocess.Popen(
                    command,
                    cwd=ROOT,
                    env=env,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            )
        result = drive(args, log)
    finally:
        # Stop the stack, then the bridge (which writes tasks.json on exit), and collect the scores.
        for process in reversed(processes):
            stop(process)
        for handle in logs:
            handle.close()
        tasks_path = args.output / "bridge/tasks.json"
        tasks = json.loads(tasks_path.read_text()) if tasks_path.exists() else {}
        result["scores"] = tasks.get("scores", {})
        result["total"] = sum(result["scores"].values())
        result["subtrees_visited"] = sorted({name for entry in log for name in entry["stack"]})
        write(args.output / "tree_stack.json", log)
        write(args.output / "mission.json", result)
    print(json.dumps({k: result.get(k) for k in ("error", "returncode", "sim_s", "total")}))
    return 0 if "error" not in result else 1


if __name__ == "__main__":
    sys.exit(main())
