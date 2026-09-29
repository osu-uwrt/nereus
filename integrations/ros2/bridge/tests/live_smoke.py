"""Live FastDDS smoke: run the Python bridge and robotics-sim-ros the same way on private ROS
domains, then compare their topic lists/types, sample messages, service replies and parameter.

    live_smoke.py <robotics-sim-ros binary> <resolved.json> <scenario pack dir>

Needs a sourced ROS 2 (Humble) with riptide_msgs2 and a python3 that can import robotics_platform.
Exit status 0 when both bridges agree. Values that depend on noise or wall time are compared
structurally (fields, frames, sizes); deterministic ones (actuator status, service replies) exactly.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
COLLECT = r'''
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Bool

WATCH = ["/clock", "/talos/vectornav/imu", "/talos/gyro/twist", "/talos/dvl_twist", "/talos/depth/pose",
         "/talos/simulator/ground_truth", "/talos/state/actuator/status", "/talos/state/actuator/busy",
         "/talos/simulator/run_score", "/talos/simulator/task_score", "/talos/simulator/actual_thruster_forces",
         "/talos/simulator/claw_joints", "/talos/simulator/magnet_lights", "/talos/simulator/task_objects",
         "/talos/simulator/projectiles", "/talos/simulator/scenario", "/talos/state/thrusters/telemetry",
         "/tf", "/tf_static"]
rclpy.init()
node = Node("smoke_probe")
time.sleep(1.0)
names = {n: sorted(t) for n, t in node.get_topic_names_and_types() if n not in ("/rosout", "/parameter_events")}
samples, counts = {}, {}
def listen(topic, type_name):
    latched = topic in ("/talos/simulator/scenario", "/tf_static")
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
    def cb(msg, topic=topic):
        counts[topic] = counts.get(topic, 0) + 1
        samples.setdefault(topic, []).append(message_to_ordereddict(msg))
    node.create_subscription(get_message(type_name), topic, cb, qos)
for topic in WATCH:
    if topic in names:
        listen(topic, names[topic][0])
end = time.time() + 3.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
def call(client_type, name, request):
    client = node.create_client(client_type, name)
    if not client.wait_for_service(timeout_sec=5.0):
        return {"error": "service unavailable"}
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()
    return None if result is None else message_to_ordereddict(result)
results = {}
results["reload"] = call(Trigger, "/talos/command/actuator/notify_reload", Trigger.Request())
results["arm_killed"] = call(SetBool, "/talos/command/actuator/arm", SetBool.Request(data=True))
results["reset_tasks"] = call(Trigger, "/talos/simulator/reset_tasks", Trigger.Request())
results["reset_sim"] = call(Trigger, "/talos/reset_sim_to_start", Trigger.Request())
results["sync_no_estimate"] = call(Trigger, "/talos/sync_sim_to_estimate", Trigger.Request())
request = SetParameters.Request()
request.parameters = [Parameter(name="real_time_factor", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.5))]
results["param_ok"] = call(SetParameters, "/talos/physics_simulator/set_parameters", request)
request.parameters = [Parameter(name="real_time_factor", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=-1.0))]
results["param_bad"] = call(SetParameters, "/talos/physics_simulator/set_parameters", request)
request.parameters = [Parameter(name="real_time_factor", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=1.0))]
call(SetParameters, "/talos/physics_simulator/set_parameters", request)
pub = node.create_publisher(Bool, "/talos/command/actuator/arm", 10)
json.dump({"topics": names, "counts": counts, "samples": {k: v[-1] for k, v in samples.items()},
           "first_clock": [s["clock"]["sec"] * 10**9 + s["clock"]["nanosec"] for s in samples.get("/clock", [])[:3]], "services": results},
          open(sys.argv[1], "w"), default=str)
rclpy.shutdown()
'''


def run_bridge(command: list[str], domain: int, env_extra: dict[str, str], collect_to: Path) -> None:
    env = {**os.environ, "ROS_DOMAIN_ID": str(domain), "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", **env_extra}
    bridge = subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, cwd=ROOT)
    try:
        time.sleep(4.0)
        script = collect_to.with_suffix(".py")
        script.write_text(COLLECT)
        probe = subprocess.run(["/usr/bin/python3", str(script), str(collect_to)], env=env, timeout=120,
                               capture_output=True, text=True)
        if probe.returncode != 0:
            raise RuntimeError(f"probe failed: {probe.stderr[-2000:]}")
    finally:
        bridge.send_signal(signal.SIGINT)
        try:
            bridge.wait(timeout=20)
        except subprocess.TimeoutExpired:
            bridge.kill()


def structure(value):
    """Shape of a message: keys, list lengths, scalar types."""
    if isinstance(value, dict):
        return {k: structure(v) for k, v in value.items()}
    if isinstance(value, list):
        return [len(value)] + ([structure(value[0])] if value and isinstance(value[0], (dict, list)) else [])
    return type(value).__name__


def main() -> int:
    binary, resolved, scenario = sys.argv[1:4]
    work = Path(tempfile.mkdtemp(prefix="bridge_smoke_"))
    cpp, py = work / "cpp.json", work / "py.json"
    run_bridge([binary, resolved, "--output", str(work / "cpp_run"), "--no-cameras"], 91, {}, cpp)
    python_path = os.pathsep.join([str(ROOT / "integrations/ros2/python"), str(ROOT / "python/src"),
                                   os.environ.get("PYTHONPATH", "")])
    run_bridge(["/usr/bin/python3", "-m", "robotics_platform_ros", scenario, "--output", str(work / "py_run"),
                "--no-cameras"], 92, {"PYTHONPATH": python_path}, py)
    a, b = json.loads(cpp.read_text()), json.loads(py.read_text())
    problems = []
    if a["topics"] != b["topics"]:
        for name in sorted(set(a["topics"]) | set(b["topics"])):
            if a["topics"].get(name) != b["topics"].get(name):
                problems.append(f"topic {name}: cpp={a['topics'].get(name)} python={b['topics'].get(name)}")
    for topic in sorted(set(a["samples"]) | set(b["samples"])):
        if topic not in a["samples"] or topic not in b["samples"]:
            problems.append(f"sample missing for {topic}: cpp={topic in a['samples']} python={topic in b['samples']}")
            continue
        if topic in ("/talos/simulator/run_score", "/talos/simulator/task_score", "/talos/simulator/scenario"):
            left, right = json.loads(a["samples"][topic]["data"]), json.loads(b["samples"][topic]["data"])
            if topic.endswith("scenario"):
                left.pop("asset_paths", None), right.pop("asset_paths", None)
                left.pop("content_sha256", None), right.pop("content_sha256", None)
                left.pop("unresolved", None), right.pop("unresolved", None)
                left.pop("bridge", None), right.pop("bridge", None)  # camera streams differ by pack filtering only
            if structure(left) != structure(right) and set(left) != set(right):
                problems.append(f"json keys differ on {topic}: {sorted(set(left) ^ set(right))}")
        elif structure(a["samples"][topic]) != structure(b["samples"][topic]):
            problems.append(f"message structure differs on {topic}")
        frame = lambda s: s.get("header", {}).get("frame_id")
        if frame(a["samples"][topic]) != frame(b["samples"][topic]):
            problems.append(f"frame_id differs on {topic}")
    status = "/talos/state/actuator/status"
    if a["samples"][status] != b["samples"][status]:
        problems.append(f"actuator status differs: {a['samples'][status]} vs {b['samples'][status]}")
    if a["services"] != b["services"]:
        for key in sorted(a["services"]):
            if a["services"][key] != b["services"].get(key):
                problems.append(f"service {key}: cpp={a['services'][key]} python={b['services'].get(key)}")
    clock_ok = all(x < y for x, y in zip(a["first_clock"], a["first_clock"][1:]))
    if not clock_ok:
        problems.append("cpp /clock is not increasing")
    print(f"topics compared: {len(a['topics'])} cpp, {len(b['topics'])} python; samples: {len(a['samples'])}; "
          f"work dir {work}")
    for problem in problems:
        print("MISMATCH:", problem)
    print("live smoke:", "FAILED" if problems else "ok")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
