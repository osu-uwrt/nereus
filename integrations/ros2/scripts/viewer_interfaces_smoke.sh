#!/bin/bash
# Live FastDDS smoke test of the viewer/operator interfaces (run manually, never in CI).
#
# Starts nereus-sim on the Talos scenario without cameras on a private ROS domain, echoes every viewer topic once,
# drives run control over simulator/run_command and simulator/reset_tasks, and pauses/resumes
# simulation time through the real_time_factor parameter of /talos/physics_simulator.
# Usage: viewer_interfaces_smoke.sh [ros_domain_id]      (kills everything it starts)
# Needs a sourced ROS 2 and a built build/ros-viewer.
set -u
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
export ROS_DOMAIN_ID="${1:-87}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$ROOT/python/src:${PYTHONPATH:-}"
SIM="$ROOT/build/ros-viewer/integrations/ros2/bridge/nereus-sim"
NS=/talos
OUT="$(mktemp -d)/run"
failures=0

# Helpers.
check() {  # check <description> <command...>: run, report PASS/FAIL
  local description="$1"; shift
  if "$@"; then echo "PASS  $description"; else echo "FAIL  $description"; failures=$((failures + 1)); fi
}
echo_once() {  # echo_once <topic> [extra args]: one message (no ros2 daemon: it is stale-prone)
  local topic="$1"; shift
  local text
  for _ in 1 2; do  # the first attempt can lose the type-discovery race
    text="$(timeout 25 ros2 topic echo --no-daemon --once --full-length "$@" "$topic" 2>&1)"
    grep -q "does not appear to be published" <<<"$text" || break
  done
  echo "$text"
}

contains() { grep -qF -- "$1" <<<"$2"; }

cleanup() { [ -n "${BRIDGE:-}" ] && kill "$BRIDGE" 2>/dev/null; wait 2>/dev/null; ros2 daemon stop >/dev/null 2>&1; }
trap cleanup EXIT

# Resolve the Talos scenario, start the bridge headless, and wait up to 60 s for its node to appear.
ros2 daemon stop >/dev/null 2>&1
mkdir -p "$OUT"
python3 -m nereus.packs resolve "$ROOT/content/packs/scenarios/talos_uwrt" -o "$OUT.resolved.json" || exit 1
"$SIM" "$OUT.resolved.json" --output "$OUT" --no-cameras --duration 240 >"$OUT.log" 2>&1 &
BRIDGE=$!
for _ in $(seq 60); do
  kill -0 "$BRIDGE" 2>/dev/null || { echo "bridge exited early:"; cat "$OUT.log"; exit 1; }
  ros2 node list --no-daemon 2>/dev/null | grep -q "$NS/physics_simulator" && break
  sleep 1
done
check "bridge node is /talos/physics_simulator" bash -c \
  "ros2 node list --no-daemon 2>/dev/null | grep -q $NS/physics_simulator"

# Every viewer topic publishes; the scenario topic is latched (transient local).
for topic in run_score task_score actual_thruster_forces claw_joints; do
  text="$(echo_once $NS/simulator/$topic)"; echo "--- $topic"; echo "$text" | head -6 | cut -c1-240
  check "$topic publishes" contains "data" "$text"
done
for topic in magnet_lights task_objects projectiles; do
  text="$(echo_once $NS/simulator/$topic)"; echo "--- $topic"; echo "$text" | head -8
  check "$topic publishes markers" contains "markers" "$text"
done
scenario="$(echo_once $NS/simulator/scenario --qos-durability transient_local --qos-reliability reliable)"
echo "--- scenario (latched, ${#scenario} chars)"; echo "$scenario" | cut -c1-160 | head -2
check "scenario is latched and carries asset_paths" contains "asset_paths" "$scenario"

# Run control: the auto-started run must be stopped before a new start is accepted.
pub() { ros2 topic pub --once -w 1 "$NS/simulator/run_command" std_msgs/msg/String "{data: '$1'}" >/dev/null 2>&1; }
score() { echo_once $NS/simulator/run_score; }
pub '{"action":"start","role":"rescue","heading_coin":false,"role_coin":true}'
check "start is rejected while running" contains "Command rejected: Stop the current run first" "$(score)"
pub '{"action":"stop"}'
check "stop stops the run" contains '"running": false' "$(score)"
pub '{"action":"start","role":"rescue","heading_coin":false,"role_coin":true}'
text="$(score)"
check "start begins a rescue run" contains '"intended_role": "rescue"' "$text"
check "start message comes from the task pack" contains "Run started; pass the gate first" "$text"
pub '{"action":"adjustment","points":75}'
check "adjustment reaches the total" contains '"total": 75.0' "$(score)"
ros2 topic pub --once -w 1 $NS/simulator/reset_tasks std_msgs/msg/Empty '{}' >/dev/null 2>&1
check "reset_tasks topic resets the run" contains '"adjustment": 0.0' "$(score)"
pub '{"action":"adjustment","points":5}'
ros2 service call $NS/simulator/reset_tasks std_srvs/srv/Trigger >/dev/null 2>&1
check "reset_tasks service still works" contains '"adjustment": 0.0' "$(score)"

# Start listening for task_events first, then reset after 4 s so the echo catches the reset event.
(sleep 4; ros2 topic pub --once -w 1 $NS/simulator/reset_tasks std_msgs/msg/Empty '{}' >/dev/null 2>&1) &
publisher=$!
events="$(echo_once $NS/simulator/task_events)"; wait "$publisher"
echo "--- task_events"; echo "$events" | head -3
check "reset publishes a tasks/reset event" contains '"kind": "tasks"' "$events"

# Pause / resume through the parameter the old viewer's Simulation panel sets.
clock() { echo_once /clock | grep -E '^ +(sec|nanosec):' | tr -d ' \n'; }
check "real_time_factor defaults to 1.0" contains "1.0" "$(ros2 param get $NS/physics_simulator real_time_factor 2>&1)"
check "/clock runs before the pause" test -n "$(clock)"
check "negative real_time_factor is rejected" bash -c \
  "ros2 param set $NS/physics_simulator real_time_factor -1 2>&1 | grep -qi 'fail'"
check "NaN real_time_factor is rejected" bash -c \
  "ros2 param set $NS/physics_simulator real_time_factor nan 2>&1 | grep -qi 'fail'"
ros2 param set $NS/physics_simulator real_time_factor 0 >/dev/null 2>&1
sleep 1; paused="$(clock)"
echo "/clock while paused: '${paused}'"
check "/clock stops at real_time_factor 0" test -z "$paused"
check "run_score keeps publishing while paused" contains "elapsed" "$(score)"
ros2 param set $NS/physics_simulator real_time_factor 1 >/dev/null 2>&1
sleep 1
check "/clock resumes at real_time_factor 1" test -n "$(clock)"

if [ "$failures" -eq 0 ]; then echo "ALL PASSED"; else echo "$failures FAILED (bridge log: $OUT.log)"; fi
exit "$failures"
