#!/bin/bash
# ALBC Stonefish integration smoke bench (role: docs/stonefish-role-decision-2026-08-03.md).
# Narrowed criteria ONLY -- divergence/NaN/safety-gate/seam. Never a performance bench.
# Reuses the 2026-07-30 hold harness (probe_archive/2026-07/hold_20260730/hold_run.sh).
# $1 = record seconds (default 30)   $2 = scenario basename (default albc_empty.scn)
# NOTE: no `set -u` -- ROS humble setup.bash reads AMENT_TRACE_SETUP_FILES unbound.
REC=${1:-30}; SCEN=${2:-albc_empty.scn}
TAG=smoke_$(date +%y%m%d_%H%M%S)
source /opt/ros/*/setup.bash
source /workspace/install/setup.bash
export DISPLAY=${DISPLAY:-:0}
SHARE=/workspace/install/share/stonefish_description
OUT=/workspace/smoke; mkdir -p "$OUT"
LOG=$OUT/${TAG}.log; BAG=$OUT/bag_${TAG}

ros2 launch albc_bridge albc_smoke.launch.py gpu:=false simulation_rate:=100.0 \
  simulation_data:="$SHARE" scenario_desc:="$SHARE/scenarios/$SCEN" > "$LOG" 2>&1 &
LPID=$!

for i in $(seq 1 90); do grep -q 'Ready for running' "$LOG" 2>/dev/null && break; sleep 1; done
LIVE=$(ps -eo stat,comm | grep stonefish | grep -v Z | grep -vc grep)
echo "--- $TAG READY=$(grep -c 'Ready for running' "$LOG") LIVE_SIM=$LIVE"
[ "$LIVE" = "1" ] || { echo "SMOKE FAIL: not exactly one live sim ($LIVE)"; kill $LPID 2>/dev/null; exit 2; }

for i in $(seq 1 30); do ros2 node list 2>/dev/null | grep -q albc_bridge && break; sleep 1; done
ros2 node list 2>/dev/null | grep -q albc_bridge || { echo "SMOKE FAIL: bridge absent"; kill $LPID; exit 3; }

sleep 20   # free-float settle (metric rule: rms only, never peak)
ros2 bag record -o "$BAG" /albc/odometry /albc/setpoint/pwm /albc/servos \
  /albc/joint_states /albc/debug/latent /albc/debug/action > "$OUT/${TAG}_bag.log" 2>&1 &
BPID=$!
sleep "$REC"
kill -INT $BPID 2>/dev/null; wait $BPID 2>/dev/null

kill $LPID 2>/dev/null
pkill -f stonefish_simulator 2>/dev/null
pkill -f 'install/lib/albc_bridge' 2>/dev/null   # path pattern: cannot self-match this script
sleep 3
python3 "$(dirname "$0")/smoke_check.py" "$BAG" "$LOG"
