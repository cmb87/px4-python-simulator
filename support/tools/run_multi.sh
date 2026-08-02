#!/bin/bash
# Launch N PX4 (none_iris) SITL instances + the multi-vehicle Python bridge (one
# process, one shared clock). Then optionally run the offboard demo to fly them.
#
#   N=4 bash support/tools/run_multi.sh              # bridge + 4 PX4, drones sit on the ground
#   N=4 bash support/tools/run_multi.sh --fly        # also arm + take off + hover (offboard_demo)
#   N=4 SIM_DEMO=orbit bash support/tools/run_multi.sh --fly
#
# Open support/tools/viz/drone_viewer.html and connect to ws://localhost:8766 to watch.
# PX4-Autopilot location: set PX4_AUTOPILOT (default ~/PX4-Autopilot). Build SITL once with
#   cd ~/PX4-Autopilot && make px4_sitl_default
set -u
PX4="${PX4_AUTOPILOT:-$HOME/PX4-Autopilot}"
SIM="$(cd "$(dirname "$0")/../.." && pwd)"
PY="${SIM_PYTHON:-python3}"   # point at a venv with numpy+pymavlink+websockets if needed
N="${N:-4}"
BR_LOG=/tmp/bridge_multi.log
FLY=0; [ "${1:-}" = "--fly" ] && FLY=1

if [ ! -x "$PX4/build/px4_sitl_default/bin/px4" ]; then
  echo "PX4 SITL not built at $PX4/build/px4_sitl_default/bin/px4"
  echo "Build it: cd $PX4 && make px4_sitl_default   (or set PX4_AUTOPILOT)"; exit 1
fi

# clean previous run (exact-match px4; PID-filtered pkill for the rest to avoid self-kill)
pkill -9 -x px4 2>/dev/null
me=$$; ppid=$PPID
for pat in "src/main_multi[.]py" "src/offboard_demo[.]py" "[t]ail -f /tmp/px4in_"; do
  for p in $(pgrep -f "$pat"); do
    [ "$p" = "$me" ] && continue; [ "$p" = "$ppid" ] && continue
    kill -9 "$p" 2>/dev/null
  done
done
sleep 2
for I in $(seq 0 $((N-1))); do
  rm -rf "$PX4/build/px4_sitl_default/rootfs/$I"
  rm -f "/tmp/px4in_$I"; mkfifo "/tmp/px4in_$I"
done

# 1) multi-vehicle bridge (binds 4560..4560+N-1; one shared clock)
cd "$SIM"
SIM_NUM_VEHICLES="$N" SIM_VEHICLE_MODEL="${SIM_VEHICLE_MODEL:-iris}" \
  SIM_FORMATION_SPACING="${SIM_FORMATION_SPACING:-2.0}" \
  SIM_GT_OUTPUT_MODE=websocket SIM_GT_WS_PORT=8765 SIM_VIZ_WS_PORT=8766 \
  setsid "$PY" src/main_multi.py >"$BR_LOG" 2>&1 &
sleep 2

# 2) N PX4 none_iris instances; PX4 i connects out to port 4560+i.
for I in $(seq 0 $((N-1))); do
  setsid bash -c "export PX4_SIM_MODEL=none_iris; \
    tail -f /tmp/px4in_$I | '$PX4/build/px4_sitl_default/bin/px4' -i $I" \
    >"/tmp/px4_$I.log" 2>&1 &
  sleep 1
done
echo "launched: bridge=$BR_LOG  px4=/tmp/px4_{0..$((N-1))}.log"
echo "viewer: open support/tools/viz/drone_viewer.html -> connect ws://localhost:8766"

# 3) optionally fly them
if [ "$FLY" = "1" ]; then
  sleep 6   # let PX4 boot + EKF settle before offboard
  SIM_NUM_VEHICLES="$N" SIM_DEMO="${SIM_DEMO:-hover}" \
    setsid "$PY" src/offboard_demo.py >/tmp/offboard.log 2>&1 &
  echo "offboard demo: /tmp/offboard.log"
fi
