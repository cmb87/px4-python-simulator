#!/bin/bash
# Tear down everything run_multi.sh started (PX4 instances + bridge + offboard demo).
# Bracket-regex patterns so this script never kills itself.
pkill -9 -x px4 2>/dev/null
me=$$; ppid=$PPID
for pat in "src/main_multi[.]py" "src/offboard_demo[.]py" "[t]ail -f /tmp/px4in_"; do
  for p in $(pgrep -f "$pat"); do
    [ "$p" = "$me" ] && continue; [ "$p" = "$ppid" ] && continue
    kill -9 "$p" 2>/dev/null
  done
done
rm -f /tmp/px4in_* 2>/dev/null
echo "stopped."
