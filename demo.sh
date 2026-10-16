#!/usr/bin/env bash
# FYP demo controller - everything you used to type, behind one command.
#
#   demo.sh start    start Gazebo + controllers (retries by itself) and press Play
#   demo.sh short    run the short gum demo (12 seams)
#   demo.sh full     run the full seam loop with gum
#   demo.sh clear    remove the gum from the ball
#   demo.sh stop     close Gazebo and everything else
#   demo.sh status   is the simulation running?
#   demo.sh backup   copy the project to Windows (Documents\FYP_backup\...)
#   demo.sh vscode   add "Run Task" entries for VS Code
#
WS="${FYP_WS:-$HOME/FYP_ball_gum}"
WORLD="football_gumming_robot_world"
LOG="$WS/launch.log"
WAIT="${FYP_WAIT:-80}"

load_env() {
  source /opt/ros/jazzy/setup.bash
  source "$WS/install/setup.bash"
}

controllers_ready() {
  local out
  out=$(timeout 8 ros2 control list_controllers 2>/dev/null) || return 1
  [ "$(echo "$out" | grep -cE '^(axis_position_controller|joint_state_broadcaster).*[[:space:]]active[[:space:]]*$')" -ge 2 ]
}

stop_all() {
  pkill -9 -f "ros2 launch" 2>/dev/null
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -9 -f ruby 2>/dev/null
  pkill -9 -f "controller_manager|robot_state_publisher|ros_gz|spawner" 2>/dev/null
  sleep 2
}

unpause() {
  timeout 8 gz service -s "/world/$WORLD/control" --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false' >/dev/null 2>&1
}

auto_scale() {
  # slower simulation -> stretch the motion so it looks right on screen
  local rtf
  rtf=$(timeout 6 gz topic -e -t "/world/$WORLD/stats" -n 1 2>/dev/null | grep -m1 real_time_factor | awk '{print $2}')
  python3 - "$rtf" <<'PYEOF'
import sys
try:
    r = float(sys.argv[1])
except Exception:
    r = 0.5
print(round(min(4.0, max(1.0, 1.0 / max(r, 0.1))), 1))
PYEOF
}

do_start() {
  load_env
  echo "Closing anything left over from before..."
  stop_all
  local ok=0 attempt t
  for attempt in 1 2 3; do
    echo "Starting Gazebo and controllers (try $attempt of 3, about 30 seconds)..."
    setsid nohup ros2 launch football_gumming_sim football_gumming.launch.py > "$LOG" 2>&1 < /dev/null &
    SECONDS=0
    while [ "$SECONDS" -lt "$WAIT" ]; do
      if controllers_ready; then ok=1; break; fi
      sleep 3
    done
    [ "$ok" = 1 ] && break
    echo "Controllers did not come up - restarting."
    stop_all
  done
  if [ "$ok" != 1 ]; then
    echo "FAILED: simulation did not start. Last lines of the log:"
    tail -n 15 "$LOG"
    exit 1
  fi
  unpause
  echo "READY - simulation is running. Now press a Run button."
}

do_run() {   # $1 = number of seam edges (empty = whole loop)
  load_env
  if ! controllers_ready; then
    echo "The simulation is not running. Press 'Start Simulation' first."
    exit 2
  fi
  unpause
  sleep 2
  local scale
  scale=$(auto_scale)
  echo "Simulation speed -> time scale $scale"
  python3 "$WS/seam_gum_sequence.py" "$scale" $1
}

do_clear() {
  load_env
  python3 "$WS/seam_gum_sequence.py" --clear
}

do_status() {
  load_env
  if controllers_ready; then echo "RUNNING: Gazebo and both controllers are active."; else echo "NOT RUNNING."; fi
}

do_backup() {
  local u dest
  u=$(cd /mnt/c 2>/dev/null && cmd.exe /c "echo %USERNAME%" 2>/dev/null | tr -d '\r\n')
  [ -z "$u" ] && u=PC
  dest="/mnt/c/Users/$u/Documents/FYP_backup/$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$dest" || { echo "Could not create $dest"; exit 1; }
  cp -r "$WS/src" "$dest/"
  cp -f "$WS"/*.py "$WS"/*.json "$WS"/demo.sh "$dest/" 2>/dev/null
  echo "Backup saved to Windows: Documents\\FYP_backup\\$(basename "$dest")"
}

do_vscode() {
  mkdir -p "$WS/.vscode"
  cat > "$WS/.vscode/tasks.json" <<'JSONEOF'
{
  "version": "2.0.0",
  "tasks": [
    { "label": "FYP: 1 Start simulation", "type": "shell", "command": "bash ${workspaceFolder}/demo.sh start", "problemMatcher": [] },
    { "label": "FYP: 2 Run gum demo (short)", "type": "shell", "command": "bash ${workspaceFolder}/demo.sh short", "problemMatcher": [] },
    { "label": "FYP: 3 Run full seam loop", "type": "shell", "command": "bash ${workspaceFolder}/demo.sh full", "problemMatcher": [] },
    { "label": "FYP: Clear gum", "type": "shell", "command": "bash ${workspaceFolder}/demo.sh clear", "problemMatcher": [] },
    { "label": "FYP: Stop everything", "type": "shell", "command": "bash ${workspaceFolder}/demo.sh stop", "problemMatcher": [] },
    { "label": "FYP: Backup to Windows", "type": "shell", "command": "bash ${workspaceFolder}/demo.sh backup", "problemMatcher": [] }
  ]
}
JSONEOF
  echo "VS Code tasks added. In VS Code: Ctrl+Shift+P -> 'Tasks: Run Task' -> pick an FYP task."
}

case "$1" in
  start)  do_start ;;
  short)  do_run 12 ;;
  full)   do_run "" ;;
  clear)  do_clear ;;
  stop)   stop_all; echo "Everything closed." ;;
  status) do_status ;;
  backup) do_backup ;;
  vscode) do_vscode ;;
  *) sed -n '2,13p' "$0" ;;
esac
