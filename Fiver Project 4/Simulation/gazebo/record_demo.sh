#!/usr/bin/env bash
# Run the WHOLE pipeline in simulation and screen-record it on a virtual
# screen (Xvfb :99, 1920x1080) - nothing from the Windows desktop is recorded.
#
#   plan (path_planner) -> fly (ROS 2 mission + geotagging camera, Gazebo + SITL)
#   -> process (WebODM: orthophoto, DSM, 3D model)
#
# Output: $SIM_DIR/video/demo_raw.mp4 (+ t_* timestamps for editing),
#         $SIM_DIR/photos, $SIM_DIR/results
# Needs: Xvfb openbox xterm xdotool ffmpeg, WebODM running, $SIM_DIR/webodm.env
# (no "set -u": ROS's setup.bash uses unset variables)
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/sim_env.sh"
bash "$HERE/sync_files.sh"
VID="$SIM_DIR/video"
mkdir -p "$VID"
cd "$SIM_DIR" && rm -rf photos plans results ./*.done sim.ready "$VID"/t_*
export DISPLAY=:99

# ---------- virtual screen ----------
pkill -f "^Xvfb :99" 2>/dev/null; sleep 1
Xvfb :99 -screen 0 1920x1080x24 -nolisten tcp >/tmp/xvfb.log 2>&1 &
sleep 2
xsetroot -solid "#1b1f24" 2>/dev/null || true
openbox >/tmp/openbox.log 2>&1 &
sleep 1

place() {  # place "<window name>" x y w h
  local id="" i
  for i in $(seq 60); do
    id=$(xdotool search --onlyvisible --name "$1" 2>/dev/null | head -1)
    [ -n "$id" ] && break
    sleep 0.5
  done
  [ -z "$id" ] && { echo "window '$1' not found"; return 1; }
  xdotool windowsize "$id" "$4" "$5"
  xdotool windowmove "$id" "$2" "$3"
}

XT=(xterm -fa "DejaVu Sans Mono" -fs 10 -bg "#0d1117" -fg "#c9d1d9" -xrm "XTerm*allowTitleOps: false" -sl 5000)
stamp() { date +%s.%N > "$VID/t_$1"; }

# ---------- start recording ----------
ffmpeg -nostdin -loglevel error -y -f x11grab -framerate 15 -video_size 1920x1080 -i :99 \
  -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p "$VID/demo_raw.mp4" &
FFMPEG=$!
stamp start

# ---------- 1. plan (full screen terminal) ----------
"${XT[@]}" -fs 14 -T "Planner terminal" -e bash "$HERE/demo_plan.sh" &
PLANNER=$!
place "Planner terminal" 0 0 1920 1080
until [ -f "$SIM_DIR/plan.done" ]; do sleep 1; done
sleep 5
stamp plan_done

# ---------- 2. simulator + windows ----------
NO_SYNC=1 bash "$HERE/start_sim.sh" >/tmp/start_sim.log 2>&1
place "Gazebo Sim" 0 0 1280 720
kill "$PLANNER" 2>/dev/null

"${XT[@]}" -T "Status (ROS 2)" -e bash -c "source '$HERE/demo_rc.sh'; run python3 project/ROS2/check_status.py; exec bash" &
place "Status (ROS 2)" 1280 480 640 600

"${XT[@]}" -T "Camera terminal" -e bash -c "source '$HERE/demo_rc.sh'; \
  run python3 project/ROS2/images.py --source ros --image-topic /survey_camera/image --output photos --trigger-distance 7.85; \
  touch camera.done; exec bash" &
place "Camera terminal" 640 720 640 360

python3 "$HERE/demo_viewer.py" --photos "$SIM_DIR/photos" --x 1280 --y 0 >/tmp/viewer.log 2>&1 &
VIEWER=$!
sleep 4
VID_WIN=$(xdotool search --onlyvisible --name "Drone camera" 2>/dev/null | head -1)
[ -n "$VID_WIN" ] && xdotool windowmove "$VID_WIN" 1280 0

# Gazebo view follows the drone
sleep 8
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'data: "iris_survey"' >/dev/null 2>&1
gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'x: -20, y: -20, z: 16' >/dev/null 2>&1
if ! pgrep -f "^python3 .*ROS2/images.py" >/dev/null; then
  echo "ABORT: camera script is not running - not flying without a camera"
  kill -INT "$FFMPEG"; bash "$HERE/stop_sim.sh"; pkill -f "^Xvfb :99"; exit 1
fi
touch "$SIM_DIR/sim.ready"
stamp sim_ready
"${XT[@]}" -T "Mission terminal" -e bash "$HERE/demo_mission.sh" &
place "Mission terminal" 0 720 640 360

# ---------- wait for the flight ----------
for _ in $(seq 5400); do [ -f "$SIM_DIR/mission.done" ] && break; sleep 1; done
stamp mission_done
sleep 6
pkill -INT -f "^python3 .*ROS2/images.py"
sleep 4
stamp camera_done
kill "$VIEWER" 2>/dev/null

# ---------- 3. WebODM (full screen terminal) ----------
"${XT[@]}" -fs 13 -T "WebODM terminal" -e bash "$HERE/demo_webodm.sh" &
place "WebODM terminal" 0 0 1920 1080
stamp webodm_start
for _ in $(seq 10800); do [ -f "$SIM_DIR/webodm.done" ] && break; sleep 2; done
stamp webodm_done
sleep 8

# ---------- stop ----------
kill -INT "$FFMPEG"; wait "$FFMPEG" 2>/dev/null
bash "$HERE/stop_sim.sh"
pkill -f "^Xvfb :99"
echo "Recorded $VID/demo_raw.mp4"
