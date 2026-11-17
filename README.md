# webodm-based-mapping-algorithm-using-drone

Drone mapping pipeline: **plan a survey → fly it automatically (ArduPilot + MAVROS) →
take geotagged photos → build a 2D/3D map in WebODM**.

```
GeoTIFF of the area ──► path_planner.py ──► block_N.waypoints + survey_info.json
                                                   │
             drone (Pixhawk + Raspberry Pi)        ▼
   mission.py  (flies the block)   +   images.py  (photo every N m, GPS-tagged + geo.txt)
                                                   │
                                                   ▼
                     webodm_upload.py ──► orthophoto.tif, DSM, 3D model
```

## Files (`Fiver Project 4/`)

| File | Runs on | Purpose |
|---|---|---|
| `Final Rasp/path_planner.py` | ground PC | GeoTIFF (any CRS, e.g. WebODM orthophoto) → `.waypoints` files. Spacing computed from camera + overlap, only flies areas with image data, 2 waypoints per line. |
| `Final Rasp/mission.py` | Pi (ROS) | Validates the file, checks GPS/battery/range, uploads mission, GUIDED → arm → takeoff → AUTO → RTL, with failsafes. |
| `Final Rasp/images.py` | Pi (ROS) | Photo every N metres while in AUTO, GPS in EXIF + `geo.txt`. Pi camera, USB camera or ROS topic (Gazebo). |
| `Final Rasp/Check_1.py` | Pi (ROS) | Read-only status monitor. |
| `Final Rasp/Check_2.py` | Pi (ROS) | Arm 5 s / disarm (**props off**). |
| `Final Rasp/Check_3.py` | Pi (ROS) | Take off to 5 m, hover, land. |
| `webodm_upload.py` | ground PC | Upload photos to WebODM, wait, download orthophoto / DSM / model. |
| `ROS2/mission.py`, `ROS2/images.py`, `ROS2/check_status.py` | ROS 2 Jazzy | Same as the ROS 1 scripts, for ROS 2 + MAVROS 2 (Ubuntu 24.04). Shared code in `ROS2/mavros_link.py`. |
| `Simulation/gazebo/` | Ubuntu 24.04 | Full simulation: generated survey world, SITL launch, demo recording (see below). |
| `Simulation/mission.py` | SITL | Same as `Final Rasp/mission.py`. |
| `Simulation/video.py` | any | Record video from a camera or ROS image topic (Gazebo). |

`mission.py`, `Path Planner.py`, `video record.py` in the root and in `Updated/` are copies of the
`Final Rasp` versions (`video record.py` = `images.py`).

## Install

```bash
pip install -r requirements.txt
```
ROS 1: ROS Noetic + MAVROS (Ubuntu 20.04). ROS 2: ROS 2 Jazzy + `ros-jazzy-mavros` (Ubuntu 24.04).

> ArduPilot only sends GPS / battery / altitude telemetry when a ground station asks for it.
> The scripts request it through MAVROS (`set_stream_rate`), so they work with MAVROS alone.

## Use

1. **Plan** (ground PC):
   ```bash
   python "Final Rasp/path_planner.py" area.tif plans --altitude 40 --focal-mm 6 --block-size 200
   ```
   Prints the photo trigger distance and a time estimate per block.
2. **Check the drone** (props off for Check_2): `Check_1.py`, `Check_2.py`, then `Check_3.py` in an open field.
3. **Fly** (on the Pi, mavros running), two terminals:
   ```bash
   python3 images.py --output ~/survey/block_1 --trigger-distance 7.9
   python3 mission.py plans/block_1.waypoints
   ```
4. **Map** (ground PC, WebODM running):
   ```bash
   python webodm_upload.py survey/block_1 --url http://localhost:8000 --user admin
   ```

## Getting a good WebODM map

Why the first version gave poor WebODM results, and what now prevents it:

| Problem in the first version | Effect in WebODM | Now |
|---|---|---|
| Photo every 2 s, whatever the speed | ~35 % front overlap at 10 m/s → holes, failed matching | Photo every N m (`--trigger-distance` from the planner, 75 % overlap) |
| 24 m line spacing at 40 m | ~43 % side overlap | Spacing computed from the lens: 70 % side overlap |
| No GPS in the photos | Map not georeferenced, wrong scale | GPS in EXIF + `geo.txt` |
| No lens info in the photos | WebODM guesses the field of view → warped 3D | `FocalLength` + `FocalLengthIn35mmFilm` in EXIF |
| Drone at default speed (often 10 m/s) | Motion blur | Mission sets the survey speed (`--speed`, default 5 m/s) |
| Old buffered video frame saved | Photo position ≠ GPS tag | Always the newest frame |
| Photos during take-off / turns / landing | Bad photos | Only while in AUTO and above `--min-alt` |

Checklist:
* Tell the planner **and** the camera your real lens: `--focal-mm` (6 mm = Pi 6 mm CS lens, 16 mm = telephoto). Wrong value = wrong overlap and a warped model.
* Use the `--trigger-distance` the planner prints. Fly 3–5 m/s. Higher altitude = fewer photos and faster flights, but less detail per pixel.
* Upload `geo.txt` with the photos (`webodm_upload.py` does this). For a fisheye/wide lens add the WebODM option `camera-lens: fisheye`.
* The camera script prints the resolution it really uses — aim for at least 2 MP (it tries the full 12 MP first).

## Simulation (Ubuntu 24.04 / WSL2: ROS 2 Jazzy + Gazebo Harmonic + ArduPilot SITL + WebODM)

Everything lives in `Fiver Project 4/Simulation/gazebo/`:

| File | Purpose |
|---|---|
| `gen_world.py` | Generates the test world: 300 × 300 m textured town (roads, fields, cars, 14 buildings, 30 trees), an Iris drone with a downward camera matching the Pi HQ camera + 6 mm lens, and `area.tif` (the area to survey). |
| `sim_env.sh` | Paths (ArduPilot, ardupilot_gazebo), Gazebo resource paths, GPU rendering on WSL2. |
| `start_sim.sh` / `stop_sim.sh` | Start / stop Gazebo + ArduCopter SITL + MAVROS 2 + camera bridge (tmux session `survey`). |
| `survey.parm` | SITL parameters: 5 m/s survey speed, RTL at 30 m, telemetry streams. |
| `record_demo.sh` | Runs the whole pipeline (plan → fly → WebODM) on a virtual screen and records it. |
| `make_video.py`, `render_model.py` | Edit the recording into the final video (with orthophoto, DSM and a 3D-model turntable). |

Manual run:
```bash
cd "Fiver Project 4/Simulation/gazebo"
python3 gen_world.py                     # once
bash start_sim.sh                        # Gazebo window + SITL + MAVROS (HEADLESS=1 for no window)
source sim_env.sh && cd ~/survey_sim
python3 "project/Final Rasp/path_planner.py" area.tif plans --altitude 40
python3 project/ROS2/images.py --source ros --image-topic /survey_camera/image --output photos --trigger-distance 7.85 &
python3 project/ROS2/mission.py plans/block_1.waypoints
python3 project/webodm_upload.py photos --url http://localhost:8000 --user admin
bash stop_sim.sh
```

## Safety

* mission.py refuses absolute (sea-level) altitudes, altitudes outside 5–120 m, and waypoints
  more than 1.5 km away (all adjustable). It asks for confirmation before take-off (`--yes` skips).
* Low battery or mission timeout → RTL. Pilot changes mode → the script stops and hands over.
  Ctrl+C in flight → RTL.
* Keep ArduPilot's own failsafes (battery, GPS/EKF, RC/GCS loss, geofence) configured — the
  script is not a replacement for them. Always test in SITL first and follow local drone laws.

## License

No license file yet.
