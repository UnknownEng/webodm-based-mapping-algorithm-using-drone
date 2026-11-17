# Autonomous Drone Mapping with WebODM

**End-to-end aerial mapping pipeline:** plan a survey from a GeoTIFF → fly it autonomously with
ArduPilot + MAVROS → capture geotagged photos at the right overlap → build an orthophoto,
elevation model and textured 3D model in WebODM.

![ArduPilot](https://img.shields.io/badge/ArduPilot-Copter-0b5cad)
![ROS](https://img.shields.io/badge/ROS-Noetic%20%7C%20Jazzy-22314e)
![MAVROS](https://img.shields.io/badge/MAVROS-1%20%7C%202-4a90d9)
![WebODM](https://img.shields.io/badge/WebODM-OpenDroneMap-3fae49)
![Python](https://img.shields.io/badge/Python-3.8%2B-3776ab)
![Status](https://img.shields.io/badge/status-simulation%20verified-orange)

---

## Demo

▶ **[Watch the full demo video (4 min 43 s)](https://github.com/UnknownEng/webodm-based-mapping-algorithm-using-drone/releases/download/v1.0-simulation/survey_demo.mp4)** — the
complete process in simulation: planning, autonomous flight with live drone camera and ROS 2 terminals,
WebODM processing, and the final maps. ([release page](https://github.com/UnknownEng/webodm-based-mapping-algorithm-using-drone/releases/tag/v1.0-simulation))

| Autonomous survey flight (Gazebo · drone camera · ROS 2 terminals) | Textured 3D model built by WebODM |
|---|---|
| ![Flight](Fiver%20Project%204/Simulation/demo_output/flight_preview.gif) | ![3D model](Fiver%20Project%204/Simulation/demo_output/model3d_preview.gif) |

| Survey plan | Orthophoto (3 cm/px) | Elevation model (DSM) |
|---|---|---|
| ![Plan](Fiver%20Project%204/Simulation/demo_output/01_plan.png) | ![Orthophoto](Fiver%20Project%204/Simulation/demo_output/06_ortho.png) | ![DSM](Fiver%20Project%204/Simulation/demo_output/07_dsm.png) |

**Simulation results** (140 × 140 m area, 40 m altitude): 195 geotagged photos, **195 / 195 reconstructed**,
orthophoto at 3 cm/px, tallest building measured **22.3 m vs 22.2 m true** (10 cm error),
survey speed held at **5.0 m/s**. Full WebODM report: [`report.pdf`](Fiver%20Project%204/Simulation/demo_output/report.pdf).

---

## How it works

```
 GeoTIFF of the area ──► path_planner.py ──► block_N.waypoints  (+ survey speed, photo distance)
                                                     │
            Drone: Pixhawk (ArduPilot) + Raspberry Pi │ MAVROS
   mission.py ── checks · upload · take-off · AUTO · RTL
   images.py  ── photo every N metres, GPS + lens in EXIF, geo.txt
                                                     │
                                                     ▼
            webodm_upload.py ──► WebODM ──► orthophoto · DSM · textured 3D model · report
```

## Features

**Mapping quality**
- Line spacing and photo distance computed from the camera (sensor + lens) and target overlap (70 % side / 75 % front)
- Flies only where the GeoTIFF has data; 2 waypoints per flight line; large areas split into battery-sized blocks
- Survey speed written into the mission (`DO_CHANGE_SPEED`, default 5 m/s) to avoid motion blur
- Photo every N metres while in AUTO — newest frame only, GPS + lens info in EXIF, `geo.txt` for WebODM
- Uses the best resolution the camera really delivers (up to 12 MP on the IMX477)

**Flight safety**
- Waypoint file validated before take-off: altitude limits, no sea-level altitudes, maximum distance from the drone
- Every command checked (mode, arming, take-off, upload); arming retried while showing ArduPilot's `PreArm` reason
- Low battery or timeout → RTL · pilot changes mode → script hands over · Ctrl+C in flight → RTL
- Works with old and new MAVROS (battery message type auto-detected) and requests telemetry streams itself

**Tooling**
- ROS 1 (Noetic) scripts for the Raspberry Pi and ROS 2 (Jazzy) versions
- One-command WebODM upload, processing and download
- Full software-in-the-loop simulation (ArduPilot SITL + Gazebo Harmonic) with automatic demo recording

## Repository structure

All code is in `Fiver Project 4/`:

| Path | Runs on | Purpose |
|---|---|---|
| `Final Rasp/path_planner.py` | ground PC | GeoTIFF (any CRS) → `.waypoints` survey blocks |
| `Final Rasp/mission.py` | Raspberry Pi (ROS 1) | Validates, uploads and flies a block with failsafes |
| `Final Rasp/images.py` | Raspberry Pi (ROS 1) | Distance-triggered, geotagged camera (Pi camera / USB / ROS topic) |
| `Final Rasp/Check_1.py` · `Check_2.py` · `Check_3.py` | Raspberry Pi (ROS 1) | Status monitor · arm/disarm test (props off) · 5 m hover test |
| `webodm_upload.py` | ground PC | Upload photos + `geo.txt` to WebODM, wait, download results |
| `ROS2/` | ROS 2 Jazzy | ROS 2 + MAVROS 2 versions of mission, camera and status |
| `Simulation/gazebo/` | Ubuntu 24.04 / WSL2 | Simulation world, SITL launch, demo recording and video editing |
| `Simulation/demo_output/` | — | Results of the simulated survey (previews, report) |

`mission.py`, `Path Planner.py` and `video record.py` in `Fiver Project 4/` and `Updated/` are copies of the
`Final Rasp/` versions (`video record.py` = `images.py`).

## Requirements

| Where | What |
|---|---|
| Drone | Pixhawk with ArduPilot Copter, Raspberry Pi, Raspberry Pi HQ camera (IMX477) or USB camera, GPS |
| Raspberry Pi | ROS Noetic + MAVROS (Ubuntu 20.04), or ROS 2 Jazzy + MAVROS 2 (Ubuntu 24.04); `pip install piexif` |
| Ground PC | Python 3.8+, `pip install -r requirements.txt`, [WebODM](https://github.com/OpenDroneMap/WebODM) |

## Quick start

```bash
# 1. Plan (ground PC) - prints the photo distance to use
python "Fiver Project 4/Final Rasp/path_planner.py" area.tif plans --altitude 40 --focal-mm 6 --speed 5

# 2. Fly (Raspberry Pi, MAVROS running) - two terminals
python3 images.py  --output ~/survey/block_1 --trigger-distance 7.85 --focal-mm 6
python3 mission.py plans/block_1.waypoints

# 3. Map (ground PC, WebODM running)
python webodm_upload.py survey/block_1 --url http://localhost:8000 --user admin
```

Before the first real flight follow **[FIELD_STEPS.md](FIELD_STEPS.md)**: bench test → camera walk test → hover → 2-line test → full block.

## Getting a good map

| Common cause of a poor WebODM result | How this project avoids it |
|---|---|
| Photos on a timer → too little overlap at speed | Photo every N metres (75 % front overlap) |
| Flight lines too far apart | Spacing from the lens (70 % side overlap) |
| No GPS in the photos | GPS in EXIF + `geo.txt` |
| No lens info → warped 3D | `FocalLength` + `FocalLengthIn35mmFilm` in EXIF |
| Flying too fast → blur | Survey speed set in the mission (default 5 m/s) |
| Stale buffered frames | Always the newest frame |
| Photos during take-off, turns, landing | Only in AUTO and above `--min-alt` |

Tips: pass your **real lens** with `--focal-mm` to both the planner and the camera script; fly 3–5 m/s; for a
fisheye lens add the WebODM option `camera-lens: fisheye`.

## Simulation

Runs the whole pipeline without hardware: ArduPilot SITL + Gazebo Harmonic + ROS 2 Jazzy + MAVROS 2 + WebODM
(Ubuntu 24.04 or WSL2).

```bash
cd "Fiver Project 4/Simulation/gazebo"
python3 gen_world.py           # once: textured town, camera drone, area.tif
bash start_sim.sh              # Gazebo + SITL + MAVROS + camera bridge (HEADLESS=1 for no window)
bash record_demo.sh            # optional: plan → fly → WebODM, recorded to video
bash stop_sim.sh
```

Scripts in [`Simulation/gazebo/`](Fiver%20Project%204/Simulation/gazebo): `gen_world.py` (world generator),
`sim_env.sh` (paths, GPU rendering on WSL2), `start_sim.sh` / `stop_sim.sh`, `survey.parm` (SITL parameters),
`record_demo.sh` (records the full run), `make_video.py` + `render_model.py` (edit the demo video).

## Safety

This software flies a real aircraft. Keep ArduPilot's own failsafes (battery, RC/GCS loss, EKF/GPS, geofence)
configured — the scripts do not replace them. Always keep a pilot ready on the RC transmitter, test in
simulation first, and follow your local drone regulations.

## Project status

- ✅ Code reviewed and improved; unit-tested against a simulated ArduPilot (13 / 13 scenarios)
- ✅ Full pipeline verified in SITL + Gazebo + WebODM
- ⏳ Real-drone validation — follow [FIELD_STEPS.md](FIELD_STEPS.md)

## License

No license has been chosen yet. Until one is added, all rights are reserved by the author.
