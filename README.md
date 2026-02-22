# webodm-based-mapping-algorithm-using-drone
Overview
This repository contains Python scripts and experiments for a drone-based mapping workflow that integrates with WebODM/OpenDroneMap. The code includes mission scripts, path planning helpers, and video recording utilities for Raspberry Pi and simulated environments. It is organized as several project variants and experiments kept in subfolders.

Quick links
Project root: this file
Example mission (local): mission.py
Raspberry Pi folder: Fiver Project 4/Final Rasp
Simulation folder: Simulation
Repository structure
Fiver Project 4
mission.py — example mission orchestration
Path Planner.py — path planning utilities
video record.py — recording helper
Final Rasp — Raspberry Pi–targeted scripts (check_*.py, path_planner.py, mission.py, images.py)
Simulation — simulation-specific mission and video playback scripts
Updated — alternate/updated versions of mission and planner scripts
Goals and expectations
This repo is a research / prototype collection. Expect scripts to be at different maturity levels. Before running anything, inspect the target script to understand required hardware (real drone / flight controller, Raspberry Pi camera, or simulation) and configuration values.

Prerequisites
Python 3.8 or newer (3.10 recommended)
pip
Virtual environment tooling (python -m venv or virtualenv)
If you plan to use WebODM/OpenDroneMap: a running WebODM server (local or remote) or access to a WebODM instance. See WebODM installation docs: https://github.com/OpenDroneMap/WebODM
For real drone operation: companion hardware (Pixhawk, telemetry), and relevant libraries (e.g., dronekit, pymavlink). Ensure compliance with local laws and safe flight practices.
Recommended Python packages
The project does not include a locked requirements.txt. To run the scripts you will commonly need packages such as:

numpy
opencv-python
pillow
dronekit
pymavlink
requests
scikit-image
shapely
pyproj
Install them into a virtual environment like this (Windows example):

If you prefer a single file for dependencies create requirements.txt and run pip install -r requirements.txt.

Setup (local development)
Clone the repository:
Create and activate a virtual environment (see commands above).

Install dependencies (see Recommended Python packages).

Inspect the script you want to run and edit any configuration constants (file paths, camera indexes, connection strings to flight controller or MAVProxy, and WebODM server URL/API key if used).

Running example scripts
Simulation mission: run the simulation mission script in the Simulation folder:
Local mission (example):
Raspberry Pi scripts: copy the contents of Fiver Project 4/Final Rasp to your Pi and run with Python on the Pi. Confirm camera indexes and hardware serial ports.
WebODM integration
This repo focuses on collecting imagery and planning flights. To produce orthophotos/3D models, upload captured images to WebODM (either local or hosted). Typical steps:

Install and run WebODM (see WebODM docs).
Transfer captured images from the drone or Raspberry Pi to the machine running WebODM.
Upload images via the WebODM web UI or use the WebODM API to create a new task programmatically.
Inspect the results (orthophoto, DTMs, point clouds) in the WebODM UI.
Notes & safety
Several scripts assume a live flight controller connection; do not run those parts without ensuring your environment is safe and legal for flying.
This repository is experimental — test in simulation first.
Where to start
If you want to reproduce mapping: run a simulation mission, collect sample images, then upload them to WebODM to confirm processing works as expected.
If you want to test autopilot integration: inspect Final Rasp/mission.py and Final Rasp/path_planner.py for hardware-specific code and update connection strings.
Contributing
Contributions and improvements are welcome. Create issues or pull requests describing the change and testing steps.

License & contact
This repository does not include an explicit license. If you want one added, open an issue or add a LICENSE file.

For questions, mention the repository owner or open an issue in this repo.
