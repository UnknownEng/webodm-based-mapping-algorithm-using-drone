#!/usr/bin/env bash
# "Planner terminal" of the demo.
source "$(dirname "${BASH_SOURCE[0]}")/demo_rc.sh"

say "STEP 1 - plan the survey from a GeoTIFF of the area"
run python3 \"project/Final Rasp/path_planner.py\" area.tif plans --altitude 40 --block-size 200
run head -5 plans/block_1.waypoints
touch plan.done
sleep 600
