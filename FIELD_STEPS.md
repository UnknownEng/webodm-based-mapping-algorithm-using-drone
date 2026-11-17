# Field steps: first real flight with the improved code

Hardware path = the original working setup: Raspberry Pi + ROS 1 (Noetic) + MAVROS + Pixhawk (ArduPilot),
IMX477 camera on `cv2.VideoCapture(0)`. Scripts are in `Fiver Project 4/Final Rasp/`.
Stop at any step that fails. Always have a pilot on the RC transmitter, ready to switch to LOITER/STABILIZE.

---

## 0. At home (before the field day)

1. **Copy** the `Final Rasp` folder to the Pi, e.g. `~/survey/`.
2. **Install** the one new dependency on the Pi:
   ```bash
   python3 -m pip install piexif
   ```
3. **Know your lens.** Find the focal length printed on it (Pi 6 mm CS lens = `6`, 16 mm C lens = `16`).
   Every command below uses `--focal-mm 6`; change it everywhere if yours differs.
4. **Make the plan** on the ground PC (needs a GeoTIFF of the area: a previous WebODM orthophoto, or an
   export from QGIS / Google Earth Pro with georeferencing):
   ```bash
   python "Final Rasp/path_planner.py" area.tif plans --altitude 40 --focal-mm 6 --speed 5 --block-size 150
   ```
   Note the printed **photo every X m** value (= `--trigger-distance`) and the minutes per block.
   Each block must take less than ~70 % of your battery time. Copy `plans/` to the Pi.
5. **ArduPilot failsafes** (Mission Planner → Config → Full Parameter List). Check that these are set:
   `FS_THR_ENABLE` (RC loss → RTL), `BATT_FS_LOW_ACT` / `BATT_LOW_VOLT` (low battery → RTL),
   `FS_GCS_ENABLE` if you use telemetry, `FENCE_ENABLE=1` with `FENCE_RADIUS` a bit larger than your area,
   `RTL_ALT` higher than the tallest obstacle.

## 1. Bench test, propellers OFF (Pi + Pixhawk powered)

```bash
roslaunch mavros apm.launch fcu_url:=<same fcu_url as your original setup>
rostopic type /mavros/battery          # note the type; the scripts handle both
python3 Check_1.py
```
✅ Pass when Check_1 shows: `Connected: True`, a battery % (or voltage), `GPS: FIX` (outdoors), mode shown.
If GPS / battery stay empty for > 1 min: MAVROS isn't receiving data (check `fcu_url` / baud rate).

```bash
python3 Check_2.py                     # props OFF: arms 5 s, then disarms
```
✅ Pass when it arms and disarms. If refused, read the `PreArm:` message in Mission Planner and fix it.

## 2. Camera + GPS walk test (on the ground, props OFF)

```bash
python3 images.py --output ~/walk_test --source opencv --always --trigger-distance 5 --focal-mm 6
```
Walk ~50 m with the drone (GPS needs sky view). ✅ Pass when:
* it prints `OpenCV camera 0: using WIDTHxHEIGHT`, ideally ≥ 1920x1080 (it tries 12 MP first);
* a photo is saved about every 5 m, each with lat/lon;
* the photos are sharp and correctly exposed, and `~/walk_test/geo.txt` lists them.

## 3. Hover test (props ON, open field, pilot ready)

```bash
python3 Check_3.py                     # GUIDED -> arm -> 5 m -> hover 10 s -> LAND
```
✅ Pass when it climbs to ~5 m, hovers steadily, lands and disarms. Pilot: if anything looks wrong, switch to
LOITER/STABILIZE; the script detects this and stops.

## 4. Short test mission (2 flight lines)

Make a 2-line file from the plan (home row + speed row + first 4 waypoints):
```bash
head -7 plans/block_1.waypoints > plans/test_2lines.waypoints
```
Terminal 1 (camera, start first):
```bash
python3 images.py --output ~/survey/test1 --source opencv --trigger-distance <X from planner> --focal-mm 6
```
Terminal 2 (mission):
```bash
python3 mission.py plans/test_2lines.waypoints
```
It checks GPS, battery and distance, uploads, asks you to type `fly`, then GUIDED → arm (retries up to 60 s,
showing ArduPilot's reason) → take-off → AUTO at 5 m/s → RTL → land.
✅ Pass when the drone flies both lines, comes home and lands, and the camera saved photos only along the lines.

**Process the test photos**: copy `~/survey/test1` from the Pi to the ground PC (e.g. `scp -r`), then (WebODM running):
```bash
python webodm_upload.py test1 --url http://<webodm-pc>:8000 --user admin --options "[{\"name\":\"fast-orthophoto\",\"value\":true}]"
```
✅ Pass when the orthophoto is in the right place on the map and the report (`report.pdf`) says all, or nearly
all, images were reconstructed.

## 5. Full block

Same as step 4 with `plans/block_1.waypoints`. New battery for every block. After each block:
check the photo count is close to the planner's estimate, then process all blocks together:
```bash
python webodm_upload.py <folder with all photos + geo.txt> --download orthophoto.tif dsm.tif textured_model.zip report.pdf
```
(For several blocks, put all photos in one folder and merge the `geo.txt` files: keep one `EPSG:4326` header line.)

---

## If something goes wrong

| Symptom | Likely cause / fix |
|---|---|
| Waits forever for GPS / battery | MAVROS gets no data streams → check connection; the scripts already request streams |
| `Arming refused (PreArm: ...)` until timeout | Fix what ArduPilot says (GPS, compass, EKF, safety switch, arming checks) |
| `waypoint is X m away` | Wrong plan file or wrong take-off site (limit: `--max-range`) |
| `altitude outside 5-120 m` | File uses sea-level altitudes → re-export with relative altitude |
| Camera `using 640x480` / warning < 2 MP | Camera driver mode; try `--width 1920 --height 1080` explicitly |
| Few photos / gaps | Trigger distance too large, or GPS lost; check `geo.txt` spacing |
| WebODM model warped / "spherical" | Wrong `--focal-mm`; fisheye lens → add option `camera-lens: fisheye` |
| WebODM fails "not enough images" | Overlap too low → lower `--trigger-distance`, or fly slower / higher |
| Drone does something unexpected | Pilot switches to LOITER / STABILIZE; the script stops and hands over |
