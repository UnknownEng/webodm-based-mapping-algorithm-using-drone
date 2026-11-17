#!/usr/bin/env python3
"""
ROS 2 (Jazzy) + MAVROS 2 version of mission.py - fly a .waypoints survey on ArduPilot Copter.

Same flow and safety checks as the ROS 1 version (Final Rasp/mission.py):
validate file -> FCU/GPS/battery -> range check -> upload (row 0 = real home)
-> confirm -> GUIDED -> arm -> takeoff (altitude verified) -> AUTO + monitoring
(low battery / timeout -> RTL, pilot takes over -> stop) -> RTL -> wait for landing.
Ctrl+C in flight = RTL, on the ground = disarm.

    python3 mission.py block_1.waypoints [--takeoff-alt 30] [--min-battery 50] [--yes]
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mavros_link import MavrosLink, LinkError  # noqa: E402
from mavros_msgs.msg import Waypoint  # noqa: E402

FRAME_GLOBAL, FRAME_GLOBAL_REL_ALT, FRAME_GLOBAL_INT = 0, 3, 5
RELATIVE_FRAMES = {3, 6, 10, 11}
ABSOLUTE_FRAMES = {0, 5}
CMD_NAV_WAYPOINT, CMD_NAV_LAND = 16, 21
POSITION_COMMANDS = {16, 17, 18, 19, 21, 22, 31, 82}


class MissionError(Exception):
    pass


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((p2 - p1) / 2) ** 2 +
         math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * 6371000.0 * math.asin(math.sqrt(a))


def load_waypoint_file(path, min_alt=5.0, max_alt=120.0):
    """QGC WPL 110 -> mission items (home row skipped). See Final Rasp/mission.py."""
    with open(path) as f:
        lines = [ln.strip() for ln in f if ln.strip()]
    if not lines or not lines[0].startswith("QGC WPL"):
        raise MissionError("Not a QGC WPL waypoint file (first line must start with 'QGC WPL')")
    rows = [ln.split() for ln in lines[1:]]
    if not rows:
        raise MissionError("Waypoint file contains no waypoints")
    widths = {len(r) for r in rows}
    if len(widths) != 1 or widths.pop() not in (11, 12):
        raise MissionError("Every line must have 12 columns (or 11 for old planner files)")
    legacy = len(rows[0]) == 11

    items = []
    for line_no, t in enumerate(rows, start=2):
        try:
            index, frame, command = int(t[0]), int(t[2]), int(t[3])
            params = tuple(float(v) for v in t[4:8])
            lat, lon, alt = float(t[8]), float(t[9]), float(t[10])
            autocontinue = bool(int(float(t[11]))) if not legacy else True
        except ValueError as e:
            raise MissionError(f"Line {line_no}: bad number ({e})")
        if not legacy and index == 0:
            continue
        if legacy:
            frame, command = FRAME_GLOBAL_REL_ALT, CMD_NAV_WAYPOINT
        if command in POSITION_COMMANDS:
            if frame in ABSOLUTE_FRAMES:
                raise MissionError(f"Line {line_no}: absolute (sea-level) altitude is not supported. "
                                   "Use 'Relative' altitude.")
            if frame not in RELATIVE_FRAMES:
                raise MissionError(f"Line {line_no}: unsupported frame {frame}")
            if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0 and lon == 0):
                raise MissionError(f"Line {line_no}: invalid position {lat}, {lon}")
            if command != CMD_NAV_LAND and not (min_alt <= alt <= max_alt):
                raise MissionError(f"Line {line_no}: altitude {alt} m outside {min_alt}-{max_alt} m")
        items.append(dict(frame=frame, command=command, params=params,
                          lat=lat, lon=lon, alt=alt, autocontinue=autocontinue))
    if not any(it["command"] in POSITION_COMMANDS for it in items):
        raise MissionError("Waypoint file contains no navigation waypoints")
    return items, legacy


def to_waypoint(it, current=False):
    wp = Waypoint()
    wp.frame = it["frame"]
    wp.command = it["command"]
    wp.is_current = current
    wp.autocontinue = it["autocontinue"]
    wp.param1, wp.param2, wp.param3, wp.param4 = (float(p) for p in it["params"])
    wp.x_lat, wp.y_long, wp.z_alt = float(it["lat"]), float(it["lon"]), float(it["alt"])
    return wp


def run(link, args):
    items, legacy = load_waypoint_file(args.file, args.min_alt, args.max_alt)
    if legacy:
        link.warn("Old 11-column planner file: altitudes treated as relative to home")
    nav = [it for it in items if it["command"] in POSITION_COMMANDS]
    takeoff_alt = args.takeoff_alt if args.takeoff_alt is not None else nav[0]["alt"]
    if not (args.min_alt <= takeoff_alt <= args.max_alt):
        raise MissionError(f"Take-off altitude {takeoff_alt} m outside {args.min_alt}-{args.max_alt} m")
    if args.min_battery > 0 and args.rtl_battery >= args.min_battery:
        raise MissionError("--rtl-battery must be lower than --min-battery")
    link.info(f"Loaded {len(items)} mission items from {args.file}")

    link.wait_for_services()
    link.info("Waiting for FCU connection...")
    if not link.wait_until(lambda: link.connected, 60):
        raise MissionError("No FCU connection after 60 s")
    if link.armed:
        raise MissionError("Drone is already armed - refusing to start a mission")

    link.info(f"Waiting for GPS fix and battery >= {args.min_battery}%")
    last_log = 0.0
    while True:
        batt_ok = args.min_battery <= 0 or (link.battery is not None and link.battery >= args.min_battery)
        if link.gps_ok and batt_ok:
            break
        if time.time() - last_log > 5:
            batt = "not reported (use --min-battery 0 to skip)" if link.battery is None else f"{link.battery:.0f}%"
            link.info(f"GPS status: {link.gps_status} | battery: {batt}")
            last_log = time.time()
        time.sleep(0.2)

    home_lat, home_lon = link.lat, link.lon
    far = max(haversine_m(home_lat, home_lon, it["lat"], it["lon"]) for it in nav)
    if far > args.max_range:
        raise MissionError(f"A waypoint is {far:.0f} m away (limit {args.max_range:.0f} m). "
                           "Wrong file or wrong take-off site?")
    pts = [(home_lat, home_lon)] + [(it["lat"], it["lon"]) for it in nav] + [(home_lat, home_lon)]
    length = sum(haversine_m(*pts[i], *pts[i + 1]) for i in range(len(pts) - 1))
    timeout_s = args.timeout_min * 60 if args.timeout_min else length / 2.0 * 1.5 + 180

    home = dict(frame=FRAME_GLOBAL, command=CMD_NAV_WAYPOINT, params=(0, 0, 0, 0),
                lat=home_lat, lon=home_lon, alt=link.amsl or 0.0, autocontinue=True)
    wps = [to_waypoint(home, current=True)] + [to_waypoint(it) for it in items]
    last_seq = len(wps) - 1
    if not link.upload(wps):
        raise MissionError("Mission upload failed - not taking off")
    link.info(f"Uploaded {len(items)} items | path {length / 1000:.2f} km | farthest {far:.0f} m | "
              f"timeout {timeout_s / 60:.0f} min")

    if not args.yes:
        if input(f"Take off to {takeoff_alt:.0f} m and fly the mission? Type 'fly': ").strip().lower() != "fly":
            link.info("Cancelled by user")
            return

    armed_by_us = False
    try:
        if not link.set_mode("GUIDED"):
            raise MissionError("Could not switch to GUIDED")
        if not link.arm_for_flight(args.arm_timeout):
            raise MissionError("Arming failed - fix the pre-arm problem shown above")
        armed_by_us = True
        link.info("Armed")
        if not link.takeoff(takeoff_alt):
            link.arm(False)
            raise MissionError("Takeoff rejected - disarmed")
        link.info(f"Taking off to {takeoff_alt:.0f} m")

        def climbed():
            if not link.armed or link.mode != "GUIDED":
                raise MissionError(f"Takeoff interrupted (armed={link.armed}, mode={link.mode})")
            return link.rel_alt is not None and link.rel_alt >= 0.9 * takeoff_alt
        if not link.wait_until(climbed, 30 + takeoff_alt * 2):
            link.set_mode("LAND")
            raise MissionError(f"Did not reach take-off altitude (now {link.rel_alt}) - LANDING")
        link.info(f"Reached {link.rel_alt:.1f} m")

        link.last_reached = -1
        if not link.set_mode("AUTO"):
            link.set_mode("RTL")
            raise MissionError("Could not switch to AUTO - RTL")
        link.info("Mission started")

        start, low_since, reason, last_log = time.time(), None, None, 0.0
        while True:
            if not link.armed:
                link.warn("Drone disarmed during mission - script stopping")
                return
            if link.mode != "AUTO":
                link.warn(f"Mode changed to {link.mode} by pilot/failsafe - script hands over and stops")
                return
            if link.last_reached >= last_seq:
                link.info("All waypoints completed")
                reason = "mission complete"
                break
            if args.rtl_battery > 0 and link.battery is not None and link.battery < args.rtl_battery:
                low_since = low_since or time.time()
                if time.time() - low_since > 5:
                    reason = f"battery low ({link.battery:.0f}%)"
                    break
            else:
                low_since = None
            if time.time() - start > timeout_s:
                reason = "mission timeout"
                break
            if time.time() - last_log > 10:
                batt = f"{link.battery:.0f}%" if link.battery is not None else "?"
                gps = "" if link.gps_ok else " | GPS LOST (ArduPilot failsafe in charge)"
                link.info(f"WP {max(link.last_reached, 0)}/{last_seq} | alt {link.rel_alt or 0:.1f} m | "
                          f"battery {batt}{gps}")
                last_log = time.time()
            time.sleep(0.5)

        link.info(f"Returning home: {reason}")
        if not link.set_mode("RTL"):
            link.error("RTL rejected - trying LAND")
            link.set_mode("LAND")
        link.info("Waiting for landing and disarm (Ctrl+C to stop waiting)...")
        link.wait_until(lambda: not link.armed, 3600, hz=1)
        link.info("Landed and disarmed. Mission finished.")
    except (KeyboardInterrupt, Exception) as e:
        if armed_by_us and link.armed:
            if link.rel_alt is not None and link.rel_alt < 1.0 and link.mode == "GUIDED":
                link.warn("Aborting on the ground - disarming")
                link.arm(False)
            elif link.mode not in ("RTL", "LAND"):
                link.warn("Aborting in flight - RTL")
                if not link.set_mode("RTL"):
                    link.set_mode("LAND")
        if isinstance(e, KeyboardInterrupt):
            link.warn("Stopped by user")
            return
        raise


def main():
    p = argparse.ArgumentParser(description="Fly a .waypoints survey mission via MAVROS 2")
    p.add_argument("file", help=".waypoints file")
    p.add_argument("--takeoff-alt", type=float, default=None)
    p.add_argument("--min-battery", type=float, default=50, help="0 = skip battery check")
    p.add_argument("--rtl-battery", type=float, default=30, help="0 = off")
    p.add_argument("--min-alt", type=float, default=5)
    p.add_argument("--max-alt", type=float, default=120)
    p.add_argument("--max-range", type=float, default=1500)
    p.add_argument("--timeout-min", type=float, default=None)
    p.add_argument("--arm-timeout", type=float, default=60, help="keep retrying arming this long, s")
    p.add_argument("--yes", action="store_true")
    args, _ = p.parse_known_args()

    # Validate the file before starting ROS at all
    try:
        load_waypoint_file(args.file, args.min_alt, args.max_alt)
    except (OSError, MissionError) as e:
        sys.exit(f"Waypoint file error: {e}")

    link = MavrosLink("mission_runner")
    try:
        run(link, args)
    except (MissionError, LinkError) as e:
        link.error(str(e))
        sys.exit(1)
    finally:
        link.shutdown()


if __name__ == "__main__":
    main()
