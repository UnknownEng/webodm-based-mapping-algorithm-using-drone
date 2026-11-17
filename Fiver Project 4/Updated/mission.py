#!/usr/bin/env python3
"""
Fly a survey mission (.waypoints file) on ArduPilot Copter through MAVROS (ROS1).

Flow (every step is checked; on failure the drone is kept on the ground,
or sent home / landed if already flying):

  1. Read + validate the waypoint file (altitude limits, no absolute altitudes)
  2. Wait for FCU connection, GPS fix and battery
  3. Check every waypoint is within --max-range of the drone
  4. Upload the mission (row 0 = real home position) and verify it
  5. Ask for confirmation (skip with --yes)
  6. GUIDED -> arm -> take off, waiting for the altitude to really be reached
  7. AUTO, then monitor: low battery / timeout -> RTL, pilot takes over -> script stops
  8. RTL after the last waypoint, wait until landed and disarmed

Ctrl+C while flying sends the drone home (RTL), or disarms it if still on the ground.

Usage:
    python mission.py block_1.waypoints [--takeoff-alt 30] [--min-battery 50] [--yes]
    python mission.py                     (file picker, needs a desktop)
"""
import argparse
import math
import os
import sys
import time

import rospy
from std_msgs.msg import Float64
from mavros_msgs.msg import State, WaypointReached, Waypoint, StatusText
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear, StreamRate
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState

# MAVLink constants
FRAME_GLOBAL = 0              # absolute altitude (AMSL)
FRAME_GLOBAL_REL_ALT = 3      # altitude above home
FRAME_GLOBAL_INT = 5
FRAME_GLOBAL_REL_ALT_INT = 6
FRAME_GLOBAL_TERRAIN_ALT = 10
FRAME_GLOBAL_TERRAIN_ALT_INT = 11
RELATIVE_FRAMES = {FRAME_GLOBAL_REL_ALT, FRAME_GLOBAL_REL_ALT_INT,
                   FRAME_GLOBAL_TERRAIN_ALT, FRAME_GLOBAL_TERRAIN_ALT_INT}
ABSOLUTE_FRAMES = {FRAME_GLOBAL, FRAME_GLOBAL_INT}

CMD_NAV_WAYPOINT = 16
CMD_NAV_LAND = 21
# NAV commands that carry a position (and therefore an altitude)
POSITION_COMMANDS = {16, 17, 18, 19, 21, 22, 31, 82}


class MissionError(Exception):
    pass


# ======================= WAYPOINT FILE ======================= #
def haversine_m(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def load_waypoint_file(path, min_alt=5.0, max_alt=120.0):
    """Parse a QGC WPL 110 file into a list of mission-item dicts (home row excluded).

    * 12-column files (QGC, Mission Planner, this project's planner): row 0 is
      the home position and is skipped; each row's frame/command/params are kept.
    * 11-column files (made by the OLD path planner): no home row, altitude was
      meant relative to home, so frame 3 is forced.
    * Absolute-altitude (AMSL) position items are rejected - convert them to
      relative altitude in Mission Planner first.
    """
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
            index = int(t[0])
            frame = int(t[2])
            command = int(t[3])
            p1, p2, p3, p4 = (float(v) for v in t[4:8])
            lat, lon, alt = float(t[8]), float(t[9]), float(t[10])
            autocontinue = bool(int(float(t[11]))) if not legacy else True
        except ValueError as e:
            raise MissionError(f"Line {line_no}: bad number ({e})")

        if not legacy and index == 0:
            continue  # home row - replaced by the real take-off position

        if legacy:
            frame = FRAME_GLOBAL_REL_ALT
            command = CMD_NAV_WAYPOINT

        if command in POSITION_COMMANDS:
            if frame in ABSOLUTE_FRAMES:
                raise MissionError(
                    f"Line {line_no}: absolute (sea-level) altitude is not supported. "
                    "Use 'Relative' altitude in Mission Planner / QGC.")
            if frame not in RELATIVE_FRAMES:
                raise MissionError(f"Line {line_no}: unsupported frame {frame}")
            if not (-90 <= lat <= 90 and -180 <= lon <= 180) or (lat == 0 and lon == 0):
                raise MissionError(f"Line {line_no}: invalid position {lat}, {lon}")
            if command != CMD_NAV_LAND and not (min_alt <= alt <= max_alt):
                raise MissionError(
                    f"Line {line_no}: altitude {alt} m outside the allowed {min_alt}-{max_alt} m "
                    "(change with --min-alt / --max-alt)")

        items.append(dict(frame=frame, command=command, params=(p1, p2, p3, p4),
                          lat=lat, lon=lon, alt=alt, autocontinue=autocontinue))

    if not any(it["command"] in POSITION_COMMANDS for it in items):
        raise MissionError("Waypoint file contains no navigation waypoints")
    return items, legacy


def path_length_m(start_lat, start_lon, items):
    pts = [(start_lat, start_lon)] + [(it["lat"], it["lon"]) for it in items
                                      if it["command"] in POSITION_COMMANDS]
    pts.append((start_lat, start_lon))
    return sum(haversine_m(*pts[i], *pts[i + 1]) for i in range(len(pts) - 1))


def to_ros_waypoint(it, current=False):
    wp = Waypoint()
    wp.frame = it["frame"]
    wp.command = it["command"]
    wp.is_current = current
    wp.autocontinue = it["autocontinue"]
    wp.param1, wp.param2, wp.param3, wp.param4 = it["params"]
    wp.x_lat = it["lat"]
    wp.y_long = it["lon"]
    wp.z_alt = it["alt"]
    return wp


# ======================= TELEMETRY ======================= #
def subscribe_battery(callback, timeout=10.0):
    """Subscribe to /mavros/battery with whatever message type this MAVROS publishes.

    Current MAVROS: sensor_msgs/BatteryState (.percentage 0..1).
    Old MAVROS:     mavros_msgs/BatteryStatus (.remaining 0..1).
    callback(fraction) gets 0..1 (NaN/negative = unknown). Returns the type used."""
    topic, found = "/mavros/battery", None
    end = time.time() + timeout
    while found is None and time.time() < end and not rospy.is_shutdown():
        try:
            found = dict(rospy.get_published_topics()).get(topic)
        except Exception:
            found = None
        if found is None:
            rospy.sleep(0.5)
    if found == "mavros_msgs/BatteryStatus":
        from mavros_msgs.msg import BatteryStatus
        rospy.Subscriber(topic, BatteryStatus, lambda m: callback(m.remaining))
    else:
        rospy.Subscriber(topic, BatteryState, lambda m: callback(m.percentage))
    return found or "sensor_msgs/BatteryState (assumed - topic not seen yet)"


class Telemetry:
    def __init__(self):
        self.state = None
        self.state_time = 0.0
        self.battery = None            # percent, None = not reported
        self.gps_status = NavSatStatus.STATUS_NO_FIX
        self.lat = None
        self.lon = None
        self.amsl = None
        self.rel_alt = None
        self.last_reached = -1
        self.fcu_text = ""             # last pre-arm / warning text from ArduPilot
        rospy.Subscriber("/mavros/state", State, self._state)
        rospy.Subscriber("/mavros/statustext/recv", StatusText, self._statustext)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._gps)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self._rel_alt)
        rospy.Subscriber("/mavros/mission/reached", WaypointReached, self._reached)
        rospy.loginfo(f"Battery topic type: {subscribe_battery(self._battery)}")

    def _state(self, msg):
        self.state = msg
        self.state_time = time.time()

    def _battery(self, fraction):
        # 0..1, NaN/negative if the FCU does not report it
        if not math.isnan(fraction) and fraction >= 0:
            self.battery = fraction * 100.0

    def _gps(self, msg):
        # NavSatStatus: -1 NO_FIX, 0 FIX, 1 SBAS_FIX, 2 GBAS_FIX
        self.gps_status = msg.status.status
        if not (math.isnan(msg.latitude) or math.isnan(msg.longitude)):
            self.lat, self.lon, self.amsl = msg.latitude, msg.longitude, msg.altitude

    def _rel_alt(self, msg):
        self.rel_alt = msg.data

    def _reached(self, msg):
        self.last_reached = max(self.last_reached, msg.wp_seq)
        rospy.loginfo(f"Waypoint reached: {msg.wp_seq}")

    def _statustext(self, msg):
        # severity <= 4 = WARNING or worse; "PreArm:" / "Arm:" texts explain arming refusals
        if msg.severity <= 4 or "arm" in msg.text.lower():
            self.fcu_text = msg.text

    # helpers
    @property
    def connected(self):
        return self.state is not None and self.state.connected and time.time() - self.state_time < 5.0

    @property
    def armed(self):
        return self.state is not None and self.state.armed

    @property
    def mode(self):
        return self.state.mode if self.state is not None else ""

    @property
    def gps_ok(self):
        return self.gps_status >= NavSatStatus.STATUS_FIX and self.lat is not None


def wait_until(predicate, timeout, hz=10.0):
    """Return True once predicate() is true, False on timeout."""
    deadline = time.time() + timeout
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown() and time.time() < deadline:
        if predicate():
            return True
        rate.sleep()
    return predicate()


# ======================= FCU COMMANDS ======================= #
class Fcu:
    def __init__(self, tel, service_timeout=30.0):
        self.tel = tel
        names = ["/mavros/cmd/arming", "/mavros/set_mode", "/mavros/cmd/takeoff",
                 "/mavros/mission/clear", "/mavros/mission/push"]
        for n in names:
            try:
                rospy.wait_for_service(n, timeout=service_timeout)
            except rospy.ROSException:
                raise MissionError(f"MAVROS service {n} not available - is mavros running?")
        self._arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self._mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self._takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self._clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
        self._push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
        self._stream = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)

    def request_streams(self, rate_hz=10):
        """ArduPilot only sends GPS / battery / altitude data when a ground station
        asks for it - MAVROS alone does not. Request all streams."""
        self._call(self._stream, "set_stream_rate", stream_id=0, message_rate=rate_hz, on_off=True)

    @staticmethod
    def _call(proxy, name, **kwargs):
        try:
            return proxy(**kwargs)
        except rospy.ServiceException as e:
            rospy.logerr(f"{name} call failed: {e}")
            return None

    def set_mode(self, mode, timeout=10.0):
        resp = self._call(self._mode, "set_mode", custom_mode=mode)
        if resp is None or not resp.mode_sent:
            rospy.logerr(f"FCU rejected mode {mode}")
            return False
        if not wait_until(lambda: self.tel.mode == mode, timeout):
            rospy.logerr(f"Mode did not change to {mode} (still {self.tel.mode})")
            return False
        return True

    def arm(self, value, timeout=10.0):
        resp = self._call(self._arm, "arming", value=value)
        if resp is None or not resp.success:
            rospy.logerr(f"FCU rejected {'arming' if value else 'disarming'} "
                         f"(result {getattr(resp, 'result', '?')}) - check pre-arm messages")
            return False
        return wait_until(lambda: self.tel.armed == value, timeout)

    def arm_for_flight(self, timeout=60.0):
        """Arm, retrying while ArduPilot's pre-arm checks settle (IMU / EKF / GPS need
        30-60 s after power-on). A refused arm request changes nothing, so retrying
        is safe. ArduPilot's own reason ("PreArm: ...") is shown each time."""
        end = time.time() + timeout
        attempt = 0
        while not rospy.is_shutdown():
            attempt += 1
            self.tel.fcu_text = ""
            resp = self._call(self._arm, "arming", value=True)
            if resp is not None and resp.success and wait_until(lambda: self.tel.armed, 10):
                return True
            rospy.sleep(1.0)  # give the FCU's explanation time to arrive
            reason = self.tel.fcu_text or f"result {getattr(resp, 'result', '?')}"
            if time.time() >= end:
                rospy.logerr(f"Arming refused: {reason}")
                return False
            rospy.logwarn(f"Arming refused ({reason}) - retry {attempt} (up to {timeout:.0f} s)")
            rospy.sleep(4.0)
        return False

    def takeoff(self, altitude):
        resp = self._call(self._takeoff, "takeoff", min_pitch=0.0, yaw=float("nan"),
                          latitude=self.tel.lat, longitude=self.tel.lon, altitude=altitude)
        if resp is None or not resp.success:
            rospy.logerr(f"FCU rejected takeoff (result {getattr(resp, 'result', '?')})")
            return False
        return True

    def upload(self, waypoints):
        resp = self._call(self._clear, "mission clear")
        if resp is None or not resp.success:
            rospy.logerr("Could not clear the old mission")
            return False
        resp = self._call(self._push, "mission push", start_index=0, waypoints=waypoints)
        if resp is None or not resp.success or resp.wp_transfered != len(waypoints):
            rospy.logerr(f"Mission upload failed ({getattr(resp, 'wp_transfered', 0)}/{len(waypoints)} items)")
            return False
        return True


# ======================= MISSION ======================= #
def pick_file():
    if os.name != "nt" and not os.environ.get("DISPLAY"):
        raise MissionError("No waypoint file given and no desktop for a file picker. "
                           "Usage: mission.py block_1.waypoints")
    from tkinter import Tk
    from tkinter.filedialog import askopenfilename
    Tk().withdraw()
    return askopenfilename(title="Select .waypoints file", filetypes=[("QGC Waypoints", "*.waypoints")])


def log_every(seconds, _last=[0.0]):
    now = time.time()
    if now - _last[0] >= seconds:
        _last[0] = now
        return True
    return False


def run(args):
    # ---------- 1. waypoint file (before touching the drone) ----------
    wp_file = args.file or pick_file()
    if not wp_file:
        raise MissionError("No waypoint file selected")
    items, legacy = load_waypoint_file(wp_file, args.min_alt, args.max_alt)
    if legacy:
        rospy.logwarn("Old 11-column planner file: altitudes treated as relative to home")
    first_alt = next(it["alt"] for it in items if it["command"] in POSITION_COMMANDS)
    takeoff_alt = args.takeoff_alt if args.takeoff_alt is not None else first_alt
    if not (args.min_alt <= takeoff_alt <= args.max_alt):
        raise MissionError(f"Take-off altitude {takeoff_alt} m outside {args.min_alt}-{args.max_alt} m")
    if args.min_battery > 0 and args.rtl_battery >= args.min_battery:
        raise MissionError("--rtl-battery must be lower than --min-battery")
    rospy.loginfo(f"Loaded {len(items)} mission items from {wp_file}")

    tel = Telemetry()
    fcu = Fcu(tel)

    # ---------- 2. connection, GPS, battery ----------
    rospy.loginfo("Waiting for FCU connection...")
    if not wait_until(lambda: tel.connected, 60):
        raise MissionError("No FCU connection after 60 s")
    if tel.armed:
        raise MissionError("Drone is already armed - refusing to start a mission")

    rospy.loginfo(f"Waiting for GPS fix and battery >= {args.min_battery}%")
    fcu.request_streams()
    last_request = time.time()
    while not rospy.is_shutdown():
        battery_ok = args.min_battery <= 0 or (tel.battery is not None and tel.battery >= args.min_battery)
        if tel.gps_ok and battery_ok:
            break
        if time.time() - last_request > 5:
            fcu.request_streams()  # re-ask while data is missing
            last_request = time.time()
        if log_every(5):
            batt = "not reported (use --min-battery 0 to skip)" if tel.battery is None else f"{tel.battery:.0f}%"
            rospy.loginfo(f"GPS status: {tel.gps_status} | battery: {batt}")
        rospy.sleep(0.2)
    if rospy.is_shutdown():
        return

    # ---------- 3. range check (wrong file / wrong site) ----------
    home_lat, home_lon = tel.lat, tel.lon
    far = max(haversine_m(home_lat, home_lon, it["lat"], it["lon"])
              for it in items if it["command"] in POSITION_COMMANDS)
    if far > args.max_range:
        raise MissionError(f"A waypoint is {far:.0f} m away (limit {args.max_range:.0f} m). "
                           "Wrong file or wrong take-off site?")
    length = path_length_m(home_lat, home_lon, items)
    timeout_s = (args.timeout_min * 60 if args.timeout_min
                 else length / 2.0 * 1.5 + 180)  # assumes >= 2 m/s, 50% margin + 3 min

    # ---------- 4. upload (row 0 = home placeholder at the real position) ----------
    home = dict(frame=FRAME_GLOBAL, command=CMD_NAV_WAYPOINT, params=(0, 0, 0, 0),
                lat=home_lat, lon=home_lon, alt=tel.amsl or 0.0, autocontinue=True)
    ros_wps = [to_ros_waypoint(home, current=True)] + [to_ros_waypoint(it) for it in items]
    last_seq = len(ros_wps) - 1
    if not fcu.upload(ros_wps):
        raise MissionError("Mission upload failed - not taking off")
    rospy.loginfo(f"Uploaded {len(items)} items | path {length / 1000:.2f} km | "
                  f"farthest {far:.0f} m | timeout {timeout_s / 60:.0f} min")

    # ---------- 5. confirmation ----------
    if not args.yes:
        answer = input(f"Take off to {takeoff_alt:.0f} m and fly the mission? Type 'fly' to continue: ")
        if answer.strip().lower() != "fly":
            rospy.loginfo("Cancelled by user")
            return

    # ---------- 6. GUIDED / ARM / TAKEOFF ----------
    flight = {"armed_by_us": False}
    try:
        if not fcu.set_mode("GUIDED"):
            raise MissionError("Could not switch to GUIDED")
        if not fcu.arm_for_flight(args.arm_timeout):
            raise MissionError("Arming failed - fix the pre-arm problem shown above")
        flight["armed_by_us"] = True
        rospy.loginfo("Armed")

        if not fcu.takeoff(takeoff_alt):
            fcu.arm(False)
            raise MissionError("Takeoff rejected - disarmed")
        rospy.loginfo(f"Taking off to {takeoff_alt:.0f} m")

        def climbed():
            if not tel.armed or tel.mode != "GUIDED":
                raise MissionError(f"Takeoff interrupted (armed={tel.armed}, mode={tel.mode}) - "
                                   "pilot or failsafe took control")
            return tel.rel_alt is not None and tel.rel_alt >= 0.9 * takeoff_alt
        if not wait_until(climbed, 30 + takeoff_alt * 2):
            fcu.set_mode("LAND")
            raise MissionError(f"Did not reach take-off altitude (now {tel.rel_alt} m) - LANDING")
        rospy.loginfo(f"Reached {tel.rel_alt:.1f} m")

        # ---------- 7. AUTO + monitoring ----------
        tel.last_reached = -1
        if not fcu.set_mode("AUTO"):
            fcu.set_mode("RTL")
            raise MissionError("Could not switch to AUTO - RTL")
        rospy.loginfo("Mission started")

        start = time.time()
        low_batt_since = None
        reason = None
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if not tel.armed:
                rospy.logwarn("Drone disarmed during mission - script stopping")
                return
            if tel.mode != "AUTO":
                rospy.logwarn(f"Mode changed to {tel.mode} by pilot/failsafe - script hands over and stops")
                return
            if tel.last_reached >= last_seq:
                rospy.loginfo("All waypoints completed")
                reason = "mission complete"
                break
            if args.rtl_battery > 0 and tel.battery is not None and tel.battery < args.rtl_battery:
                low_batt_since = low_batt_since or time.time()
                if time.time() - low_batt_since > 5:
                    reason = f"battery low ({tel.battery:.0f}%)"
                    break
            else:
                low_batt_since = None
            if time.time() - start > timeout_s:
                reason = "mission timeout"
                break
            if not tel.connected and log_every(5):
                rospy.logwarn("No telemetry from FCU - ArduPilot failsafes are in charge")
            if not tel.gps_ok and log_every(5):
                rospy.logwarn("GPS fix lost - ArduPilot GPS/EKF failsafe will act")
            if log_every(10):
                batt = f"{tel.battery:.0f}%" if tel.battery is not None else "?"
                rospy.loginfo(f"WP {max(tel.last_reached, 0)}/{last_seq} | alt {tel.rel_alt or 0:.1f} m | "
                              f"battery {batt}")
            rate.sleep()
        if rospy.is_shutdown():
            return

        # ---------- 8. RTL ----------
        rospy.loginfo(f"Returning home: {reason}")
        if not fcu.set_mode("RTL"):
            rospy.logerr("RTL rejected - trying LAND")
            fcu.set_mode("LAND")
        rospy.loginfo("Waiting for landing and disarm (Ctrl+C to stop waiting)...")
        wait_until(lambda: not tel.armed, 3600, hz=1)
        rospy.loginfo("Landed and disarmed. Mission finished.")

    except (KeyboardInterrupt, MissionError, Exception) as e:
        # Make the drone safe before leaving
        if flight["armed_by_us"] and tel.armed:
            if tel.rel_alt is not None and tel.rel_alt < 1.0 and tel.mode == "GUIDED":
                rospy.logwarn("Aborting on the ground - disarming")
                fcu.arm(False)
            elif tel.mode not in ("RTL", "LAND"):
                rospy.logwarn("Aborting in flight - RTL")
                if not fcu.set_mode("RTL"):
                    fcu.set_mode("LAND")
        if isinstance(e, KeyboardInterrupt):
            rospy.logwarn("Stopped by user")
            return
        raise


def main():
    p = argparse.ArgumentParser(description="Fly a .waypoints survey mission via MAVROS")
    p.add_argument("file", nargs="?", help=".waypoints file (file picker if omitted)")
    p.add_argument("--takeoff-alt", type=float, default=None, help="m, default = first waypoint altitude")
    p.add_argument("--min-battery", type=float, default=50, help="min batt%% to take off (0 = skip check)")
    p.add_argument("--rtl-battery", type=float, default=30, help="RTL below this battery %% (0 = off)")
    p.add_argument("--min-alt", type=float, default=5, help="lowest allowed waypoint altitude, m")
    p.add_argument("--max-alt", type=float, default=120, help="highest allowed waypoint altitude, m")
    p.add_argument("--max-range", type=float, default=1500, help="max waypoint distance from drone, m")
    p.add_argument("--timeout-min", type=float, default=None, help="RTL after this many minutes in AUTO")
    p.add_argument("--arm-timeout", type=float, default=60, help="keep retrying arming this long, s")
    p.add_argument("--yes", action="store_true", help="do not ask for confirmation before take-off")
    args = p.parse_args(rospy.myargv(argv=sys.argv)[1:])

    # disable_signals: Ctrl+C raises KeyboardInterrupt while ROS still works,
    # so the script can still send RTL / disarm before exiting.
    rospy.init_node("mission_runner", anonymous=True, disable_signals=True)
    try:
        run(args)
    except MissionError as e:
        rospy.logerr(str(e))
        sys.exit(1)
    finally:
        rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
