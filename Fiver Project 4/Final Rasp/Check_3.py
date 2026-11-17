#!/usr/bin/env python3
"""Check 3 - take off to 5 m, hover, land.

Order: wait GPS + battery -> GUIDED -> arm -> takeoff -> confirm altitude ->
hover -> LAND -> wait until disarmed. Every command is checked.
Ctrl+C in the air = LAND, on the ground = disarm."""
import argparse
import math
import sys
import time

import rospy
from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState

current_state = None
battery_percent = None
gps_fix = NavSatStatus.STATUS_NO_FIX
gps_pos = (None, None)
rel_alt = None


def state_cb(msg):
    global current_state
    current_state = msg


def battery_msg_type(timeout=10.0):
    """MAVROS publishes sensor_msgs/BatteryState (current) or mavros_msgs/BatteryStatus (old).
    Use whichever this drone's MAVROS publishes."""
    end = time.time() + timeout
    while time.time() < end and not rospy.is_shutdown():
        try:
            t = dict(rospy.get_published_topics()).get("/mavros/battery")
        except Exception:
            t = None
        if t == "mavros_msgs/BatteryStatus":
            from mavros_msgs.msg import BatteryStatus
            return BatteryStatus
        if t:
            return BatteryState
        rospy.sleep(0.5)
    return BatteryState


def battery_cb(msg):
    # BatteryState.percentage / BatteryStatus.remaining: 0..1 (NaN/negative if unknown)
    global battery_percent
    frac = msg.percentage if hasattr(msg, "percentage") else msg.remaining
    if not math.isnan(frac) and frac >= 0:
        battery_percent = frac * 100


def gps_cb(msg):
    global gps_fix, gps_pos
    gps_fix = msg.status.status  # -1 NO_FIX, 0 FIX, 1 SBAS, 2 GBAS/RTK
    gps_pos = (msg.latitude, msg.longitude)


def rel_alt_cb(msg):
    global rel_alt
    rel_alt = msg.data


def wait_until(pred, timeout):
    end = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < end:
        if pred():
            return True
        rospy.sleep(0.1)
    return pred()


def armed():
    return current_state is not None and current_state.armed


def mode():
    return current_state.mode if current_state is not None else ""


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--altitude", type=float, default=5.0)
    p.add_argument("--hover", type=float, default=10.0, help="hover time, s")
    p.add_argument("--min-battery", type=float, default=50, help="0 = skip battery check")
    p.add_argument("--yes", action="store_true", help="skip confirmation")
    args = p.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("takeoff_land_test", disable_signals=True)
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/battery", battery_msg_type(), battery_cb)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_cb)
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, rel_alt_cb)

    for s in ("/mavros/cmd/arming", "/mavros/set_mode", "/mavros/cmd/takeoff"):
        try:
            rospy.wait_for_service(s, timeout=30)
        except rospy.ROSException:
            sys.exit(f"{s} not available - is mavros running?")
    arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

    def set_mode(m):
        try:
            ok = mode_srv(custom_mode=m).mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"set_mode failed: {e}")
            return False
        return ok and wait_until(lambda: mode() == m, 10)

    def set_armed(v):
        try:
            ok = arming_srv(v).success
        except rospy.ServiceException as e:
            rospy.logerr(f"arming failed: {e}")
            return False
        return ok and wait_until(lambda: armed() == v, 10)

    if not wait_until(lambda: current_state is not None and current_state.connected, 60):
        sys.exit("No FCU connection after 60 s")
    if armed():
        sys.exit("Drone is already armed - refusing to run")

    rospy.loginfo("Waiting for GPS fix and battery...")
    stream_srv = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)
    while not rospy.is_shutdown():
        if gps_pos[0] is None or rel_alt is None:
            try:  # ArduPilot sends no GPS/battery data until a ground station asks for it
                stream_srv(stream_id=0, message_rate=10, on_off=True)
            except rospy.ServiceException:
                pass
        batt_ok = args.min_battery <= 0 or (battery_percent is not None and battery_percent >= args.min_battery)
        if gps_fix >= NavSatStatus.STATUS_FIX and batt_ok:
            break
        rospy.loginfo(f"GPS: {gps_fix} | Battery: {battery_percent}")
        rospy.sleep(1)

    if not args.yes and input(f"Take off to {args.altitude} m? Type 'yes': ").strip().lower() != "yes":
        sys.exit("Cancelled")

    try:
        if not set_mode("GUIDED"):
            sys.exit("Could not switch to GUIDED")
        if not set_armed(True):
            sys.exit("Arming failed - check pre-arm messages")
        rospy.loginfo("Armed")

        try:
            ok = takeoff_srv(min_pitch=0, yaw=float("nan"), latitude=gps_pos[0],
                             longitude=gps_pos[1], altitude=args.altitude).success
        except rospy.ServiceException:
            ok = False
        if not ok:
            set_armed(False)
            sys.exit("Takeoff rejected - disarmed")

        rospy.loginfo(f"Taking off to {args.altitude} m...")
        if not wait_until(lambda: rel_alt is not None and rel_alt >= 0.9 * args.altitude, 40):
            raise RuntimeError(f"Altitude not reached (now {rel_alt})")
        rospy.loginfo(f"At {rel_alt:.1f} m - hovering {args.hover:.0f} s")
        end = time.time() + args.hover
        while time.time() < end:
            if mode() != "GUIDED" or not armed():
                rospy.logwarn(f"Pilot/failsafe took control (mode {mode()}) - test stops")
                return
            rospy.sleep(0.2)

        rospy.loginfo("Landing...")
        if not set_mode("LAND"):
            raise RuntimeError("LAND mode rejected")
        wait_until(lambda: not armed(), 120)
        rospy.loginfo("Landed and disarmed. Test complete." if not armed()
                      else "Still armed after 120 s - check the drone!")
    except (KeyboardInterrupt, RuntimeError) as e:
        rospy.logwarn(f"Abort: {e or 'Ctrl+C'}")
        if armed():
            if rel_alt is not None and rel_alt < 1.0:
                set_armed(False)
            else:
                set_mode("LAND")
                rospy.logwarn("LAND commanded")
    finally:
        rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
