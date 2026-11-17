#!/usr/bin/env python3
"""Check 1 - status monitor: FCU connection, mode, armed, battery, GPS, altitude.
Safe to run any time: the only request it sends is "please send telemetry"
(ArduPilot sends no GPS/battery data until a ground station asks)."""
import math
import time

import rospy
from std_msgs.msg import Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate
from sensor_msgs.msg import NavSatFix, NavSatStatus, BatteryState

GPS_TEXT = {NavSatStatus.STATUS_NO_FIX: "NO FIX", NavSatStatus.STATUS_FIX: "FIX",
            NavSatStatus.STATUS_SBAS_FIX: "SBAS FIX", NavSatStatus.STATUS_GBAS_FIX: "RTK/GBAS FIX"}

current_state = None
battery_percent = None
battery_voltage = None
gps_fix = NavSatStatus.STATUS_NO_FIX
gps_pos = None
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
    global battery_percent, battery_voltage
    battery_voltage = msg.voltage
    frac = msg.percentage if hasattr(msg, "percentage") else msg.remaining
    if not math.isnan(frac) and frac >= 0:
        battery_percent = frac * 100


def gps_cb(msg):
    global gps_fix, gps_pos
    gps_fix = msg.status.status
    gps_pos = (msg.latitude, msg.longitude)


def rel_alt_cb(msg):
    global rel_alt
    rel_alt = msg.data


def main():
    rospy.init_node("check_drone_status")
    rospy.Subscriber("/mavros/state", State, state_cb)
    batt_type = battery_msg_type()
    rospy.loginfo(f"Battery message type: {batt_type._type}")
    rospy.Subscriber("/mavros/battery", batt_type, battery_cb)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_cb)
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, rel_alt_cb)

    stream_srv = rospy.ServiceProxy("/mavros/set_stream_rate", StreamRate)
    rate = rospy.Rate(1)
    ticks = 0
    while not rospy.is_shutdown():
        ticks += 1
        if current_state is not None and current_state.connected and gps_pos is None and ticks % 5 == 1:
            try:
                stream_srv(stream_id=0, message_rate=10, on_off=True)
            except rospy.ServiceException:
                pass
        if current_state is None:
            rospy.loginfo("Waiting for /mavros/state ... (is mavros running?)")
        else:
            batt = "not reported" if battery_percent is None else f"{battery_percent:.0f}%"
            volt = f" {battery_voltage:.2f} V" if battery_voltage else ""
            pos = f" @ {gps_pos[0]:.6f}, {gps_pos[1]:.6f}" if gps_pos else ""
            alt = f"{rel_alt:.1f} m" if rel_alt is not None else "?"
            rospy.loginfo(
                f"Connected: {current_state.connected} | Mode: {current_state.mode} | "
                f"Armed: {current_state.armed} | Battery: {batt}{volt} | "
                f"GPS: {GPS_TEXT.get(gps_fix, gps_fix)}{pos} | Alt: {alt}")
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
