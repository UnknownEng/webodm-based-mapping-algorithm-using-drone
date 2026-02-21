#!/usr/bin/env python3
import rospy
import time

from mavros_msgs.msg import State, BatteryStatus, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint
from sensor_msgs.msg import NavSatFix
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# ---------------- GLOBAL VARIABLES ---------------- #
current_state = None
battery_percent = 0
gps_fix = 0
last_wp_reached = -1
total_wps = 0

# ---------------- CALLBACKS ---------------- #
def state_cb(msg):
    global current_state
    current_state = msg

def battery_cb(msg):
    global battery_percent
    if msg.percentage >= 0:
        battery_percent = msg.percentage * 100

def gps_cb(msg):
    global gps_fix
    gps_fix = msg.status.status  # 2 = 3D FIX

def wp_reached_cb(msg):
    global last_wp_reached
    last_wp_reached = msg.wp_seq
    rospy.loginfo(f"Waypoint reached: {msg.wp_seq}")

# ---------------- FILE PICKER ---------------- #
def select_waypoint_file():
    Tk().withdraw()
    return askopenfilename(
        title="Select .waypoints file",
        filetypes=[("QGC Waypoints", "*.waypoints")]
    )

# ---------------- MAIN ---------------- #
def main():
    global total_wps

    rospy.init_node("mission_runner", anonymous=True)

    # ---------------- SUBSCRIBERS (ALL REAL MAVROS TOPICS) ---------------- #
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/battery", BatteryStatus, battery_cb)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_cb)
    rospy.Subscriber("/mavros/mission/reached", WaypointReached, wp_reached_cb)

    # ---------------- SERVICES (ALL REAL MAVROS SERVICES) ---------------- #
    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    rospy.wait_for_service("/mavros/cmd/takeoff")
    rospy.wait_for_service("/mavros/mission/clear")
    rospy.wait_for_service("/mavros/mission/push")

    arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
    wp_clear_srv = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
    wp_push_srv = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)

    # ---------------- SELECT WAYPOINT FILE ---------------- #
    wp_file = select_waypoint_file()
    if not wp_file:
        rospy.logerr("No waypoint file selected")
        return

    # ---------------- WAIT FOR GPS & BATTERY ---------------- #
    rospy.loginfo("Waiting for GPS fix and battery >= 50%")
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        if gps_fix >= 2 and battery_percent >= 50:
            break
        rate.sleep()

    # ---------------- GUIDED / ARM / TAKEOFF ---------------- #
    mode_srv(custom_mode="GUIDED")
    time.sleep(2)

    arming_srv(True)
    time.sleep(2)

    takeoff_srv(altitude=30)
    time.sleep(8)

    # ---------------- LOAD WAYPOINTS ---------------- #
    wp_clear_srv()
    waypoints = []

    with open(wp_file) as f:
        lines = f.readlines()

    for line in lines[1:]:
        t = line.strip().split("\t")
        if len(t) >= 12:
            wp = Waypoint()
            wp.frame = 3  # GLOBAL_REL_ALT
            wp.command = 16  # NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.param1 = wp.param2 = wp.param3 = wp.param4 = 0
            wp.x_lat = float(t[8])
            wp.y_long = float(t[9])
            wp.z_alt = float(t[10])
            waypoints.append(wp)

    total_wps = len(waypoints)
    wp_push_srv(waypoints)
    rospy.loginfo(f"{total_wps} waypoints uploaded")

    # ---------------- AUTO MODE ---------------- #
    mode_srv(custom_mode="AUTO")
    rospy.loginfo("Mission started")

    # ---------------- WAIT UNTIL LAST WAYPOINT ---------------- #
    while not rospy.is_shutdown():
        if last_wp_reached == total_wps - 1:
            rospy.loginfo("All waypoints completed")
            break
        rate.sleep()

    # ---------------- RTL ---------------- #
    mode_srv(custom_mode="RTL")
    rospy.loginfo("RTL activated")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
