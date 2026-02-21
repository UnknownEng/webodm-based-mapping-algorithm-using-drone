#!/usr/bin/env python3
import rospy
import time
import sys

from mavros_msgs.msg import State, WaypointReached
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint

current_state = None
last_wp_reached = -1
total_wps = 0

# ---------- Callbacks ----------
def state_cb(msg):
    global current_state
    current_state = msg

def wp_reached_cb(msg):
    global last_wp_reached
    last_wp_reached = msg.wp_seq
    rospy.loginfo(f"Reached WP {msg.wp_seq}")

# ---------- Main ----------
def main():

    global total_wps

    if len(sys.argv) < 2:
        print("Usage: mission_sitl.py mission.waypoints")
        return

    wp_file = sys.argv[1]

    rospy.init_node("mission_runner_sitl")

    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/mission/reached", WaypointReached, wp_reached_cb)

    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    rospy.wait_for_service("/mavros/cmd/takeoff")
    rospy.wait_for_service("/mavros/mission/clear")
    rospy.wait_for_service("/mavros/mission/push")

    arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    takeoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
    wp_clear = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
    wp_push = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)

    rate = rospy.Rate(2)

    # ---------- Wait for FCU ----------
    rospy.loginfo("Waiting for FCU connection...")
    while not rospy.is_shutdown() and (current_state is None or not current_state.connected):
        rate.sleep()

    rospy.loginfo("Connected to SITL")

    # ---------- GUIDED ----------
    set_mode(custom_mode="GUIDED")
    time.sleep(2)

    # ---------- ARM ----------
    arm(True)
    time.sleep(2)

    # ---------- TAKEOFF ----------
    rospy.loginfo("Taking off")
    takeoff(altitude=20)
    time.sleep(10)

    # ---------- LOAD WAYPOINTS ----------
    rospy.loginfo("Loading waypoints")
    wp_clear()

    waypoints = []
    with open(wp_file) as f:
        lines = f.readlines()

    for line in lines[1:]:
        t = line.strip().split("\t")
        if len(t) >= 12:
            wp = Waypoint()
            wp.frame = 3
            wp.command = 16
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = float(t[8])
            wp.y_long = float(t[9])
            wp.z_alt = float(t[10])
            waypoints.append(wp)

    total_wps = len(waypoints)
    wp_push(waypoints)

    rospy.loginfo(f"Uploaded {total_wps} WPs")

    # ---------- AUTO ----------
    set_mode(custom_mode="AUTO")
    rospy.loginfo("Mission running")

    while not rospy.is_shutdown():
        if last_wp_reached == total_wps - 1:
            rospy.loginfo("Mission complete")
            break
        rate.sleep()

    set_mode(custom_mode="RTL")
    rospy.loginfo("RTL")

if __name__ == "__main__":
    main()
