#!/usr/bin/env python3
import rospy
import time
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State, BatteryStatus
from sensor_msgs.msg import NavSatFix

# ---------------- GLOBALS ---------------- #
current_state = None
battery_percent = 0
gps_fix = 0

def state_cb(msg):
    global current_state
    current_state = msg

def battery_cb(msg):
    global battery_percent
    if msg.percentage >= 0:
        battery_percent = msg.percentage * 100

def gps_cb(msg):
    global gps_fix
    gps_fix = msg.status.status

def main():
    rospy.init_node("takeoff_land_test")

    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/battery", BatteryStatus, battery_cb)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_cb)

    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    rospy.wait_for_service("/mavros/cmd/takeoff")

    arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

    rate = rospy.Rate(1)

    # Wait for connection
    while current_state is None or not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rate.sleep()

    # Wait for GPS and battery
    while gps_fix < 2 or battery_percent < 50:
        rospy.loginfo(f"GPS Fix: {gps_fix}, Battery: {battery_percent:.1f}%")
        rate.sleep()

    rospy.loginfo("Arming drone...")
    arming_srv(True)
    time.sleep(2)

    rospy.loginfo("Switching to GUIDED mode...")
    mode_srv(custom_mode="GUIDED")
    time.sleep(2)

    rospy.loginfo("Taking off to 5 meters...")
    takeoff_srv(altitude=5)
    time.sleep(10)  # hover

    rospy.loginfo("Landing...")
    mode_srv(custom_mode="LAND")
    rospy.loginfo("Test complete.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
