#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State, BatteryStatus
from sensor_msgs.msg import NavSatFix

# ---------------- GLOBALS ---------------- #
current_state = None
battery_percent = 0
gps_fix = 0

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

# ---------------- MAIN ---------------- #
def main():
    rospy.init_node("check_drone_status")

    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/battery", BatteryStatus, battery_cb)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_cb)

    rate = rospy.Rate(1)
    rospy.loginfo("Waiting for connection and GPS fix...")
    while not rospy.is_shutdown():
        if current_state is not None:
            rospy.loginfo(f"Connected to FCU: {current_state.connected}")
        rospy.loginfo(f"Battery: {battery_percent:.1f}% | GPS Fix: {gps_fix}")
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
