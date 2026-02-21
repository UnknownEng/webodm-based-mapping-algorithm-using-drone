#!/usr/bin/env python3
import rospy
import time
from mavros_msgs.srv import CommandBool
from mavros_msgs.msg import State

current_state = None

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node("arm_disarm_test")

    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    # Wait for connection
    while current_state is None or not current_state.connected:
        rospy.loginfo("Waiting for FCU connection...")
        rospy.sleep(1)

    rospy.loginfo("Arming drone...")
    arming_srv(True)
    time.sleep(5)

    rospy.loginfo("Disarming drone...")
    arming_srv(False)
    rospy.loginfo("Test complete.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
