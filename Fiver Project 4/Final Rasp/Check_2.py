#!/usr/bin/env python3
"""Check 2 - arm for 5 seconds, then disarm.

MOTORS WILL SPIN. Remove the propellers first. The script asks for
confirmation (skip with --yes). Ctrl+C disarms immediately."""
import argparse
import sys
import time

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool

current_state = None


def state_cb(msg):
    global current_state
    current_state = msg


def wait_until(pred, timeout):
    end = time.time() + timeout
    while not rospy.is_shutdown() and time.time() < end:
        if pred():
            return True
        rospy.sleep(0.1)
    return pred()


def set_armed(srv, value):
    try:
        resp = srv(value)
    except rospy.ServiceException as e:
        rospy.logerr(f"Arming service failed: {e}")
        return False
    if not resp.success:
        rospy.logerr(f"FCU rejected {'arm' if value else 'disarm'} (result {resp.result}) - "
                     "check pre-arm messages in Mission Planner / QGC")
        return False
    return wait_until(lambda: current_state is not None and current_state.armed == value, 5)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--yes", action="store_true", help="skip the propeller confirmation")
    p.add_argument("--seconds", type=float, default=5.0)
    args = p.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("arm_disarm_test", disable_signals=True)
    rospy.Subscriber("/mavros/state", State, state_cb)
    try:
        rospy.wait_for_service("/mavros/cmd/arming", timeout=30)
    except rospy.ROSException:
        sys.exit("MAVROS arming service not available - is mavros running?")
    arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)

    if not wait_until(lambda: current_state is not None and current_state.connected, 60):
        sys.exit("No FCU connection after 60 s")
    if current_state.armed:
        sys.exit("Drone is already armed - refusing to run")

    if not args.yes and input("PROPELLERS REMOVED? Type 'yes' to arm: ").strip().lower() != "yes":
        sys.exit("Cancelled")

    armed = False
    try:
        rospy.loginfo("Arming...")
        armed = set_armed(arming_srv, True)
        if not armed:
            sys.exit("Arming failed")
        rospy.loginfo(f"Armed - waiting {args.seconds:.0f} s")
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted")
    finally:
        if armed or (current_state is not None and current_state.armed):
            rospy.loginfo("Disarming...")
            if set_armed(arming_srv, False):
                rospy.loginfo("Disarmed. Test complete.")
            else:
                rospy.logerr("DISARM FAILED - disarm with the RC transmitter / GCS now!")
        rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
