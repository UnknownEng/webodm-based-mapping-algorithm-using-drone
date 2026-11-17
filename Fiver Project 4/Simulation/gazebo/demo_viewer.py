#!/usr/bin/env python3
"""Live drone-camera window with a HUD (mode, altitude, GPS, battery, photo count).
Used for the demo video; any ROS 2 image topic works.

    python3 demo_viewer.py --topic /survey_camera/image --photos ~/survey_sim/photos
"""
import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROS2_DIR = next(p for p in (os.path.join(os.environ.get("PROJECT_DIR", ""), "ROS2"),
                             os.path.join(HERE, "..", "..", "ROS2")) if os.path.isdir(p))
sys.path.insert(0, ROS2_DIR)
from mavros_link import MavrosLink  # noqa: E402
import importlib.util  # noqa: E402
_spec = importlib.util.spec_from_file_location("ros2_images", os.path.join(ROS2_DIR, "images.py"))
_ros2_images = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_ros2_images)
Ros2Camera = _ros2_images.Ros2Camera


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/survey_camera/image")
    p.add_argument("--photos", default=os.path.expanduser("~/survey_sim/photos"))
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--x", type=int, default=1280)
    p.add_argument("--y", type=int, default=0)
    args, _ = p.parse_known_args()

    link = MavrosLink("demo_viewer")
    cam = Ros2Camera(link.node, args.topic)
    win = "Drone camera (live)"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)  # no Qt toolbar
    cv2.moveWindow(win, args.x, args.y)
    last_count, count = 0.0, 0
    try:
        while True:
            frame = cam.read()
            h = int(args.width * 3 / 4)
            if frame is None:
                frame = np.full((h, args.width, 3), 30, np.uint8)
                cv2.putText(frame, "waiting for camera...", (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
            else:
                frame = cv2.resize(frame, (args.width, h), interpolation=cv2.INTER_AREA)
            if time.time() - last_count > 1:
                count = len(glob.glob(os.path.join(args.photos, "*.jpg")))
                last_count = time.time()
            batt = f"{link.battery:.0f}%" if link.battery is not None else "?"
            alt = f"{link.rel_alt:5.1f} m" if link.rel_alt is not None else "  ?"
            gps = "3D fix" if link.gps_ok else "no fix"
            lines = [f"{link.mode or '---':8s} {'ARMED' if link.armed else 'disarmed'}",
                     f"ALT {alt}   GPS {gps}   BATT {batt}",
                     f"PHOTOS {count}"]
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (args.width, 78), (0, 0, 0), -1)
            frame = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)
            for i, t in enumerate(lines):
                cv2.putText(frame, t, (10, 22 + i * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 255, 120), 1, cv2.LINE_AA)
            # crosshair
            cx, cy = args.width // 2, h // 2
            cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (255, 255, 255), 1)
            cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (255, 255, 255), 1)
            cv2.imshow(win, frame)
            if cv2.waitKey(66) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        link.shutdown()


if __name__ == "__main__":
    main()
