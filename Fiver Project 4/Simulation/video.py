#!/usr/bin/env python3
"""
Record video from a camera to ~/sitl_videos (or --output).

Sources:
    opencv (default) - USB/V4L2 camera --index N
    ros              - ROS image topic, e.g. the Gazebo drone camera (--topic /webcam/image_raw)

The video size and FPS are taken from the real frames, so the file is never
empty/corrupt because of a size mismatch. Press q in the preview window or
Ctrl+C to stop. --no-display records without a preview window (headless).
"""
import argparse
import os
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np


class RosFrames:
    def __init__(self, topic):
        import rospy
        from sensor_msgs.msg import Image
        rospy.init_node("sim_video_recorder", anonymous=True, disable_signals=True)
        self.frame, self.stamp = None, 0.0
        self.lock = threading.Lock()
        rospy.Subscriber(topic, Image, self._cb, queue_size=1, buff_size=2 ** 24)

    def _cb(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.frame, self.stamp = img.copy(), time.time()

    def read(self):
        with self.lock:
            if self.frame is None or time.time() - self.stamp > 2.0:
                return False, None
            return True, self.frame.copy()

    def release(self):
        import rospy
        rospy.signal_shutdown("done")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", choices=["opencv", "ros"], default="opencv")
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--topic", default="/webcam/image_raw")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=float, default=0, help="0 = use the camera's FPS (or 30)")
    p.add_argument("--output", default=os.path.expanduser("~/sitl_videos"))
    p.add_argument("--no-display", action="store_true")
    args, _ = p.parse_known_args()

    os.makedirs(args.output, exist_ok=True)
    filename = os.path.join(args.output, f"video_{datetime.now():%Y%m%d_%H%M%S}.mp4")

    if args.source == "ros":
        cap = RosFrames(args.topic)
        fps = args.fps or 30.0
    else:
        cap = cv2.VideoCapture(args.index)
        if not cap.isOpened():
            sys.exit(f"ERROR: camera {args.index} not found")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cam_fps = cap.get(cv2.CAP_PROP_FPS)
        fps = args.fps or (cam_fps if 1 <= cam_fps <= 240 else 30.0)

    # Use the real frame size for the writer
    deadline = time.time() + 15
    ok, frame = cap.read()
    while not ok and time.time() < deadline:
        time.sleep(0.1)
        ok, frame = cap.read()
    if not ok:
        cap.release()
        sys.exit("ERROR: no frames from the camera")
    h, w = frame.shape[:2]

    out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
    if not out.isOpened():
        cap.release()
        sys.exit("ERROR: cannot create the video file")

    print(f"Recording {w}x{h} @ {fps:.0f} fps to {filename} - press q (or Ctrl+C) to stop")
    frames = 0
    period = 1.0 / fps
    next_t = time.time()
    try:
        while True:
            if frame.shape[:2] != (h, w):
                frame = cv2.resize(frame, (w, h))
            out.write(frame)
            frames += 1
            if not args.no_display:
                cv2.imshow("Drone camera", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if args.source == "ros":  # keep real-time pacing for topic input
                next_t += period
                time.sleep(max(0.0, next_t - time.time()))
            ok, new = cap.read()
            if not ok:
                if args.source == "ros":
                    continue  # keep last frame while the topic is briefly quiet
                print("Camera stopped delivering frames")
                break
            frame = new
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        out.release()
        cv2.destroyAllWindows()
    print(f"Saved {frames} frames ({frames / fps:.1f} s) to {filename}")


if __name__ == "__main__":
    main()
