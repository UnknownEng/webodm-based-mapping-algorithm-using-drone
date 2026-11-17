#!/usr/bin/env python3
"""
ROS 2 version of the survey camera: a photo every N metres while the drone is
in AUTO, geotagged (EXIF GPS + geo.txt) for WebODM.

Photo saving / EXIF / Pi camera / USB camera code is shared with
Final Rasp/images.py (keep both folders together).

    python3 images.py --output ~/survey/block_1 --trigger-distance 7.9                    # Pi / USB camera
    python3 images.py --source ros --image-topic /camera/image --output ~/sim_photos     # Gazebo camera
"""
import argparse
import os
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mavros_link import MavrosLink  # noqa: E402


def _load_core():
    """Load Final Rasp/images.py under its own name (both files are called images.py)."""
    import importlib.util
    path = os.path.join(HERE, "..", "Final Rasp", "images.py")
    spec = importlib.util.spec_from_file_location("survey_camera_core", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_core = _load_core()
Saver, PiCamera, OpenCvCamera = _core.Saver, _core.PiCamera, _core.OpenCvCamera
haversine_m, piexif, MIN_FREE_MB = _core.haversine_m, _core.piexif, _core.MIN_FREE_MB


class Ros2Camera:
    """sensor_msgs/Image topic (e.g. Gazebo camera bridged with ros_gz_bridge)."""

    def __init__(self, node, topic):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        self.frame, self.stamp = None, 0.0
        self.lock = threading.Lock()
        node.create_subscription(Image, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ("rgb8", "bgr8"):
            img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding in ("mono8", "8UC1"):
            img = cv2.cvtColor(buf.reshape(msg.height, msg.step)[:, :msg.width], cv2.COLOR_GRAY2BGR)
        else:
            return
        with self.lock:
            self.frame, self.stamp = img.copy(), time.time()

    def read(self):
        with self.lock:
            if self.frame is None or time.time() - self.stamp > 1.0:
                return None
            return self.frame.copy()

    def close(self):
        pass


def main():
    p = argparse.ArgumentParser(description="Distance-triggered, geotagged survey camera (ROS 2)")
    p.add_argument("--output", help="output folder (default ~/survey_images/<date_time>)")
    p.add_argument("--trigger-distance", type=float, default=8.0)
    p.add_argument("--min-alt", type=float, default=5.0)
    p.add_argument("--always", action="store_true",
                   help="capture whenever GPS is OK - any mode, disarmed, on the ground (camera/GPS walk test)")
    p.add_argument("--source", choices=["auto", "picamera2", "opencv", "ros"], default="auto")
    p.add_argument("--index", type=int, default=0)
    p.add_argument("--image-topic", default="/camera/image")
    p.add_argument("--width", type=int, default=4056)
    p.add_argument("--height", type=int, default=3040)
    p.add_argument("--quality", type=int, default=95)
    p.add_argument("--focal-mm", type=float, default=6.0, help="lens focal length in mm (written to EXIF for WebODM)")
    p.add_argument("--sensor-w-mm", type=float, default=6.287, help="sensor width in mm (IMX477 = 6.287)")
    args, _ = p.parse_known_args()

    folder = args.output or os.path.expanduser(
        os.path.join("~", "survey_images", datetime.now().strftime("%Y%m%d_%H%M%S")))
    link = MavrosLink("survey_camera")

    if args.source == "ros":
        cam = Ros2Camera(link.node, args.image_topic)
    else:
        cam = None
        if args.source in ("auto", "picamera2"):
            try:
                cam = PiCamera(args.width, args.height)
            except Exception as e:
                if args.source == "picamera2":
                    raise
                print(f"picamera2 not available ({e.__class__.__name__}) - using OpenCV camera {args.index}")
        cam = cam or OpenCvCamera(args.index, args.width, args.height)

    saver = Saver(folder, args.quality, args.focal_mm, args.sensor_w_mm)
    if piexif is None:
        print("piexif not installed - GPS goes to geo.txt only")
    link.info(f"Saving to {folder} - photo every {args.trigger_distance:.1f} m while in AUTO (Ctrl+C to stop)")

    last_pos, last_status = None, 0.0
    try:
        while True:
            if not saver.disk_ok():
                link.error(f"Less than {MIN_FREE_MB} MB free - capture stopped")
                break
            flying = args.always or (link.armed and link.mode == "AUTO")
            high = args.always or (link.rel_alt is not None and link.rel_alt >= args.min_alt)
            if not (flying and link.gps_ok and high):
                if time.time() - last_status > 10:
                    link.info(f"Waiting: armed={link.armed} mode={link.mode or '?'} "
                              f"gps={link.gps_status} alt={link.rel_alt}")
                    last_status = time.time()
                last_pos = None
                time.sleep(0.1)
                continue
            here = (link.lat, link.lon)
            if last_pos is not None and haversine_m(*last_pos, *here) < args.trigger_distance:
                time.sleep(0.02)
                continue
            frame = cam.read()
            if frame is None:
                link.warn("No camera frame - retrying")
                time.sleep(0.5)
                continue
            path = saver.save(frame, link.lat, link.lon, link.amsl)
            last_pos = here
            link.info(f"Photo {saver.count}: {os.path.basename(path)} @ {link.lat:.6f}, {link.lon:.6f}, "
                      f"{link.rel_alt or 0:.1f} m")
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        link.info(f"{saver.count} photos saved in {folder}")
        link.shutdown()


if __name__ == "__main__":
    main()
