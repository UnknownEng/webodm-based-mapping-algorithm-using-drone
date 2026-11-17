#!/usr/bin/env python3
"""
Survey camera: takes a photo every N metres while the drone flies its AUTO
mission and geotags each photo for WebODM.

* Position comes from MAVROS (/mavros/global_position/global + rel_alt).
* GPS is written into the JPEG EXIF (needs `pip install piexif`) AND into a
  `geo.txt` file in the output folder (WebODM/ODM geolocation file format) -
  upload geo.txt together with the photos.
* Camera sources:
    picamera2  - Raspberry Pi HQ camera (IMX477) on Raspberry Pi OS (default if installed)
    opencv     - USB / V4L2 camera
    ros        - a ROS image topic, e.g. the Gazebo simulated camera
* Without ROS (--no-ros) it falls back to a photo every --interval seconds,
  without geotags.

Usage on the drone (mission running in another terminal):
    python3 images.py --output ~/survey/block_1 --trigger-distance 7.9
Simulation (Gazebo camera):
    python3 images.py --source ros --image-topic /webcam/image_raw --output ~/sim_photos
"""
import argparse
import math
import os
import shutil
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

try:
    import piexif
except ImportError:
    piexif = None

MIN_FREE_MB = 300


# ======================= CAMERA SOURCES ======================= #
class PiCamera:
    """Raspberry Pi camera via picamera2 / libcamera (IMX477)."""

    def __init__(self, width, height):
        from picamera2 import Picamera2
        self.cam = Picamera2()
        # "RGB888" in picamera2 is BGR byte order = what OpenCV expects
        cfg = self.cam.create_still_configuration(main={"size": (width, height), "format": "RGB888"})
        self.cam.configure(cfg)
        self.cam.start()
        time.sleep(2)  # let auto-exposure settle

    def read(self):
        return self.cam.capture_array("main")  # always a fresh frame

    def close(self):
        self.cam.stop()


class OpenCvCamera:
    """USB/V4L2 camera (the original project's working setup: cv2.VideoCapture(0)).

    Tries the requested size, then smaller common IMX477 sizes, and keeps the
    largest one the driver really delivers (an unsupported size can silently
    fall back to 640x480, which ruins a map). A background thread keeps only
    the newest frame, so a capture never returns an old buffered frame."""

    SIZES = [(4056, 3040), (3840, 2160), (2592, 1944), (2028, 1520), (1920, 1080), (1280, 720)]

    def __init__(self, index, width, height):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera {index}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        tried = [(width, height)] + [s for s in self.SIZES if s != (width, height) and s[0] <= width]
        self.size = None
        for w, h in tried:
            for fourcc in ("MJPG", None):  # MJPG lets USB/V4L2 cameras deliver large frames
                if fourcc:
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                ok, frame = False, None
                for _ in range(5):  # first frames after a mode change can be empty
                    ok, frame = self.cap.read()
                    if ok:
                        break
                if ok and frame.shape[1] == w and frame.shape[0] == h:
                    self.size = (w, h)
                    break
            if self.size:
                break
        if self.size is None:
            ok, frame = self.cap.read()
            if not ok:
                raise RuntimeError(f"Camera {index} delivers no frames")
            self.size = (frame.shape[1], frame.shape[0])
        print(f"OpenCV camera {index}: using {self.size[0]}x{self.size[1]}")
        if self.size[0] * self.size[1] < 1920 * 1080:
            print("WARNING: below 2 MP - the WebODM map will be coarse")
        self.frame, self.stamp = None, 0.0
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            ok, frame = self.cap.read()
            if ok:
                with self.lock:
                    self.frame, self.stamp = frame, time.time()
            else:
                time.sleep(0.05)

    def read(self):
        with self.lock:
            if self.frame is None or time.time() - self.stamp > 1.0:
                return None
            return self.frame.copy()

    def close(self):
        self.running = False
        self.cap.release()


class RosCamera:
    """sensor_msgs/Image topic (e.g. Gazebo camera). No cv_bridge needed."""

    def __init__(self, topic):
        import rospy
        from sensor_msgs.msg import Image
        self.frame, self.stamp = None, 0.0
        self.lock = threading.Lock()
        rospy.Subscriber(topic, Image, self._cb, queue_size=1, buff_size=2 ** 24)

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


def open_camera(args):
    if args.source in ("auto", "picamera2"):
        try:
            return PiCamera(args.width, args.height)
        except Exception as e:
            if args.source == "picamera2":
                raise
            print(f"picamera2 not available ({e.__class__.__name__}) - using OpenCV camera {args.index}")
    if args.source == "ros":
        return RosCamera(args.image_topic)
    return OpenCvCamera(args.index, args.width, args.height)


# ======================= POSITION (MAVROS) ======================= #
class Position:
    def __init__(self):
        import rospy
        from std_msgs.msg import Float64
        from mavros_msgs.msg import State
        from sensor_msgs.msg import NavSatFix
        self.lat = self.lon = self.amsl = self.rel_alt = None
        self.fix = -1
        self.armed = False
        self.mode = ""
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._gps)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self._alt)
        rospy.Subscriber("/mavros/state", State, self._state)

    def _gps(self, m):
        self.fix = m.status.status
        if not math.isnan(m.latitude):
            self.lat, self.lon, self.amsl = m.latitude, m.longitude, m.altitude

    def _alt(self, m):
        self.rel_alt = m.data

    def _state(self, m):
        self.armed, self.mode = m.armed, m.mode


def haversine_m(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    a = (math.sin((p2 - p1) / 2) ** 2 +
         math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2)
    return 2 * 6371000.0 * math.asin(math.sqrt(a))


# ======================= SAVING ======================= #
def _dms(value):
    value = abs(value)
    d = int(value)
    m_float = (value - d) * 60
    m = int(m_float)
    s = round((m_float - m) * 60 * 10000)
    return ((d, 1), (m, 1), (s, 10000))


def build_exif(when, focal_mm, lat=None, lon=None, alt=None, sensor_w_mm=6.287):
    if piexif is None:
        return None
    zeroth = {piexif.ImageIFD.Make: b"RaspberryPi", piexif.ImageIFD.Model: b"IMX477",
              piexif.ImageIFD.DateTime: when.strftime("%Y:%m:%d %H:%M:%S").encode()}
    # FocalLengthIn35mmFilm tells WebODM/OpenSfM the real field of view
    # (focal ratio = f35 / 36). Without it WebODM guesses and the 3D model warps.
    exif = {piexif.ExifIFD.DateTimeOriginal: when.strftime("%Y:%m:%d %H:%M:%S").encode(),
            piexif.ExifIFD.FocalLength: (int(focal_mm * 100), 100),
            piexif.ExifIFD.FocalLengthIn35mmFilm: int(round(focal_mm * 36.0 / sensor_w_mm))}
    gps = {}
    if lat is not None and lon is not None:
        gps = {piexif.GPSIFD.GPSVersionID: (2, 3, 0, 0),
               piexif.GPSIFD.GPSLatitudeRef: b"N" if lat >= 0 else b"S",
               piexif.GPSIFD.GPSLatitude: _dms(lat),
               piexif.GPSIFD.GPSLongitudeRef: b"E" if lon >= 0 else b"W",
               piexif.GPSIFD.GPSLongitude: _dms(lon)}
        if alt is not None:
            gps[piexif.GPSIFD.GPSAltitudeRef] = 0 if alt >= 0 else 1
            gps[piexif.GPSIFD.GPSAltitude] = (int(abs(alt) * 100), 100)
    return piexif.dump({"0th": zeroth, "Exif": exif, "GPS": gps})


class Saver:
    def __init__(self, folder, quality, focal_mm, sensor_w_mm=6.287):
        self.folder = folder
        self.quality = quality
        self.focal_mm = focal_mm
        self.sensor_w_mm = sensor_w_mm
        self.count = 0
        os.makedirs(folder, exist_ok=True)
        self.geo_path = os.path.join(folder, "geo.txt")
        if not os.path.exists(self.geo_path):
            with open(self.geo_path, "w") as f:
                f.write("EPSG:4326\n")

    def disk_ok(self):
        return shutil.disk_usage(self.folder).free / 1e6 > MIN_FREE_MB

    def save(self, frame, lat=None, lon=None, alt=None):
        when = datetime.now()
        self.count += 1
        name = f"image_{when:%Y%m%d_%H%M%S}_{self.count:05d}.jpg"
        path = os.path.join(self.folder, name)
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        data = jpg.tobytes()
        exif = build_exif(when, self.focal_mm, lat, lon, alt, self.sensor_w_mm)
        if exif:
            import io
            out = io.BytesIO()
            piexif.insert(exif, data, out)
            data = out.getvalue()
        tmp = path + ".part"
        with open(tmp, "wb") as f:
            f.write(data)
        os.replace(tmp, path)
        if lat is not None:
            with open(self.geo_path, "a") as f:
                f.write(f"{name} {lon:.8f} {lat:.8f} {alt if alt is not None else 0:.2f}\n")
        return path


# ======================= MAIN ======================= #
def pick_folder():
    if os.name != "nt" and not os.environ.get("DISPLAY"):
        return None
    from tkinter import Tk
    from tkinter.filedialog import askdirectory
    Tk().withdraw()
    return askdirectory(title="Select folder to save images") or None


def main():
    p = argparse.ArgumentParser(description="Distance-triggered, geotagged survey camera")
    p.add_argument("--output", help="output folder (default ~/survey_images/<date_time>)")
    p.add_argument("--pick", action="store_true", help="choose the output folder with a dialog")
    p.add_argument("--trigger-distance", type=float, default=8.0,
                   help="metres between photos (use the value printed by path_planner.py)")
    p.add_argument("--interval", type=float, default=2.0, help="seconds between photos in --no-ros mode")
    p.add_argument("--min-alt", type=float, default=5.0, help="no photos below this altitude, m")
    p.add_argument("--always", action="store_true",
                   help="capture whenever GPS is OK - any mode, disarmed, on the ground (camera/GPS walk test)")
    p.add_argument("--no-ros", action="store_true", help="time-based capture without geotags")
    p.add_argument("--source", choices=["auto", "picamera2", "opencv", "ros"], default="auto")
    p.add_argument("--index", type=int, default=0, help="OpenCV camera index")
    p.add_argument("--image-topic", default="/webcam/image_raw", help="ROS image topic for --source ros")
    p.add_argument("--width", type=int, default=4056)
    p.add_argument("--height", type=int, default=3040)
    p.add_argument("--quality", type=int, default=95)
    p.add_argument("--focal-mm", type=float, default=6.0, help="lens focal length in mm (written to EXIF for WebODM)")
    p.add_argument("--sensor-w-mm", type=float, default=6.287, help="sensor width in mm (IMX477 = 6.287)")
    args, _ = p.parse_known_args()

    folder = args.output or (pick_folder() if args.pick else None) or os.path.expanduser(
        os.path.join("~", "survey_images", datetime.now().strftime("%Y%m%d_%H%M%S")))

    use_ros = not args.no_ros
    if use_ros or args.source == "ros":
        try:
            import rospy
            rospy.init_node("survey_camera", anonymous=True, disable_signals=True)
        except ImportError:
            if args.source == "ros":
                sys.exit("--source ros needs ROS (rospy)")
            print("rospy not found - falling back to time-based capture WITHOUT geotags")
            use_ros = False

    cam = open_camera(args)
    saver = Saver(folder, args.quality, args.focal_mm, args.sensor_w_mm)
    pos = Position() if use_ros else None
    if piexif is None:
        print("piexif not installed - GPS goes to geo.txt only (pip install piexif for EXIF tags)")
    print(f"Saving to {folder}")
    print(f"Mode: {'every %.1f m while in AUTO' % args.trigger_distance if use_ros else 'every %.1f s, no GPS' % args.interval}"
          " - Ctrl+C to stop")

    last_pos = None
    last_time = 0.0
    last_status = 0.0
    try:
        while True:
            if not saver.disk_ok():
                print(f"Less than {MIN_FREE_MB} MB free - capture stopped")
                break

            if use_ros:
                flying = args.always or (pos.armed and pos.mode == "AUTO")
                have_fix = pos.fix >= 0 and pos.lat is not None
                high = args.always or (pos.rel_alt is not None and pos.rel_alt >= args.min_alt)
                if not (flying and have_fix and high):
                    if time.time() - last_status > 10:
                        print(f"Waiting: armed={pos.armed} mode={pos.mode or '?'} fix={pos.fix} "
                              f"alt={pos.rel_alt}")
                        last_status = time.time()
                    last_pos = None  # start a fresh sequence on the next line/flight
                    time.sleep(0.1)
                    continue
                here = (pos.lat, pos.lon)
                due = last_pos is None or haversine_m(*last_pos, *here) >= args.trigger_distance
            else:
                due = time.time() - last_time >= args.interval

            if not due:
                time.sleep(0.02)
                continue

            frame = cam.read()
            if frame is None:
                print("No camera frame - retrying")
                time.sleep(0.5)
                continue

            if use_ros:
                path = saver.save(frame, pos.lat, pos.lon, pos.amsl)
                last_pos = here
                print(f"Saved {os.path.basename(path)} @ {pos.lat:.6f}, {pos.lon:.6f}, {pos.rel_alt or 0:.1f} m")
            else:
                path = saver.save(frame)
                print(f"Saved {os.path.basename(path)}")
            last_time = time.time()
    except KeyboardInterrupt:
        print("\nStopping capture...")
    finally:
        cam.close()
        print(f"{saver.count} photos saved in {folder}")
        if use_ros:
            import rospy
            rospy.signal_shutdown("done")


if __name__ == "__main__":
    main()
