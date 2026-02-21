#!/usr/bin/env python3
import cv2
import os
from datetime import datetime

# ---------- Video directory ----------
video_dir = os.path.expanduser("~/sitl_videos")
os.makedirs(video_dir, exist_ok=True)

def get_video_filename():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(video_dir, f"video_{ts}.mp4")

# ---------- Camera ----------
camera_index = 0   # change if needed
width = 1280
height = 720
fps = 30

cap = cv2.VideoCapture(camera_index)

if not cap.isOpened():
    print("ERROR: Camera not found")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter(get_video_filename(), fourcc, fps, (width, height))

print("Recording… press q to stop")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    out.write(frame)
    cv2.imshow("SITL Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()

print("Saved.")
