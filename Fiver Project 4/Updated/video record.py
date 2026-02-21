import cv2
import os
import time
from datetime import datetime
from tkinter import Tk
from tkinter.filedialog import askdirectory

# ------------------- OUTPUT FOLDER GUI ------------------- #
def select_output_folder():
    Tk().withdraw()
    folder = askdirectory(title="Select folder to save images")
    if not folder:
        print("❌ No folder selected. Exiting.")
        exit()
    return folder

# ------------------- IMAGE FILENAME ------------------- #
def get_image_filename(output_dir):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(output_dir, f"image_{timestamp}.jpg")

# ------------------- CAMERA SETUP ------------------- #
camera_index = 0  # IMX477 camera
frame_width = 1920
frame_height = 1080

cap = cv2.VideoCapture(camera_index)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

# ------------------- MAIN LOOP ------------------- #
output_folder = select_output_folder()
print(f"📂 Saving images to: {output_folder}")
print("📸 Capturing images every 2 seconds... Press Ctrl+C to stop.")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("⚠️ Failed to capture frame")
            continue

        filename = get_image_filename(output_folder)
        cv2.imwrite(filename, frame)
        print(f"✅ Saved: {filename}")

        time.sleep(2)  # wait 2 seconds before next capture

except KeyboardInterrupt:
    print("\n🛑 Stopping capture...")

finally:
    cap.release()
    print("✅ Camera released. Program ended.")
