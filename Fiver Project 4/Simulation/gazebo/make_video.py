#!/usr/bin/env python3
"""
Edit the demo recording into the final video.

Inputs (in $SIM_DIR, made by record_demo.sh):
  video/demo_raw.mp4 + video/t_* timestamps, area.tif, plans/block_1.waypoints,
  photos/, results/{orthophoto.tif, dsm.tif, textured_model.zip}
Output:
  video/survey_demo.mp4  (1920x1080, 30 fps)

    python3 make_video.py [--sim-dir ~/survey_sim]
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import zipfile

import cv2
import numpy as np

W, H, FPS = 1920, 1080, 30
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
BG = (36, 31, 27)  # BGR dark


def sh(cmd):
    print("+", " ".join(cmd) if isinstance(cmd, list) else cmd)
    subprocess.run(cmd, check=True)


def stamp(vid, name):
    with open(os.path.join(vid, f"t_{name}")) as f:
        return float(f.read().strip())


def canvas():
    img = np.zeros((H, W, 3), np.uint8)
    img[:] = BG
    return img


def text(img, s, xy, scale=1.0, color=(240, 240, 240), thick=2):
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_DUPLEX, scale, color, thick, cv2.LINE_AA)


def fit(img, box_w, box_h):
    h, w = img.shape[:2]
    k = min(box_w / w, box_h / h)
    return cv2.resize(img, (int(w * k), int(h * k)), interpolation=cv2.INTER_AREA)


def paste(dst, src, x, y):
    h, w = src.shape[:2]
    dst[y:y + h, x:x + w] = src


def slide_title(path, title, subtitle, lines=()):
    img = canvas()
    text(img, title, (120, 420), 2.4, (255, 255, 255), 4)
    text(img, subtitle, (124, 500), 1.2, (120, 220, 255), 2)
    for i, line in enumerate(lines):
        text(img, line, (124, 600 + i * 50), 0.95, (200, 200, 200), 1)
    cv2.imwrite(path, img)


# ---------------------------------------------------------------- slides
def slide_plan(path, sim):
    import rasterio
    with rasterio.open(os.path.join(sim, "area.tif")) as src:
        rgb = src.read([1, 2, 3]).transpose(1, 2, 0)
        tr = src.transform
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    pts = []
    with open(os.path.join(sim, "plans", "block_1.waypoints")) as f:
        for ln in f.readlines()[2:]:
            t = ln.split()
            if t[3] != "16":  # skip non-waypoint items (e.g. DO_CHANGE_SPEED)
                continue
            col, row = ~tr * (float(t[9]), float(t[8]))
            pts.append((int(col), int(row)))
    for a, b in zip(pts, pts[1:]):
        cv2.arrowedLine(img, a, b, (0, 220, 255), 5, cv2.LINE_AA, tipLength=0.02)
    for i, p in enumerate(pts):
        cv2.circle(img, p, 11, (0, 0, 255), -1)
    cv2.circle(img, pts[0], 18, (0, 255, 0), 4)
    info = json.load(open(os.path.join(sim, "plans", "survey_info.json")))
    out = canvas()
    paste(out, fit(img, 1000, 1000), 60, 40)
    text(out, "1. Survey plan", (1120, 140), 1.8, (255, 255, 255), 3)
    rows = [f"Area: 140 x 140 m (GeoTIFF)",
            f"Altitude: {info['altitude_m']:.0f} m above take-off",
            f"Camera footprint: {info['footprint_m'][0]} x {info['footprint_m'][1]} m",
            f"Overlap: {info['side_overlap']:.0%} side / {info['front_overlap']:.0%} front",
            f"Line spacing: {info['line_spacing_m']} m",
            f"Photo every {info['trigger_distance_m']} m",
            f"Waypoints: {len(pts)} (2 per flight line)",
            f"UTM zone: EPSG:{info['utm_epsg']}"]
    for i, r in enumerate(rows):
        text(out, r, (1124, 240 + i * 62), 1.05, (215, 215, 215), 2)
    text(out, "green = first waypoint, arrows = flight path", (1124, 800), 0.8, (150, 200, 150), 1)
    cv2.imwrite(path, out)


def slide_photos(path, sim):
    files = sorted(glob.glob(os.path.join(sim, "photos", "*.jpg")))
    pick = [files[int(i)] for i in np.linspace(0, len(files) - 1, 12)] if files else []
    out = canvas()
    text(out, f"2. Geotagged photos: {len(files)} taken automatically every 7.85 m", (60, 80), 1.4, (255, 255, 255), 2)
    geo = {}
    gp = os.path.join(sim, "photos", "geo.txt")
    if os.path.exists(gp):
        for ln in open(gp).read().splitlines()[1:]:
            t = ln.split()
            geo[t[0]] = (float(t[2]), float(t[1]))
    tw, th, gap = 400, 300, 18
    x0 = (W - (4 * tw + 3 * gap)) // 2
    for k, fpath in enumerate(pick):
        im = cv2.resize(cv2.imread(fpath), (tw, th), interpolation=cv2.INTER_AREA)
        x, y = x0 + (k % 4) * (tw + gap), 120 + (k // 4) * (th + gap)
        paste(out, im, x, y)
        name = os.path.basename(fpath)
        if name in geo:
            lat, lon = geo[name]
            cv2.rectangle(out, (x, y + th - 30), (x + tw, y + th), (0, 0, 0), -1)
            text(out, f"{lat:.6f}, {lon:.6f}", (x + 8, y + th - 9), 0.6, (120, 255, 140), 1)
    cv2.imwrite(path, out)


def slide_ortho(path, sim):
    import rasterio
    with rasterio.open(os.path.join(sim, "results", "orthophoto.tif")) as src:
        k = max(src.width, src.height) / 2000
        data = src.read(out_shape=(src.count, int(src.height / k), int(src.width / k)))
    rgb = data[:3].transpose(1, 2, 0)
    alpha = data[3] if data.shape[0] > 3 else np.full(rgb.shape[:2], 255, np.uint8)
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    img[alpha == 0] = BG
    out = canvas()
    f = fit(img, 1180, 1000)
    paste(out, f, 60, (H - f.shape[0]) // 2)
    text(out, "4. Orthophoto", (1300, 140), 1.8, (255, 255, 255), 3)
    for i, r in enumerate(["Made by WebODM from the", "drone photos: one", "georeferenced, distortion-free",
                           "map image of the whole area."]):
        text(out, r, (1304, 240 + i * 55), 1.0, (215, 215, 215), 2)
    cv2.imwrite(path, out)


def slide_dsm(path, sim):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import rasterio
    with rasterio.open(os.path.join(sim, "results", "dsm.tif")) as src:
        k = max(src.width, src.height) / 1500
        dsm = src.read(1, out_shape=(int(src.height / k), int(src.width / k)), masked=True)
    ground = np.percentile(dsm.compressed(), 5)
    rel = dsm - ground
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100, facecolor="#1b1f24")
    ax = fig.add_axes([0.03, 0.05, 0.6, 0.9])
    im = ax.imshow(rel, cmap="turbo", vmin=0, vmax=max(5, float(np.percentile(rel.compressed(), 99.5))))
    ax.axis("off")
    cb = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02)
    cb.set_label("height above ground (m)", color="white", fontsize=16)
    cb.ax.tick_params(colors="white", labelsize=13)
    fig.text(0.68, 0.85, "5. Elevation model", color="white", fontsize=32, weight="bold")
    fig.text(0.68, 0.80, "(DSM, from the 3D point cloud)", color="#9ccfff", fontsize=18)
    for i, r in enumerate(["Every pixel has a height.", "Buildings and trees stand out",
                           f"(tallest: {float(rel.max()):.1f} m above ground)."]):
        fig.text(0.68, 0.70 - i * 0.06, r, color="#d7d7d7", fontsize=22)
    fig.savefig(path, facecolor=fig.get_facecolor())
    plt.close(fig)


# ---------------------------------------------------------------- video helpers
def still_clip(png, out, seconds):
    sh(["ffmpeg", "-y", "-loglevel", "error", "-loop", "1", "-i", png, "-t", str(seconds),
        "-vf", f"scale={W}:{H},fps={FPS},format=yuv420p,fade=t=in:st=0:d=0.5,fade=t=out:st={seconds - 0.5}:d=0.5",
        "-c:v", "libx264", "-preset", "medium", "-crf", "20", out])


def cut(raw, out, start, end, speed, label, box_w="iw"):
    dur = (end - start) / speed
    vf = (f"setpts=(PTS-STARTPTS)/{speed},fps={FPS},scale={W}:{H},format=yuv420p,"
          f"drawbox=x=0:y=0:w={box_w}:h=54:color=black@0.55:t=fill,"
          f"drawtext=fontfile={FONT}:text='{label}':x=20:y=13:fontsize=28:fontcolor=white")
    sh(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{start:.2f}", "-to", f"{end:.2f}", "-i", raw,
        "-vf", vf, "-an", "-c:v", "libx264", "-preset", "medium", "-crf", "21", out])
    return dur


def render_model(sim, tmp):
    out = os.path.join(tmp, "model_turntable.mp4")
    if os.path.exists(out) and os.path.getsize(out) > 100000:
        print("reusing", out)  # delete it to render again
        return out
    zpath = os.path.join(sim, "results", "textured_model.zip")
    mdir = os.path.join(tmp, "model")
    shutil.rmtree(mdir, ignore_errors=True)
    with zipfile.ZipFile(zpath) as z:
        z.extractall(mdir)
    objs = glob.glob(os.path.join(mdir, "**", "*geo.obj"), recursive=True) or \
        glob.glob(os.path.join(mdir, "**", "*.obj"), recursive=True)
    out = os.path.join(tmp, "model_turntable.mp4")
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "render_model.py")
    xvfb = None
    env = dict(os.environ)
    if not env.get("DISPLAY") or env["DISPLAY"] == ":99":  # Blender's Workbench needs an OpenGL screen
        xvfb = subprocess.Popen(["Xvfb", ":98", "-screen", "0", "1280x720x24", "-nolisten", "tcp"],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        env["DISPLAY"] = ":98"
        import time
        time.sleep(2)
    try:
        print("+ blender (turntable render)")
        subprocess.run(["blender", "-b", "--factory-startup", "-P", script, "--", objs[0], out, "12"],
                       check=True, env=env, stdout=subprocess.DEVNULL)
    finally:
        if xvfb:
            xvfb.terminate()
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--sim-dir", default=os.path.expanduser("~/survey_sim"))
    p.add_argument("--flight-seconds", type=float, default=150, help="length of the flight part")
    args = p.parse_args()
    sim = args.sim_dir
    vid = os.path.join(sim, "video")
    tmp = os.path.join(vid, "parts")
    os.makedirs(tmp, exist_ok=True)
    raw = os.path.join(vid, "demo_raw.mp4")
    t0 = stamp(vid, "start")
    rel = {n: stamp(vid, n) - t0 for n in ("plan_done", "sim_ready", "mission_done", "camera_done",
                                           "webodm_start", "webodm_done")}
    n_photos = len(glob.glob(os.path.join(sim, "photos", "*.jpg")))
    parts = []

    def add_still(name, seconds, maker, *a):
        png = os.path.join(tmp, f"{name}.png")
        maker(png, *a)
        mp4 = os.path.join(tmp, f"{name}.mp4")
        still_clip(png, mp4, seconds)
        parts.append(mp4)

    add_still("00_title", 6, slide_title, "Drone mapping pipeline", "Full simulation: plan -> fly -> photos -> WebODM 3D map",
              ["ArduPilot SITL + Gazebo Harmonic  |  ROS 2 Jazzy + MAVROS  |  WebODM",
               "Every step below is the project's own code running for real.",
               "Recorded on a virtual screen; long parts are sped up (shown top-left)."])
    add_still("01_plan", 9, slide_plan, sim)

    seg = os.path.join(tmp, "02_planner.mp4")
    cut(raw, seg, 0, rel["plan_done"], 1.0, "1. Planning - path_planner.py (real time)")
    parts.append(seg)

    flight_len = rel["camera_done"] - rel["sim_ready"] + 10
    speed = max(1.0, round(flight_len / args.flight_seconds))
    seg = os.path.join(tmp, "03_flight.mp4")
    cut(raw, seg, rel["sim_ready"] - 10, rel["camera_done"], speed,
        f"2. Autonomous survey flight  ({speed:.0f}x speed)", box_w=1280)
    parts.append(seg)

    add_still("04_photos", 7, slide_photos, sim)

    wstart, wend = rel["webodm_start"], rel["webodm_done"]
    seg = os.path.join(tmp, "05_webodm_a.mp4")
    first = min(25.0, wend - wstart)
    cut(raw, seg, wstart, wstart + first, 1.0, "3. WebODM processing - webodm_upload.py (real time)")
    parts.append(seg)
    if wend - wstart > first + 5:
        rest = wend + 6 - (wstart + first)
        sp = max(2.0, round(rest / 20))
        seg = os.path.join(tmp, "05_webodm_b.mp4")
        cut(raw, seg, wstart + first, wend + 6, sp, f"3. WebODM processing ({sp:.0f}x speed, {rest / 60:.0f} min)")
        parts.append(seg)

    add_still("06_ortho", 7, slide_ortho, sim)
    add_still("07_dsm", 7, slide_dsm, sim)

    try:
        tt = render_model(sim, tmp)
        seg = os.path.join(tmp, "08_model.mp4")
        sh(["ffmpeg", "-y", "-loglevel", "error", "-i", tt, "-vf",
            f"scale={W}:{H},fps={FPS},format=yuv420p,drawbox=x=0:y=0:w=iw:h=54:color=black@0.55:t=fill,"
            f"drawtext=fontfile={FONT}:text='6. Textured 3D model from WebODM':x=20:y=13:fontsize=28:fontcolor=white",
            "-c:v", "libx264", "-crf", "20", seg])
        parts.append(seg)
    except Exception as e:  # keep going without the 3D clip
        print("3D render failed:", e)

    flight_min = (rel["mission_done"] - rel["sim_ready"]) / 60
    proc_min = (wend - wstart) / 60
    add_still("09_end", 7, slide_title, "Done", "Plan -> fly -> photograph -> map, fully automatic",
              [f"{n_photos} geotagged photos, 1 survey block, 140 x 140 m at 40 m",
               f"Flight incl. take-off/landing: {flight_min:.0f} min (sim ran slower than real time)",
               f"WebODM processing: {proc_min:.0f} min  ->  orthophoto, DSM, textured 3D model"])

    listfile = os.path.join(tmp, "list.txt")
    with open(listfile, "w") as f:
        for p_ in parts:
            f.write(f"file '{p_}'\n")
    final = os.path.join(vid, "survey_demo.mp4")
    sh(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", listfile,
        "-c:v", "libx264", "-preset", "medium", "-crf", "21", "-pix_fmt", "yuv420p", "-movflags", "+faststart", final])
    print("Final video:", final)


if __name__ == "__main__":
    main()
