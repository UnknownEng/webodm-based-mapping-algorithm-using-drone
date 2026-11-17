#!/usr/bin/env python3
"""
Generate the Gazebo (Harmonic) survey world used to test the full pipeline:

  models/survey_town/   300 x 300 m textured ground + textured buildings + trees
  models/iris_survey/   ArduPilot Iris with a downward camera (same FOV as the
                        Pi HQ camera IMX477 + 6 mm lens), 1280x960, topic /survey_camera/image
  worlds/survey.sdf     world (ArduPilot 1 ms physics, sensors, sun)
  area.tif              GeoTIFF of the area to survey (input for path_planner.py)

Everything is procedurally generated with a fixed seed, so it is reproducible.
Needs: numpy, opencv-python, rasterio.

    python gen_world.py [--out DIR] [--home-lat -35.363262 --home-lon 149.165237]
"""
import argparse
import math
import os

import cv2
import numpy as np

GROUND_M = 300.0       # ground plane size
GROUND_PX = 8192       # texture resolution (3.7 cm/px, close to the camera GSD at 40 m)
AREA_M = 140.0         # survey area (square, centred on the take-off point)
CAM_HFOV = 2 * math.atan(6.287 / 2 / 6.0)  # IMX477 width / 6 mm lens = 0.966 rad


# ======================= TEXTURES ======================= #
def noise(rng, size, octaves=6, base=4):
    """Multi-octave value noise in [0, 1]."""
    out = np.zeros((size, size), np.float32)
    amp, total = 1.0, 0.0
    for o in range(octaves):
        n = base * (2 ** o)
        small = rng.random((n, n)).astype(np.float32)
        out += cv2.resize(small, (size, size), interpolation=cv2.INTER_CUBIC) * amp
        total += amp
        amp *= 0.55
    out /= total
    return np.clip((out - out.min()) / (np.ptp(out) + 1e-6), 0, 1)


def m2px(v):
    return int(round(v / GROUND_M * GROUND_PX))


def world_to_px(x, y):
    """World metres (x east-ish, y north-ish, origin centre) -> texture pixel (col, row)."""
    return (int(round((x / GROUND_M + 0.5) * GROUND_PX)), int(round((0.5 - y / GROUND_M) * GROUND_PX)))


def make_ground(rng, buildings):
    s = GROUND_PX
    n = noise(rng, s, octaves=8, base=3)
    img = np.empty((s, s, 3), np.uint8)  # BGR, built channel by channel to save memory
    for c, (a, b, f) in enumerate([(40, 50, 20), (90, 70, 25), (35, 40, 15)]):
        fine = rng.random((s, s), dtype=np.float32)
        img[..., c] = np.clip(a + b * n + f * fine, 0, 255).astype(np.uint8)
    del n, fine

    # crop fields with row patterns (worked on the field's bounding box only)
    for _ in range(14):
        cx, cy = rng.uniform(-140, 140, 2)
        if abs(cx) < 70 and abs(cy) < 70:
            continue  # keep the town centre for buildings
        w, h, ang = rng.uniform(25, 60), rng.uniform(25, 60), rng.uniform(0, 180)
        box = cv2.boxPoints((world_to_px(cx, cy), (m2px(w), m2px(h)), ang)).astype(np.int32)
        x0, y0 = np.clip(box.min(0), 0, s - 1)
        x1, y1 = np.clip(box.max(0) + 1, 1, s)
        if x1 <= x0 or y1 <= y0:
            continue
        mask = np.zeros((y1 - y0, x1 - x0), np.uint8)
        cv2.fillPoly(mask, [box - [x0, y0]], 255)
        base = np.array([rng.integers(20, 90), rng.integers(90, 200), rng.integers(100, 220)], np.int16)
        period = int(rng.integers(6, 18))
        rows = np.arange(y0, y1)[:, None]
        cols = np.arange(x0, x1)[None, :]
        stripes = ((rows * math.cos(math.radians(ang)) + cols * math.sin(math.radians(ang))) // period % 2)
        layer = (base[None, None, :] - 30 * stripes[..., None]).clip(0, 255).astype(np.uint8)
        sub = img[y0:y1, x0:x1]
        sub[mask > 0] = layer[mask > 0]

    # roads (grid through the town) with lane markings
    for axis, pos in [("x", -45.0), ("x", 25.0), ("y", -20.0), ("y", 50.0), ("x", 95.0)]:
        if axis == "x":
            p1, p2 = world_to_px(pos, -150), world_to_px(pos, 150)
        else:
            p1, p2 = world_to_px(-150, pos), world_to_px(150, pos)
        cv2.line(img, p1, p2, (80, 82, 85), m2px(7))
        cv2.line(img, p1, p2, (60, 62, 64), m2px(0.3))
        # dashed centre line
        length = 300
        for k in range(0, length, 6):
            t1, t2 = k / length, (k + 3) / length
            a = (int(p1[0] + (p2[0] - p1[0]) * t1), int(p1[1] + (p2[1] - p1[1]) * t1))
            b = (int(p1[0] + (p2[0] - p1[0]) * t2), int(p1[1] + (p2[1] - p1[1]) * t2))
            cv2.line(img, a, b, (235, 235, 235), m2px(0.25))
    # road noise (asphalt texture)
    gray = (img[..., 0] == 80) & (img[..., 1] == 82)
    img[gray] = np.clip(img[gray].astype(np.int16) + rng.integers(-25, 25, (gray.sum(), 1)), 0, 255).astype(np.uint8)

    # parking lots with cars
    for (px, py) in [(-10, 10), (40, -40), (-70, 30)]:
        tl, br = world_to_px(px - 12, py + 8), world_to_px(px + 12, py - 8)
        cv2.rectangle(img, tl, br, (105, 105, 108), -1)
        for i in range(8):
            for j in range(2):
                cx, cy = px - 10.5 + i * 3, py + 4 - j * 8
                cv2.line(img, world_to_px(cx - 1.5, cy + 2.5), world_to_px(cx - 1.5, cy - 2.5), (230, 230, 230), m2px(0.12))
                if rng.random() < 0.7:
                    col = tuple(int(c) for c in rng.integers(20, 240, 3))
                    cv2.rectangle(img, world_to_px(cx - 0.9, cy + 2.2), world_to_px(cx + 0.9, cy - 2.2), col, -1)
                    cv2.rectangle(img, world_to_px(cx - 0.7, cy + 1.0), world_to_px(cx + 0.7, cy - 0.6), (40, 40, 40), -1)

    # concrete pads under buildings
    for b in buildings:
        x, y, w, d = b["x"], b["y"], b["w"] + 3, b["d"] + 3
        box = cv2.boxPoints((world_to_px(x, y), (m2px(w), m2px(d)), -b["yaw"])).astype(np.int32)
        cv2.fillPoly(img, [box], (150, 155, 160))

    # bushes, rocks, small details = lots of features for photogrammetry
    for _ in range(9000):
        x, y = rng.uniform(-150, 150, 2)
        r = m2px(rng.uniform(0.15, 0.9))
        col = (int(rng.integers(20, 90)), int(rng.integers(60, 160)), int(rng.integers(20, 110)))
        if rng.random() < 0.25:
            col = tuple(int(c) for c in rng.integers(90, 200, 3))
        cv2.circle(img, world_to_px(x, y), max(r, 1), col, -1)

    # take-off pad "H" at the origin
    cv2.circle(img, world_to_px(0, 0), m2px(3), (200, 200, 200), -1)
    cv2.circle(img, world_to_px(0, 0), m2px(3), (30, 30, 200), m2px(0.3))
    cv2.putText(img, "H", (world_to_px(-1.3, -1.6)), cv2.FONT_HERSHEY_SIMPLEX, m2px(3.2) / 30, (30, 30, 200), m2px(0.6))

    # slight blur so the ground is not aliased
    return cv2.GaussianBlur(img, (3, 3), 0)


def make_facade(rng, size=1024):
    base = np.array(rng.integers(90, 230, 3), np.int16)
    img = np.empty((size, size, 3), np.int16)
    img[:] = base
    img += (noise(rng, size, 5, 8)[..., None] * 40 - 20).astype(np.int16)
    rows, cols = rng.integers(4, 9), rng.integers(4, 10)
    win = np.array(rng.integers(30, 120, 3), np.int16)
    for r in range(rows):
        for c in range(cols):
            x0 = int((c + 0.2) * size / cols); x1 = int((c + 0.8) * size / cols)
            y0 = int((r + 0.2) * size / rows); y1 = int((r + 0.7) * size / rows)
            shade = win + rng.integers(-20, 40)
            img[y0:y1, x0:x1] = shade
            img[y0:y1, (x0 + x1) // 2 - 3:(x0 + x1) // 2 + 3] = base - 40
    return np.clip(img, 0, 255).astype(np.uint8)


def make_roof(rng, size=1024):
    g = rng.integers(70, 170)
    img = np.empty((size, size, 3), np.int16)
    img[:] = (g, g + rng.integers(-15, 15), g + rng.integers(-15, 15))
    img += (rng.random((size, size, 1)) * 60 - 30).astype(np.int16)
    img += (noise(rng, size, 4, 4)[..., None] * 50 - 25).astype(np.int16)
    img = np.clip(img, 0, 255).astype(np.uint8)
    for _ in range(rng.integers(3, 9)):  # AC units / skylights
        x, y = rng.integers(80, size - 200, 2)
        w, h = rng.integers(60, 180, 2)
        col = tuple(int(c) for c in rng.integers(150, 250, 3))
        cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), col, -1)
        cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), (40, 40, 40), 4)
    cv2.rectangle(img, (0, 0), (size - 1, size - 1), (200, 200, 200), 24)  # parapet
    return img


def make_leaves(rng, size=512):
    n = noise(rng, size, 6, 8)
    img = np.stack([20 + 40 * n, 70 + 110 * n, 20 + 50 * n], -1)
    img += rng.random((size, size, 1)) * 40
    return np.clip(img, 0, 255).astype(np.uint8)


# ======================= SDF ======================= #
def visual_box(name, pose, size, tex):
    return f"""
      <visual name="{name}">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse><specular>0.1 0.1 0.1 1</specular>
          <pbr><metal><albedo_map>{tex}</albedo_map><roughness>0.9</roughness><metalness>0</metalness></metal></pbr>
        </material>
      </visual>"""


def town_sdf(buildings, trees):
    parts = []
    parts.append(f"""
    <link name="ground">
      <collision name="ground_collision">
        <geometry><plane><normal>0 0 1</normal><size>{GROUND_M} {GROUND_M}</size></plane></geometry>
      </collision>
      <visual name="ground_visual">
        <geometry><plane><normal>0 0 1</normal><size>{GROUND_M} {GROUND_M}</size></plane></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>
          <pbr><metal><albedo_map>model://survey_town/textures/ground.jpg</albedo_map>
            <roughness>1</roughness><metalness>0</metalness></metal></pbr>
        </material>
      </visual>
    </link>""")
    for i, b in enumerate(buildings):
        roof_t = 0.3
        parts.append(f"""
    <link name="building_{i}">
      <pose>{b['x']:.2f} {b['y']:.2f} 0 0 0 {math.radians(b['yaw']):.4f}</pose>
      <collision name="c"><pose>0 0 {b['h'] / 2:.2f} 0 0 0</pose>
        <geometry><box><size>{b['w']:.2f} {b['d']:.2f} {b['h']:.2f}</size></box></geometry></collision>
      {visual_box('walls', f"0 0 {b['h'] / 2:.2f} 0 0 0", f"{b['w']:.2f} {b['d']:.2f} {b['h']:.2f}",
                  f"model://survey_town/textures/facade_{i}.jpg")}
      {visual_box('roof', f"0 0 {b['h'] + roof_t / 2:.2f} 0 0 0", f"{b['w'] + 0.4:.2f} {b['d'] + 0.4:.2f} {roof_t}",
                  f"model://survey_town/textures/roof_{i}.jpg")}
    </link>""")
    for i, t in enumerate(trees):
        parts.append(f"""
    <link name="tree_{i}">
      <pose>{t['x']:.2f} {t['y']:.2f} 0 0 0 0</pose>
      <visual name="trunk"><pose>0 0 {t['h'] / 2:.2f} 0 0 0</pose>
        <geometry><cylinder><radius>0.25</radius><length>{t['h']:.2f}</length></cylinder></geometry>
        <material><diffuse>0.35 0.25 0.15 1</diffuse></material></visual>
      <visual name="crown"><pose>0 0 {t['h']:.2f} 0 0 0</pose>
        <geometry><sphere><radius>{t['r']:.2f}</radius></sphere></geometry>
        <material><diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://survey_town/textures/leaves.jpg</albedo_map><roughness>1</roughness></metal></pbr>
        </material></visual>
    </link>""")
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="survey_town">
    <static>true</static>{''.join(parts)}
  </model>
</sdf>
"""


IRIS_SDF = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="iris_survey">
    <include merge="true">
      <uri>model://iris_with_ardupilot</uri>
    </include>

    <!-- Downward survey camera: same horizontal FOV as the Pi HQ camera (IMX477) + 6 mm lens.
         Long image side across the flight direction (as assumed by path_planner.py). -->
    <link name="survey_camera_link">
      <pose>0 0 -0.08 0 1.5708 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <sensor name="survey_camera" type="camera">
        <camera>
          <horizontal_fov>{CAM_HFOV:.4f}</horizontal_fov>
          <image><width>1280</width><height>960</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>600</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <visualize>false</visualize>
        <topic>/survey_camera/image</topic>
      </sensor>
    </link>
    <joint name="survey_camera_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>survey_camera_link</child>
    </joint>
  </model>
</sdf>
"""

WORLD_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="survey">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

    <scene>
      <ambient>0.7 0.7 0.7</ambient>
      <background>0.6 0.75 0.9</background>
      <sky></sky>
      <shadows>true</shadows>
    </scene>

    <spherical_coordinates>
      <latitude_deg>{lat}</latitude_deg>
      <longitude_deg>{lon}</longitude_deg>
      <elevation>{elev}</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.3 0.2 -0.93</direction>
    </light>

    <include>
      <uri>model://survey_town</uri>
    </include>

    <include>
      <uri>model://iris_survey</uri>
      <pose degrees="true">0 0 0.195 0 0 90</pose>
    </include>
  </world>
</sdf>
"""


def write_area_tif(path, img, lat, lon):
    """GeoTIFF of the survey area (square AREA_M, centred on the take-off point)."""
    import rasterio
    from rasterio.transform import from_bounds
    half = AREA_M / 2
    dlat = half / 111320.0
    dlon = half / (111320.0 * math.cos(math.radians(lat)))
    c0, r0 = world_to_px(-half, half)
    c1, r1 = world_to_px(half, -half)
    crop = cv2.resize(img[r0:r1, c0:c1], (1400, 1400), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)
    transform = from_bounds(lon - dlon, lat - dlat, lon + dlon, lat + dlat, 1400, 1400)
    with rasterio.open(path, "w", driver="GTiff", width=1400, height=1400, count=3, dtype="uint8",
                       crs="EPSG:4326", transform=transform, compress="deflate") as dst:
        dst.write(rgb)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
    p.add_argument("--home-lat", type=float, default=-35.363262)
    p.add_argument("--home-lon", type=float, default=149.165237)
    p.add_argument("--home-alt", type=float, default=584.0)
    p.add_argument("--seed", type=int, default=7)
    args = p.parse_args()
    rng = np.random.default_rng(args.seed)

    # Buildings in the town centre (keep the take-off pad clear)
    buildings = []
    while len(buildings) < 14:
        x, y = rng.uniform(-60, 60, 2)
        w, d = rng.uniform(8, 20, 2)
        if math.hypot(x, y) < 18 or any(math.hypot(x - b["x"], y - b["y"]) < (max(w, d) + max(b["w"], b["d"])) / 2 + 4
                                        for b in buildings):
            continue
        # keep off the roads
        if min(abs(x + 45), abs(x - 25)) < w / 2 + 5 or min(abs(y + 20), abs(y - 50)) < d / 2 + 5:
            continue
        buildings.append(dict(x=x, y=y, w=w, d=d, h=float(rng.uniform(5, 22)), yaw=float(rng.choice([0, 0, 15, -20, 30]))))
    trees = []
    while len(trees) < 30:
        x, y = rng.uniform(-75, 75, 2)
        if math.hypot(x, y) < 10 or any(abs(x - b["x"]) < b["w"] / 2 + 4 and abs(y - b["y"]) < b["d"] / 2 + 4 for b in buildings):
            continue
        trees.append(dict(x=x, y=y, h=float(rng.uniform(4, 9)), r=float(rng.uniform(1.8, 3.5))))

    town = os.path.join(args.out, "models", "survey_town")
    tex = os.path.join(town, "textures")
    iris = os.path.join(args.out, "models", "iris_survey")
    worlds = os.path.join(args.out, "worlds")
    for d in (tex, iris, worlds):
        os.makedirs(d, exist_ok=True)

    print("Generating ground texture (8192x8192)...")
    ground = make_ground(rng, buildings)
    cv2.imwrite(os.path.join(tex, "ground.jpg"), ground, [cv2.IMWRITE_JPEG_QUALITY, 92])
    for i in range(len(buildings)):
        cv2.imwrite(os.path.join(tex, f"facade_{i}.jpg"), make_facade(rng), [cv2.IMWRITE_JPEG_QUALITY, 90])
        cv2.imwrite(os.path.join(tex, f"roof_{i}.jpg"), make_roof(rng), [cv2.IMWRITE_JPEG_QUALITY, 90])
    cv2.imwrite(os.path.join(tex, "leaves.jpg"), make_leaves(rng), [cv2.IMWRITE_JPEG_QUALITY, 90])

    for folder, name, sdf in ((town, "survey_town", town_sdf(buildings, trees)), (iris, "iris_survey", IRIS_SDF)):
        with open(os.path.join(folder, "model.sdf"), "w", newline="\n") as f:
            f.write(sdf)
        with open(os.path.join(folder, "model.config"), "w", newline="\n") as f:
            f.write(f'<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version>'
                    f'<sdf version="1.9">model.sdf</sdf><description>Generated by gen_world.py</description></model>\n')
    with open(os.path.join(worlds, "survey.sdf"), "w", newline="\n") as f:
        f.write(WORLD_SDF.format(lat=args.home_lat, lon=args.home_lon, elev=args.home_alt))

    write_area_tif(os.path.join(args.out, "area.tif"), ground, args.home_lat, args.home_lon)
    cv2.imwrite(os.path.join(args.out, "ground_preview.jpg"), cv2.resize(ground, (1024, 1024), interpolation=cv2.INTER_AREA))
    print(f"World written to {args.out}: {len(buildings)} buildings, {len(trees)} trees, "
          f"camera HFOV {math.degrees(CAM_HFOV):.1f} deg, area.tif = {AREA_M:.0f} x {AREA_M:.0f} m")


if __name__ == "__main__":
    main()
