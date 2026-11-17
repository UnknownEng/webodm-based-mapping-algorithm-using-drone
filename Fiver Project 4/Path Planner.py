#!/usr/bin/env python3
"""
Survey path planner.

Reads a GeoTIFF of the area to map (any CRS - e.g. a WebODM orthophoto in UTM,
or a lat/lon GeoTIFF), and writes one ArduPilot/QGC ".waypoints" file per block.

* Line spacing and photo trigger distance are computed from the camera
  (sensor size + focal length), the flight altitude and the requested overlap.
* Only areas that contain image data are flown (nodata / transparent areas are
  skipped), not the whole bounding box.
* Each flight line is just 2 waypoints (start + end). Photos are taken by the
  camera script every `trigger_distance_m` metres along the line.
* Output files follow the standard QGC WPL 110 format (12 columns). Row 0 is a
  home placeholder (Mission Planner / QGC convention); survey rows use
  frame 3 = altitude relative to home.

Usage:
    python path_planner.py area.tif out_folder [--altitude 40] [--focal-mm 6] ...
    python path_planner.py            (opens file pickers)
"""
import argparse
import json
import math
import os

import numpy as np
import rasterio
from pyproj import Transformer
from rasterio.warp import transform_bounds
from rasterio.transform import Affine, rowcol

# Raspberry Pi HQ camera (Sony IMX477): 4056 x 3040 px, 1.55 um pixels
IMX477_SENSOR_W_MM = 6.287
IMX477_SENSOR_H_MM = 4.712
IMX477_PIXEL_UM = 1.55

MAX_MASK_PIXELS = 2000  # data mask is read downsampled to at most this many px per side


# ------------------- CRS HELPERS ------------------- #
def utm_epsg_for(lon, lat):
    """EPSG code of the WGS84 UTM zone containing (lon, lat)."""
    zone = int((lon + 180.0) // 6.0) + 1
    zone = min(max(zone, 1), 60)
    return (32600 if lat >= 0 else 32700) + zone


# ------------------- CAMERA MATHS ------------------- #
def camera_layout(altitude, focal_mm, sensor_w_mm, sensor_h_mm, side_overlap, front_overlap):
    """Return (line_spacing_m, trigger_distance_m, footprint_w_m, footprint_h_m).

    Assumes the camera's long side is mounted across the flight direction.
    """
    if not (0 <= side_overlap < 1 and 0 <= front_overlap < 1):
        raise ValueError("Overlaps must be between 0 and 1 (e.g. 0.7 for 70%)")
    footprint_w = altitude * sensor_w_mm / focal_mm   # across track
    footprint_h = altitude * sensor_h_mm / focal_mm   # along track
    line_spacing = footprint_w * (1 - side_overlap)
    trigger_distance = footprint_h * (1 - front_overlap)
    return line_spacing, trigger_distance, footprint_w, footprint_h


def block_starts(lo, hi, size, step):
    """Start coordinates of blocks covering [lo, hi], evenly spaced so that
    neighbours overlap by at least (size - step) and the last block ends exactly
    at hi (no tiny leftover strips, no near-duplicate blocks)."""
    if hi - lo <= size:
        return [lo]
    n = int(math.ceil((hi - lo - size) / step)) + 1
    return list(np.linspace(lo, hi - size, n))


def inclusive_range(lo, hi, step):
    """Values from lo to hi (inclusive) spaced at most `step` apart."""
    if hi <= lo:
        return np.array([lo])
    n = int(math.ceil((hi - lo) / step))
    return np.linspace(lo, hi, n + 1)


# ------------------- DATA MASK ------------------- #
class DataMask:
    """Tells whether a UTM point lies on valid (non-nodata) raster pixels."""

    def __init__(self, src, utm_crs):
        scale = max(src.width, src.height) / MAX_MASK_PIXELS
        scale = max(scale, 1.0)
        out_h = max(1, int(round(src.height / scale)))
        out_w = max(1, int(round(src.width / scale)))
        # dataset_mask uses nodata values / alpha band / internal mask (255 = valid)
        self.mask = src.dataset_mask(out_shape=(out_h, out_w)) > 0
        self.transform = src.transform * Affine.scale(src.width / out_w, src.height / out_h)
        self.to_src = Transformer.from_crs(utm_crs, src.crs, always_xy=True)
        self.to_utm = Transformer.from_crs(src.crs, utm_crs, always_xy=True)
        self.h, self.w = self.mask.shape

    def valid_bounds(self):
        """UTM bounding box of the pixels that contain data (ignores empty margins)."""
        rows, cols = np.nonzero(self.mask)
        if len(rows) == 0:
            raise ValueError("GeoTIFF contains no valid (non-nodata) pixels")
        r0, r1, c0, c1 = rows.min(), rows.max() + 1, cols.min(), cols.max() + 1
        corners = [self.transform * (c, r) for c, r in ((c0, r0), (c1, r0), (c0, r1), (c1, r1))]
        xs, ys = self.to_utm.transform([p[0] for p in corners], [p[1] for p in corners])
        return min(xs), min(ys), max(xs), max(ys)

    def valid(self, xs, ys):
        sx, sy = self.to_src.transform(np.asarray(xs, dtype=float), np.asarray(ys, dtype=float))
        rows, cols = rowcol(self.transform, sx, sy)
        rows = np.asarray(rows)
        cols = np.asarray(cols)
        inside = (rows >= 0) & (rows < self.h) & (cols >= 0) & (cols < self.w)
        out = np.zeros(len(rows), dtype=bool)
        out[inside] = self.mask[rows[inside], cols[inside]]
        return out


# ------------------- SURVEY GENERATION ------------------- #
def generate_survey(
    tif_path,
    altitude=40.0,
    focal_mm=6.0,
    sensor_w_mm=IMX477_SENSOR_W_MM,
    sensor_h_mm=IMX477_SENSOR_H_MM,
    side_overlap=0.7,
    front_overlap=0.75,
    block_size_m=200.0,
    speed_ms=5.0,
):
    """Return (blocks, info). Each block is a dict with 'waypoints' [(lat, lon, alt), ...]
    and stats; info holds the survey parameters."""
    if altitude <= 0 or focal_mm <= 0 or block_size_m <= 0:
        raise ValueError("altitude, focal length and block size must be positive")

    line_spacing, trigger_dist, fp_w, fp_h = camera_layout(
        altitude, focal_mm, sensor_w_mm, sensor_h_mm, side_overlap, front_overlap)
    if block_size_m < 2 * line_spacing:
        raise ValueError(f"Block size ({block_size_m} m) must be at least 2x the line spacing ({line_spacing:.1f} m)")

    with rasterio.open(tif_path) as src:
        if src.crs is None:
            raise ValueError("GeoTIFF has no coordinate system (CRS) - it must be georeferenced")

        # Pick the UTM zone from the centre of the raster in lat/lon (works for any source CRS)
        lon_min, lat_min, lon_max, lat_max = transform_bounds(src.crs, "EPSG:4326", *src.bounds)
        utm_epsg = utm_epsg_for((lon_min + lon_max) / 2, (lat_min + lat_max) / 2)
        utm_crs = f"EPSG:{utm_epsg}"
        mask = DataMask(src, utm_crs)
        minx, miny, maxx, maxy = mask.valid_bounds()  # tile only where there is image data

    to_wgs = Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True)
    sample_step = max(1.0, min(trigger_dist, 5.0))
    block_step = block_size_m - line_spacing  # neighbouring blocks share one flight line

    blocks = []
    for x0 in block_starts(minx, maxx, block_size_m, block_step):
        x1 = min(x0 + block_size_m, maxx)
        for y0 in block_starts(miny, maxy, block_size_m, block_step):
            y1 = min(y0 + block_size_m, maxy)

            xs = inclusive_range(x0, x1, sample_step)
            lines = []
            for y in inclusive_range(y0, y1, line_spacing):
                ok = mask.valid(xs, np.full(len(xs), y))
                if ok.any():
                    valid_x = xs[ok]
                    lines.append((float(valid_x.min()), float(valid_x.max()), float(y)))
            if not lines:
                continue  # no image data in this block

            waypoints_utm = []
            for i, (xa, xb, y) in enumerate(lines):
                if i % 2 == 1:  # serpentine
                    xa, xb = xb, xa
                waypoints_utm.append((xa, y))
                waypoints_utm.append((xb, y))

            length = sum(math.dist(waypoints_utm[i], waypoints_utm[i + 1])
                         for i in range(len(waypoints_utm) - 1))
            waypoints = []
            for x, y in waypoints_utm:
                lon, lat = to_wgs.transform(x, y)
                waypoints.append((lat, lon, altitude))

            blocks.append({
                "waypoints": waypoints,
                "lines": len(lines),
                "length_m": length,
                "area_m2": (x1 - x0) * (y1 - y0),
                "est_minutes": length / speed_ms / 60.0,
                "est_photos": int(length / trigger_dist) + len(lines),
            })

    info = {
        "altitude_m": altitude,
        "focal_mm": focal_mm,
        "sensor_w_mm": sensor_w_mm,
        "sensor_h_mm": sensor_h_mm,
        "side_overlap": side_overlap,
        "front_overlap": front_overlap,
        "line_spacing_m": round(line_spacing, 2),
        "trigger_distance_m": round(trigger_dist, 2),
        "footprint_m": [round(fp_w, 1), round(fp_h, 1)],
        "gsd_cm_per_px": round(altitude * IMX477_PIXEL_UM / 1000.0 / focal_mm * 100, 2),
        "utm_epsg": utm_epsg,
        "block_size_m": block_size_m,
        "speed_ms": speed_ms,
    }
    return blocks, info


# ------------------- SAVE WAYPOINT FILES ------------------- #
def write_waypoints_file(filename, waypoints, speed_ms=None):
    """Write a QGC WPL 110 file: row 0 = home placeholder, [row 1 = survey speed],
    then the survey waypoints."""
    with open(filename, "w", newline="\n") as f:
        f.write("QGC WPL 110\n")
        lat0, lon0, _ = waypoints[0]
        # INDEX CURRENT FRAME COMMAND P1 P2 P3 P4 LAT LON ALT AUTOCONTINUE
        # Row 0: home placeholder (frame 0). The mission script replaces it with
        # the real take-off position.
        f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{lat0:.8f}\t{lon0:.8f}\t0\t1\n")
        i = 1
        if speed_ms:
            # MAV_CMD_DO_CHANGE_SPEED (178): p1=1 ground speed, p2=m/s, p3=-1 keep throttle.
            # Without it the drone flies at its WPNAV_SPEED (often 10 m/s) -> blurry photos.
            f.write(f"{i}\t0\t3\t178\t1\t{speed_ms:.1f}\t-1\t0\t0\t0\t0\t1\n")
            i += 1
        for lat, lon, alt in waypoints:
            # frame 3 = MAV_FRAME_GLOBAL_RELATIVE_ALT (altitude above home)
            f.write(f"{i}\t0\t3\t16\t0\t0\t0\t0\t{lat:.8f}\t{lon:.8f}\t{alt:.2f}\t1\n")
            i += 1


def save_blocks(blocks, info, output_folder, max_minutes=15.0):
    if not blocks:
        print("No blocks generated - the GeoTIFF seems to contain no valid data.")
        return []

    os.makedirs(output_folder, exist_ok=True)
    files = []
    for idx, block in enumerate(blocks, start=1):
        filename = os.path.join(output_folder, f"block_{idx}.waypoints")
        write_waypoints_file(filename, block["waypoints"], info.get("speed_ms"))
        files.append(filename)
        warn = "  <-- longer than battery limit, use a smaller --block-size" if block["est_minutes"] > max_minutes else ""
        print(f"Saved {filename} | lines: {block['lines']} | waypoints: {len(block['waypoints'])} | "
              f"block: {block['area_m2'] / 10000:.2f} ha | path: {block['length_m'] / 1000:.2f} km | "
              f"~{block['est_minutes']:.1f} min | ~{block['est_photos']} photos{warn}")

    info = dict(info, blocks=len(blocks))
    with open(os.path.join(output_folder, "survey_info.json"), "w") as f:
        json.dump(info, f, indent=2)
    return files


# ------------------- FILE PICKERS (fallback only) ------------------- #
def pick_tif_file():
    from tkinter import Tk
    from tkinter.filedialog import askopenfilename
    Tk().withdraw()
    return askopenfilename(title="Select input GeoTIFF", filetypes=[("TIFF files", "*.tif *.tiff")])


def pick_output_folder():
    from tkinter import Tk
    from tkinter.filedialog import askdirectory
    Tk().withdraw()
    return askdirectory(title="Select output folder")


# ------------------- MAIN ------------------- #
def main():
    p = argparse.ArgumentParser(description="Generate survey .waypoints files from a GeoTIFF")
    p.add_argument("tif", nargs="?", help="input GeoTIFF (file picker if omitted)")
    p.add_argument("output", nargs="?", help="output folder (folder picker if omitted)")
    p.add_argument("--altitude", type=float, default=40.0, help="flight altitude above home, m (default 40)")
    p.add_argument("--focal-mm", type=float, default=6.0, help="lens focal length, mm (default 6 = Pi 6mm CS lens)")
    p.add_argument("--sensor-w-mm", type=float, default=IMX477_SENSOR_W_MM)
    p.add_argument("--sensor-h-mm", type=float, default=IMX477_SENSOR_H_MM)
    p.add_argument("--side-overlap", type=float, default=0.70, help="side overlap 0-1 (default 0.70)")
    p.add_argument("--front-overlap", type=float, default=0.75, help="front overlap 0-1 (default 0.75)")
    p.add_argument("--block-size", type=float, default=200.0, help="block edge length, m (default 200)")
    p.add_argument("--speed", type=float, default=5.0,
                   help="survey ground speed, m/s - written into the mission (default 5; 3-5 m/s keeps photos sharp)")
    p.add_argument("--max-minutes", type=float, default=15.0, help="warn when a block takes longer than this")
    args = p.parse_args()

    tif = args.tif or pick_tif_file()
    if not tif:
        raise SystemExit("No GeoTIFF selected")
    out = args.output or pick_output_folder()
    if not out:
        raise SystemExit("No output folder selected")

    print(f"Generating waypoints from {tif} ...")
    blocks, info = generate_survey(
        tif,
        altitude=args.altitude,
        focal_mm=args.focal_mm,
        sensor_w_mm=args.sensor_w_mm,
        sensor_h_mm=args.sensor_h_mm,
        side_overlap=args.side_overlap,
        front_overlap=args.front_overlap,
        block_size_m=args.block_size,
        speed_ms=args.speed,
    )
    print(f"Camera footprint {info['footprint_m'][0]} x {info['footprint_m'][1]} m | "
          f"GSD {info['gsd_cm_per_px']} cm/px | line spacing {info['line_spacing_m']} m | "
          f"photo every {info['trigger_distance_m']} m (UTM EPSG:{info['utm_epsg']})")
    files = save_blocks(blocks, info, out, args.max_minutes)
    if files:
        print(f"\nDone: {len(files)} block(s). Start the camera with "
              f"--trigger-distance {info['trigger_distance_m']}")


if __name__ == "__main__":
    main()
