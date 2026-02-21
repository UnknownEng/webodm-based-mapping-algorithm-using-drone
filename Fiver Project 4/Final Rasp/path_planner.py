#!/usr/bin/env python3
import rasterio
from rasterio.warp import calculate_default_transform, reproject, Resampling
from pyproj import Transformer
from tkinter import Tk
from tkinter.filedialog import askopenfilename, askdirectory
import os
import numpy as np
import tempfile

# ------------------- CRS HELPERS ------------------- #
def latlon_to_utm_epsg(lat, lon):
    zone = int((lon + 180) / 6) + 1
    return 32600 + zone if lat >= 0 else 32700 + zone

# ------------------- REPROJECT TO UTM ------------------- #
def reproject_to_utm(src):
    lon, lat = src.bounds.left, src.bounds.bottom
    utm_epsg = latlon_to_utm_epsg(lat, lon)

    transform, width, height = calculate_default_transform(
        src.crs, f"EPSG:{utm_epsg}",
        src.width, src.height, *src.bounds
    )

    meta = src.meta.copy()
    meta.update({
        "crs": f"EPSG:{utm_epsg}",
        "transform": transform,
        "width": width,
        "height": height
    })

    return meta, utm_epsg

# ------------------- SURVEY GENERATION (METER-BASED) ------------------- #
def generate_survey_waypoints(
    tif_path,
    block_size_m=500,       # 500 m blocks
    block_overlap=0.5,
    line_spacing_m=24,
    inline_spacing_m=18,
    altitude=40
):
    with rasterio.open(tif_path) as src:
        utm_meta, utm_epsg = reproject_to_utm(src)
        pixel_size = abs(src.transform.a)

        if pixel_size > 10:
            print(f"⚠️ WARNING: GeoTIFF resolution is coarse ({pixel_size:.1f} m/pixel). Using meter-based spacing.")

        # Temporary UTM raster
        with tempfile.NamedTemporaryFile(suffix=".tif", delete=False) as tmp:
            utm_path = tmp.name

        with rasterio.open(utm_path, "w", **utm_meta) as dst:
            for i in range(1, src.count + 1):
                reproject(
                    source=rasterio.band(src, i),
                    destination=rasterio.band(dst, i),
                    src_transform=src.transform,
                    src_crs=src.crs,
                    dst_transform=utm_meta["transform"],
                    dst_crs=utm_meta["crs"],
                    resampling=Resampling.nearest
                )

    utm = rasterio.open(utm_path)
    transformer = Transformer.from_crs(utm.crs, "EPSG:4326", always_xy=True)
    minx, miny, maxx, maxy = utm.bounds
    utm.close()
    os.remove(utm_path)

    block_step = block_size_m * (1 - block_overlap)
    all_blocks = []

    xs_block = np.arange(minx, maxx, block_step)
    ys_block = np.arange(miny, maxy, block_step)

    for x0 in xs_block:
        for y0 in ys_block:
            x_points = np.arange(x0, min(x0 + block_size_m, maxx), inline_spacing_m)
            y_points = np.arange(y0, min(y0 + block_size_m, maxy), line_spacing_m)

            if len(x_points) < 2 or len(y_points) < 2:
                continue  # skip tiny blocks

            waypoints = []
            direction = 1
            for y in y_points:
                row_xs = x_points if direction == 1 else x_points[::-1]
                for x in row_xs:
                    lon, lat = transformer.transform(x, y)
                    waypoints.append((lat, lon, altitude))
                direction *= -1

            all_blocks.append(waypoints)

    return all_blocks

# ------------------- SAVE WAYPOINT FILES ------------------- #
def save_waypoints_files(all_blocks, output_folder):
    if not all_blocks:
        print("❌ No blocks generated. Check raster bounds or spacing.")
        return

    for idx, block_wps in enumerate(all_blocks, start=1):
        filename = os.path.join(output_folder, f"block_{idx}.waypoints")
        with open(filename, "w") as f:
            f.write("QGC WPL 110\n")
            for i, (lat, lon, alt) in enumerate(block_wps):
                current = 1 if i == 0 else 0
                f.write(
                    f"{i}\t{current}\t0\t16\t0\t0\t0\t0\t"
                    f"{lat}\t{lon}\t{alt}\n"
                )
        # Print summary for each block
        print(f"✅ Saved: {filename} | Waypoints: {len(block_wps)} | Block Area: {500*500} m²")

# ------------------- FILE PICKERS ------------------- #
def select_tif_file():
    Tk().withdraw()
    return askopenfilename(
        title="Select input GeoTIFF",
        filetypes=[("TIFF files", "*.tif *.tiff")]
    )

def select_output_folder():
    Tk().withdraw()
    return askdirectory(title="Select output folder")

# ------------------- MAIN ------------------- #
if __name__ == "__main__":
    print("📂 Select input GeoTIFF")
    tif_file = select_tif_file()
    if not tif_file:
        exit()

    print("📂 Select output folder")
    out_folder = select_output_folder()
    if not out_folder:
        exit()

    print(f"📌 Generating waypoints from {tif_file} to {out_folder} ...")

    blocks = generate_survey_waypoints(
        tif_file,
        block_size_m=500,
        block_overlap=0.5,
        line_spacing_m=24,
        inline_spacing_m=18,
        altitude=40
    )

    save_waypoints_files(blocks, out_folder)

    print(f"\n✅ Done! Generated {len(blocks)} blocks of waypoints.")
