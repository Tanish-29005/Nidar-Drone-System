#!/usr/bin/env python3
"""
Standalone LawnMower Path Generator (FIXED)
Author: NIDAR / FSCOUT
"""

import math
import numpy as np
import xml.etree.ElementTree as ET
import simplekml
import matplotlib.pyplot as plt

from shapely.geometry import Polygon, LineString
from pyproj import Transformer
import sys
import os

# =====================================================
# CAMERA + MISSION PARAMETERS
# =====================================================
MISSION_ALT = 55.0  # meters AGL
IMAGE_W = 1920
IMAGE_H = 1440

SENSOR_WIDTH_MM = 6.287
SENSOR_HEIGHT_MM = 4.712
FOCAL_LENGTH_MM = 4.74

SIDE_OVERLAP = 0.20   # 20%
CRUISE_SPEED = 8.0    # m/s

# =====================================================
# CAMERA DERIVED VALUES
# =====================================================
FX = (FOCAL_LENGTH_MM * IMAGE_W) / SENSOR_WIDTH_MM
FY = (FOCAL_LENGTH_MM * IMAGE_H) / SENSOR_HEIGHT_MM

GROUND_W = (IMAGE_W / FX) * MISSION_ALT * 2
GROUND_H = (IMAGE_H / FY) * MISSION_ALT * 2
SPACING = GROUND_W * (1 - SIDE_OVERLAP)

print("📷 Ground footprint: %.1f x %.1f m" % (GROUND_W, GROUND_H))
print("↔️  Line spacing:", round(SPACING, 1), "m")

# =====================================================
# KML PARSER
# =====================================================
def read_polygon_from_kml(kml_file):
    tree = ET.parse(kml_file)
    root = tree.getroot()
    ns = {"kml": "http://www.opengis.net/kml/2.2"}

    coords_elem = root.find(".//kml:coordinates", ns)
    if coords_elem is None:
        raise ValueError("❌ No coordinates found in KML")

    coords = []
    for line in coords_elem.text.strip().split():
        lon, lat, *_ = map(float, line.split(","))
        coords.append((lat, lon))

    if coords[0] == coords[-1]:
        coords.pop()

    return coords

# =====================================================
# METRICS
# =====================================================
def polygon_metrics(boundary):
    to_m = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    poly = Polygon([to_m.transform(lon, lat) for lat, lon in boundary])
    minx, miny, maxx, maxy = poly.bounds
    return poly, (minx, miny, maxx, maxy)

# =====================================================
# FIXED LAWNMOWER GENERATOR
# =====================================================
def generate_lawnmower(boundary):
    poly, (minx, miny, maxx, maxy) = polygon_metrics(boundary)

    to_ll = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)

    width = maxx - minx
    height = maxy - miny

    sweep_x = width > height
    span = height if sweep_x else width

    # 🔒 GUARANTEE at least one sweep
    num_lines = max(1, int(span / SPACING) + 1)

    cx, cy = poly.centroid.coords[0]
    start = cy - (num_lines // 2) * SPACING if sweep_x else cx - (num_lines // 2) * SPACING

    path = []
    direction = 1

    for i in range(num_lines):
        if sweep_x:
            y = start + i * SPACING
            sweep = LineString([(minx - 50, y), (maxx + 50, y)])
        else:
            x = start + i * SPACING
            sweep = LineString([(x, miny - 50), (x, maxy + 50)])

        clipped = sweep.intersection(poly)

        if clipped.is_empty:
            continue

        if clipped.geom_type == "LineString":
            lines = [clipped]
        elif clipped.geom_type == "MultiLineString":
            lines = list(clipped.geoms)
        else:
            continue

        for line in lines:
            pts = list(line.coords)
            if direction < 0:
                pts.reverse()

            for x, y in pts:
                lon, lat = to_ll.transform(x, y)
                path.append((lat, lon))

        direction *= -1

    return path

# =====================================================
# VISUALIZATION (SAFE)
# =====================================================
def visualize(path, boundary):
    if len(path) == 0:
        print("⚠️ Polygon smaller than camera swath → single-pass case")
        return

    b = np.array(boundary + [boundary[0]])
    p = np.array(path)

    plt.figure(figsize=(7, 7))
    plt.plot(b[:, 1], b[:, 0], "y-", lw=2)
    plt.fill(b[:, 1], b[:, 0], alpha=0.15)
    plt.plot(p[:, 1], p[:, 0], "r-", lw=1)
    plt.scatter(p[::5, 1], p[::5, 0], s=15)

    plt.scatter(p[0, 1], p[0, 0], c="green", s=100, label="START")
    plt.scatter(p[-1, 1], p[-1, 0], c="red", s=100, label="END")

    plt.legend()
    plt.axis("equal")
    plt.grid(True)
    plt.title("Lawnmower Coverage Path")
    plt.show()

# =====================================================
# KML EXPORT
# =====================================================
def save_kml(path, boundary):
    kml = simplekml.Kml()

    ls = kml.newlinestring(name="Mission Path")
    ls.coords = [(lon, lat, MISSION_ALT) for lat, lon in path]
    ls.altitudemode = simplekml.AltitudeMode.relativetoground

    bd = kml.newlinestring(name="Boundary")
    bd.coords = [(lon, lat, MISSION_ALT) for lat, lon in boundary + [boundary[0]]]

    kml.save("mission_path.kml")
    print("📁 Saved: mission_path.kml")

# =====================================================
# MAIN
# =====================================================
def main():
    if len(sys.argv) < 2:
        print("Usage: python gbu.py input_polygon.kml")
        return

    kml_file = sys.argv[1]
    if not os.path.exists(kml_file):
        print("❌ File not found")
        return

    boundary = read_polygon_from_kml(kml_file)
    path = generate_lawnmower(boundary)

    print("✅ Waypoints generated:", len(path))
    save_kml(path, boundary)
    visualize(path, boundary)

if __name__ == "__main__":
    main()
