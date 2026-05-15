from shapely.geometry import Polygon, LineString
from pyproj import Transformer
import simplekml
from lxml import etree

# =====================================================
# USER INPUTS
# =====================================================

BOUNDARY_KML = "bla bla2.kml"
OUTPUT_KML = "path.kml"

ALTITUDE_M = 2
SIDE_OVERLAP = 0.20

# Official Raspberry Pi AI Camera (Sony IMX500)
SENSOR_WIDTH_MM = 6.45
FOCAL_LENGTH_MM = 4.74

# =====================================================
# PROJECTIONS
# =====================================================

to_m = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
to_ll = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)

# =====================================================
# LOAD & PARSE KML (ROBUST)
# =====================================================

with open(BOUNDARY_KML, "rb") as f:
    tree = etree.parse(f)

root = tree.getroot()
ns = {"kml": "http://www.opengis.net/kml/2.2"}

coords_text = root.xpath(
    "//kml:coordinates/text()",
    namespaces=ns
)

if not coords_text:
    raise RuntimeError("❌ No coordinates found in KML")

# Use the FIRST coordinate set (boundary)
raw_coords = coords_text[0].strip().split()

coords_ll = []
for c in raw_coords:
    lon, lat, *_ = map(float, c.split(","))
    coords_ll.append((lon, lat))

# Ensure closed polygon
if coords_ll[0] != coords_ll[-1]:
    coords_ll.append(coords_ll[0])

# =====================================================
# CONVERT TO METERS
# =====================================================

coords_m = [to_m.transform(lon, lat) for lon, lat in coords_ll]
polygon = Polygon(coords_m)

if not polygon.is_valid or polygon.area == 0:
    raise RuntimeError("❌ Invalid polygon geometry")

# =====================================================
# CAMERA FOOTPRINT
# =====================================================

ground_width = (SENSOR_WIDTH_MM / FOCAL_LENGTH_MM) * ALTITUDE_M
line_spacing = ground_width * (1 - SIDE_OVERLAP)

# =====================================================
# GENERATE LAWNMOWER PATH
# =====================================================

minx, miny, maxx, maxy = polygon.bounds
y = miny
direction = 1
path_points = []

while y <= maxy:
    sweep = LineString([(minx - 200, y), (maxx + 200, y)])
    clipped = sweep.intersection(polygon)

    if not clipped.is_empty and clipped.geom_type == "LineString":
        pts = list(clipped.coords)
        if direction == -1:
            pts.reverse()
        path_points.extend(pts)

    y += line_spacing
    direction *= -1

# =====================================================
# EXPORT TO KML
# =====================================================

kml = simplekml.Kml()
coords_out = []

for x, y in path_points:
    lon, lat = to_ll.transform(x, y)
    coords_out.append((lon, lat, ALTITUDE_M))

ls = kml.newlinestring(name="Survey LawnMower Path")
ls.coords = coords_out
ls.altitudemode = simplekml.AltitudeMode.absolute
ls.style.linestyle.width = 3
ls.style.linestyle.color = simplekml.Color.red

kml.save(OUTPUT_KML)

print("✅ Lawnmower path generated successfully")
print(f"📂 Output: {OUTPUT_KML}")
print(f"📏 Line spacing: {line_spacing:.2f} m")
print(f"📸 Ground footprint width: {ground_width:.2f} m")
