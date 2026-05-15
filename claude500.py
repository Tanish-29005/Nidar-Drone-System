#!/usr/bin/env python3
"""
FSCOUT - PRODUCTION READY SAR SYSTEM
Features:
- Tiled inference for high-altitude detection
- Temporal voting (multi-frame confirmation)
- Altitude-aware detection logic
- Geometric filtering
- Enhanced geotagging with camera tilt compensation
"""

import numpy as np
import cv2
from pymavlink import mavutil
from picamera2 import Picamera2
from ultralytics import YOLO
from shapely.geometry import Polygon, LineString
from pyproj import Transformer
import simplekml

import os
import math
import time
import serial
import json
import threading
from datetime import datetime
from collections import deque, defaultdict

# =========================================================
# PORTS
# =========================================================
PIXHAWK_PORT = "/dev/ttyACM0"
PIXHAWK_BAUD = 57600
TELEM_PORT = "/dev/ttyUSB0"
TELEM_BAUD = 57600

# =========================================================
# MISSION
# =========================================================
TAKEOFF_ALT = 55.0
MISSION_ALT = 55.0
CRUISE_SPEED = 8.0
WAYPOINT_RADIUS = 3.0
DETECTION_START_ALT = 50.0
RTL_AFTER_MISSION = True
BENCH_TEST = False
BENCH_TEST_WAYPOINT_DELAY = 3.0

# =========================================================
# ALTITUDE-AWARE DETECTION MODES
# =========================================================
HIGH_ALT_THRESHOLD = 45.0  # Enable tiled mode above this
MED_ALT_THRESHOLD = 30.0
LOW_ALT_THRESHOLD = 20.0

# =========================================================
# TILED INFERENCE CONFIG
# =========================================================
TILE_SIZE = 640
TILE_OVERLAP = 0.25  # 25% overlap
MIN_TILE_CONF = 0.25

# =========================================================
# TEMPORAL VOTING CONFIG
# =========================================================
TEMPORAL_WINDOW = 5  # Last N frames
MIN_CONFIRMATIONS = 3  # Must appear in 3/5 frames
TEMPORAL_IOU_THRESHOLD = 0.3

# =========================================================
# GEOMETRIC FILTERS
# =========================================================
MIN_ASPECT_RATIO = 1.2
MAX_ASPECT_RATIO = 3.5
MIN_PIXEL_AREA_HIGH_ALT = 400
MIN_PIXEL_AREA_LOW_ALT = 1000

# =========================================================
# SAFETY THRESHOLDS
# =========================================================
GPS_HDOP_MAX = 2.0
GPS_SATS_MIN = 8
GPS_CHECK_INTERVAL = 2.0
DRONE_STATUS_INTERVAL = 2.0

# =========================================================
# CAMERA - IMX500
# =========================================================
IMAGE_W =  4056
IMAGE_H = 3040
SENSOR_WIDTH_MM = 6.81
SENSOR_HEIGHT_MM = 4.71
FOCAL_LENGTH_MM = 4.74

FOCAL_LENGTH_PX_X = (FOCAL_LENGTH_MM * IMAGE_W) / SENSOR_WIDTH_MM
FOCAL_LENGTH_PX_Y = (FOCAL_LENGTH_MM * IMAGE_H) / SENSOR_HEIGHT_MM

CX = IMAGE_W / 2.0
CY = IMAGE_H / 2.0
FX = FOCAL_LENGTH_PX_X
FY = FOCAL_LENGTH_PX_Y

DISTORTION_COEFFS = np.array([-0.15, 0.08, 0.0, 0.0, -0.02])
CAMERA_MATRIX = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float32)

ALT_CORRECTION_FACTORS = {50: 0.98, 55: 0.975, 60: 0.97, 70: 0.965}
STATE_BUFFER_SIZE = 5
MIN_GEOTAG_ALT = 45.0

MODEL_PATH = "/home/tanish/Downloads/best.pt"

print(f"[CAMERA] IMX500 Production: {IMAGE_W}x{IMAGE_H}, f={FX:.1f}px")
print(f"[DETECT] Tiled inference: ENABLED ({TILE_SIZE}px, {TILE_OVERLAP*100:.0f}% overlap)")
print(f"[DETECT] Temporal voting: {MIN_CONFIRMATIONS}/{TEMPORAL_WINDOW} frames")
print(f"[DETECT] Geometric filters: ENABLED")

# =========================================================
# GEO
# =========================================================
METERS_PER_DEG = 111320.0
DUPLICATE_THRESH_M = 6.0

# =========================================================
# OUTPUT
# =========================================================
OUT_DIR = "detections"
KML_OUT = "hehe.kml"
os.makedirs(OUT_DIR, exist_ok=True)
os.makedirs(os.path.join(OUT_DIR, "images"), exist_ok=True)

# =========================================================
# GLOBAL STATE
# =========================================================
boundary = []
start_flag = False
detected_points = []
stop_detection = threading.Event()
emergency_land_flag = threading.Event()

drone_state = {
    'lat': None, 'lon': None, 'alt_msl': None, 'rel_alt': 0.0,
    'heading': 0.0, 'pitch': 0.0, 'roll': 0.0, 'home_alt': None,
    'fix_type': 0, 'sats': 0, 'battery_voltage': 0.0, 'battery_percent': 0
}

state_buffers = {
    'lat': deque(maxlen=STATE_BUFFER_SIZE),
    'lon': deque(maxlen=STATE_BUFFER_SIZE),
    'alt': deque(maxlen=STATE_BUFFER_SIZE),
    'heading': deque(maxlen=STATE_BUFFER_SIZE),
    'pitch': deque(maxlen=STATE_BUFFER_SIZE),
    'roll': deque(maxlen=STATE_BUFFER_SIZE)
}

detection_history = deque(maxlen=TEMPORAL_WINDOW)
candidate_detections = {}
frame_counter = 0

telem_serial = None

# =========================================================
# ROTATION MATRICES
# =========================================================
def Rx(a):
    return np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])

def Ry(a):
    return np.array([[math.cos(a),0,math.sin(a)],[0,1,0],[-math.sin(a),0,math.cos(a)]])

def Rz(a):
    return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])

# =========================================================
# GEOMETRIC FILTERS
# =========================================================
def passes_geometric_filter(bbox, altitude):
    """Filter out non-human shapes"""
    x1, y1, x2, y2 = bbox
    width = x2 - x1
    height = y2 - y1
    
    if width < 1 or height < 1:
        return False, "too_small"
    
    aspect_ratio = height / width
    if aspect_ratio < MIN_ASPECT_RATIO:
        return False, f"too_wide_AR={aspect_ratio:.2f}"
    if aspect_ratio > MAX_ASPECT_RATIO:
        return False, f"too_tall_AR={aspect_ratio:.2f}"
    
    area = width * height
    min_area = MIN_PIXEL_AREA_HIGH_ALT if altitude > HIGH_ALT_THRESHOLD else MIN_PIXEL_AREA_LOW_ALT
    if area < min_area:
        return False, f"area={area:.0f}<{min_area}"
    
    return True, "OK"

# =========================================================
# TILED INFERENCE
# =========================================================
def generate_tiles(image_shape, tile_size=TILE_SIZE, overlap=TILE_OVERLAP):
    """Generate tile coordinates"""
    h, w = image_shape[:2]
    stride = int(tile_size * (1 - overlap))
    
    tiles = []
    for y in range(0, h, stride):
        for x in range(0, w, stride):
            x1, y1 = x, y
            x2, y2 = min(x + tile_size, w), min(y + tile_size, h)
            
            if (x2 - x1) < tile_size * 0.5 or (y2 - y1) < tile_size * 0.5:
                continue
            
            tiles.append((x1, y1, x2, y2))
    
    return tiles

def run_tiled_inference(model, frame, conf_threshold=MIN_TILE_CONF):
    """Run YOLO on tiles and merge results"""
    tiles = generate_tiles(frame.shape)
    all_detections = []
    
    for tile_x1, tile_y1, tile_x2, tile_y2 in tiles:
        tile = frame[tile_y1:tile_y2, tile_x1:tile_x2]
        
        if tile.shape[0] != TILE_SIZE or tile.shape[1] != TILE_SIZE:
            tile_resized = cv2.resize(tile, (TILE_SIZE, TILE_SIZE))
            scale_x = (tile_x2 - tile_x1) / TILE_SIZE
            scale_y = (tile_y2 - tile_y1) / TILE_SIZE
        else:
            tile_resized = tile
            scale_x = scale_y = 1.0
        
        results = model(tile_resized, conf=conf_threshold, verbose=False)
        
        if results[0].boxes and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                
                x1 = int(xyxy[0] * scale_x) + tile_x1
                y1 = int(xyxy[1] * scale_y) + tile_y1
                x2 = int(xyxy[2] * scale_x) + tile_x1
                y2 = int(xyxy[3] * scale_y) + tile_y1
                
                all_detections.append((x1, y1, x2, y2, conf))
    
    if len(all_detections) > 0:
        all_detections = nms_detections(all_detections, iou_threshold=0.4)
    
    return all_detections

def nms_detections(detections, iou_threshold=0.4):
    """Non-Maximum Suppression"""
    if len(detections) == 0:
        return []
    
    boxes = np.array([[d[0], d[1], d[2], d[3]] for d in detections])
    scores = np.array([d[4] for d in detections])
    
    indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), 0.0, iou_threshold)
    
    if len(indices) > 0:
        indices = indices.flatten()
        return [detections[i] for i in indices]
    
    return []

# =========================================================
# TEMPORAL VOTING
# =========================================================
def calculate_iou(box1, box2):
    """Calculate IoU between two boxes"""
    x1_1, y1_1, x2_1, y2_1 = box1
    x1_2, y1_2, x2_2, y2_2 = box2
    
    x1_i = max(x1_1, x1_2)
    y1_i = max(y1_1, y1_2)
    x2_i = min(x2_1, x2_2)
    y2_i = min(y2_1, y2_2)
    
    if x2_i < x1_i or y2_i < y1_i:
        return 0.0
    
    intersection = (x2_i - x1_i) * (y2_i - y1_i)
    area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
    area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
    union = area1 + area2 - intersection
    
    return intersection / union if union > 0 else 0.0

def update_temporal_voting(current_detections, frame_idx):
    """Update temporal voting tracker"""
    global candidate_detections
    
    new_candidates = defaultdict(list)
    
    for det in current_detections:
        bbox_current = det[:4]
        best_match_id = None
        best_iou = 0.0
        
        for track_id, history in candidate_detections.items():
            if len(history) > 0:
                last_bbox = history[-1][1][:4]
                iou = calculate_iou(bbox_current, last_bbox)
                if iou > best_iou and iou > TEMPORAL_IOU_THRESHOLD:
                    best_iou = iou
                    best_match_id = track_id
        
        if best_match_id is not None:
            new_candidates[best_match_id].append((frame_idx, det))
        else:
            new_track_id = f"track_{frame_idx}_{len(new_candidates)}"
            new_candidates[new_track_id].append((frame_idx, det))
    
    for track_id, new_dets in new_candidates.items():
        if track_id in candidate_detections:
            candidate_detections[track_id].extend(new_dets)
        else:
            candidate_detections[track_id] = new_dets
    
    for track_id in list(candidate_detections.keys()):
        candidate_detections[track_id] = [
            d for d in candidate_detections[track_id]
            if frame_idx - d[0] < TEMPORAL_WINDOW
        ]
        if len(candidate_detections[track_id]) == 0:
            del candidate_detections[track_id]
    
    confirmed = []
    for track_id, history in candidate_detections.items():
        if len(history) >= MIN_CONFIRMATIONS:
            latest = history[-1][1]
            confirmed.append(latest)
    
    return confirmed

# =========================================================
# ENHANCED GEOTAGGING
# =========================================================
def undistort_pixel(px, py):
    """Remove lens distortion"""
    x_norm = (px - CX) / FX
    y_norm = (py - CY) / FY
    
    r2 = x_norm**2 + y_norm**2
    r4, r6 = r2 * r2, r2 * r2 * r2
    
    k1, k2, p1, p2, k3 = DISTORTION_COEFFS
    radial = 1 + k1*r2 + k2*r4 + k3*r6
    
    x_distorted = x_norm * radial + 2*p1*x_norm*y_norm + p2*(r2 + 2*x_norm**2)
    y_distorted = y_norm * radial + p1*(r2 + 2*y_norm**2) + 2*p2*x_norm*y_norm
    
    return x_distorted * FX + CX, y_distorted * FY + CY

def get_altitude_correction(altitude):
    """Get altitude-specific correction factor"""
    alts = sorted(ALT_CORRECTION_FACTORS.keys())
    
    if altitude <= alts[0]:
        return ALT_CORRECTION_FACTORS[alts[0]]
    if altitude >= alts[-1]:
        return ALT_CORRECTION_FACTORS[alts[-1]]
    
    for i in range(len(alts) - 1):
        if alts[i] <= altitude <= alts[i+1]:
            a1, a2 = alts[i], alts[i+1]
            c1, c2 = ALT_CORRECTION_FACTORS[a1], ALT_CORRECTION_FACTORS[a2]
            t = (altitude - a1) / (a2 - a1)
            return c1 + t * (c2 - c1)
    
    return 1.0

def get_filtered_state():
    """Get smoothed drone state"""
    if len(state_buffers['lat']) == 0:
        return None
    
    filtered = {
        'lat': np.mean(state_buffers['lat']),
        'lon': np.mean(state_buffers['lon']),
        'alt': np.mean(state_buffers['alt']),
        'pitch': np.mean(state_buffers['pitch']),
        'roll': np.mean(state_buffers['roll'])
    }
    
    headings = np.array(state_buffers['heading'])
    sin_sum = np.sum(np.sin(np.radians(headings)))
    cos_sum = np.sum(np.cos(np.radians(headings)))
    filtered['heading'] = np.degrees(np.arctan2(sin_sum, cos_sum))
    if filtered['heading'] < 0:
        filtered['heading'] += 360
    
    return filtered

def pixel_to_gps_enhanced(cx_px, cy_px, use_filtered_state=True):
    """Enhanced geotagging with tilt compensation"""
    if use_filtered_state:
        drone_data = get_filtered_state()
        if drone_data is None:
            return None
    else:
        drone_data = {
            'lat': drone_state['lat'], 'lon': drone_state['lon'],
            'alt': drone_state['rel_alt'], 'heading': drone_state['heading'],
            'pitch': drone_state['pitch'], 'roll': drone_state['roll']
        }
    
    if drone_data['alt'] < MIN_GEOTAG_ALT:
        return None
    
    cx_undistorted, cy_undistorted = undistort_pixel(cx_px, cy_px)
    
    px = cx_undistorted - CX
    py = cy_undistorted - CY
    
    v_camera = np.array([px/FX, py/FY, 1.0])
    v_camera = v_camera / np.linalg.norm(v_camera)
    
    roll_rad = math.radians(drone_data['roll'])
    pitch_rad = math.radians(drone_data['pitch'])
    heading_rad = math.radians(drone_data['heading'])
    
    R = Rz(heading_rad) @ Ry(pitch_rad) @ Rx(roll_rad)
    v_ned = R @ v_camera
    
    if v_ned[2] <= 0.05:
        return None
    
    altitude_corrected = drone_data['alt'] * get_altitude_correction(drone_data['alt'])
    t = altitude_corrected / v_ned[2]
    
    north_offset = t * v_ned[0]
    east_offset = t * v_ned[1]
    
    lat = drone_data['lat'] + north_offset / METERS_PER_DEG
    cos_lat = math.cos(math.radians(drone_data['lat']))
    lon = drone_data['lon'] + east_offset / (METERS_PER_DEG * cos_lat)
    
    distance = math.sqrt(north_offset**2 + east_offset**2)
    if distance > 150:
        return None
    
    return lat, lon

# =========================================================
# DUPLICATE FILTER
# =========================================================
def is_duplicate(lat, lon):
    for p in detected_points:
        d = math.hypot(
            (lat - p[0]) * METERS_PER_DEG,
            (lon - p[1]) * METERS_PER_DEG * math.cos(math.radians(lat))
        )
        if d < DUPLICATE_THRESH_M:
            return True
    return False

# =========================================================
# SAFETY FUNCTIONS
# =========================================================
def emergency_land_now(master, reason):
    """Immediately switch to LAND mode"""
    print(f"\n{'='*60}")
    print(f"🚨 EMERGENCY LAND: {reason}")
    print(f"{'='*60}\n")
    
    emergency_land_flag.set()
    stop_detection.set()
    
    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print("[SAFETY] LAND command sent")
        
        for _ in range(5):
            try:
                hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if hb and mavutil.mode_string_v10(hb) == "LAND":
                    print("[SAFETY] LAND mode confirmed")
                    return True
            except:
                pass
            time.sleep(0.2)
        
        print("[SAFETY] ⚠️ Could not confirm LAND mode")
        return False
    except Exception as e:
        print(f"[SAFETY] ERROR: {e}")
        return False

def check_gps_quality(master):
    """Check GPS quality"""
    try:
        gps_raw = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
    except:
        return False, "No GPS data"
    
    if not gps_raw:
        return False, "No GPS data"
    
    hdop = gps_raw.eph / 100.0
    sats = gps_raw.satellites_visible
    fix_type = gps_raw.fix_type
    
    if fix_type < 3:
        return False, f"GPS fix insufficient (type {fix_type})"
    if hdop > GPS_HDOP_MAX:
        return False, f"HDOP too high ({hdop:.2f} > {GPS_HDOP_MAX})"
    if sats < GPS_SATS_MIN:
        return False, f"Too few satellites ({sats} < {GPS_SATS_MIN})"
    
    return True, f"GPS OK (HDOP: {hdop:.2f}, Sats: {sats})"

def send_drone_status():
    """Send drone status via telemetry"""
    global telem_serial
    
    if not telem_serial:
        return
    
    try:
        status = {
            "type": "drone_status",
            "timestamp": datetime.now().isoformat(),
            "lat": round(drone_state.get('lat', 0), 8),
            "lon": round(drone_state.get('lon', 0), 8),
            "alt_m": round(drone_state.get('rel_alt', 0), 2),
            "heading_deg": round(drone_state.get('heading', 0), 1),
            "battery_v": round(drone_state.get('battery_voltage', 0), 2),
            "battery_pct": drone_state.get('battery_percent', 0),
            "gps_sats": drone_state.get('sats', 0),
            "gps_fix": drone_state.get('fix_type', 0)
        }
        
        msg = json.dumps(status) + "\n"
        telem_serial.write(msg.encode())
        telem_serial.flush()
    except Exception as e:
        print(f"[TELEM] Status send error: {e}")

# =========================================================
# MAVLINK READER THREAD
# =========================================================
def mavlink_reader(master, stop_event):
    """Background thread to read MAVLink messages"""
    global drone_state
    last_gps_check = time.time()
    last_status_send = time.time()
    
    while not stop_event.is_set() and not emergency_land_flag.is_set():
        try:
            msg = master.recv_match(blocking=True, timeout=1)
        except:
            continue
        
        if not msg:
            continue
        
        msg_type = msg.get_type()
        
        if msg_type == 'GLOBAL_POSITION_INT':
            drone_state['lat'] = msg.lat / 1e7
            drone_state['lon'] = msg.lon / 1e7
            drone_state['alt_msl'] = msg.alt / 1000.0
            drone_state['rel_alt'] = msg.relative_alt / 1000.0
            
            state_buffers['lat'].append(drone_state['lat'])
            state_buffers['lon'].append(drone_state['lon'])
            state_buffers['alt'].append(drone_state['rel_alt'])
        
        elif msg_type == 'ATTITUDE':
            drone_state['pitch'] = math.degrees(msg.pitch)
            drone_state['roll'] = math.degrees(msg.roll)
            hdg = math.degrees(msg.yaw)
            if hdg < 0:
                hdg += 360
            drone_state['heading'] = hdg
            
            state_buffers['pitch'].append(drone_state['pitch'])
            state_buffers['roll'].append(drone_state['roll'])
            state_buffers['heading'].append(hdg)
        
        elif msg_type == 'GPS_RAW_INT':
            drone_state['fix_type'] = msg.fix_type
            drone_state['sats'] = msg.satellites_visible
        
        elif msg_type == 'BATTERY_STATUS' or msg_type == 'SYS_STATUS':
            if msg_type == 'BATTERY_STATUS':
                drone_state['battery_voltage'] = msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else 0
                drone_state['battery_percent'] = msg.battery_remaining if msg.battery_remaining != -1 else 0
            elif msg_type == 'SYS_STATUS':
                drone_state['battery_voltage'] = msg.voltage_battery / 1000.0
                drone_state['battery_percent'] = msg.battery_remaining
        
        elif msg_type == 'HOME_POSITION':
            try:
                drone_state['home_alt'] = msg.altitude / 1000.0
            except:
                pass
        
        elif msg_type == 'HEARTBEAT':
            try:
                current_mode = mavutil.mode_string_v10(msg)
                if current_mode.startswith("Mode(0x"):
                    continue
                if current_mode not in ["GUIDED", "LAND", "RTL", "AUTO"]:
                    print(f"\n[SAFETY] 🚨 RC OVERRIDE - Mode: {current_mode}")
                    emergency_land_flag.set()
                    return
            except:
                pass
        
        if time.time() - last_gps_check > GPS_CHECK_INTERVAL:
            gps_ok, gps_msg = check_gps_quality(master)
            if not gps_ok:
                print(f"\n[SAFETY] 🚨 GPS DEGRADED: {gps_msg}")
                emergency_land_now(master, f"GPS degraded: {gps_msg}")
                return
            last_gps_check = time.time()
        
        if time.time() - last_status_send > DRONE_STATUS_INTERVAL:
            send_drone_status()
            last_status_send = time.time()

# =========================================================
# ANNOTATED IMAGE SAVING
# =========================================================
def save_annotated_detection(frame, detections_data, drone_data, person_count):
    """Save detection with annotations"""
    try:
        annotated = frame.copy()
        
        drone_lat = drone_data.get('lat', 0)
        drone_lon = drone_data.get('lon', 0)
        drone_alt = drone_data.get('rel_alt', 0)
        drone_heading = drone_data.get('heading', 0)
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        info_lines = [
            f"Drone: {drone_lat:.7f}N, {drone_lon:.7f}E",
            f"Alt: {drone_alt:.2f}m | Heading: {drone_heading:.2f}deg",
            f"Time: {timestamp}",
            f"Mode: Tiled+Temporal | Enhanced Geotag"
        ]
        
        y_offset = 20
        for line in info_lines:
            (w, h), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (5, y_offset - 15), (w + 10, y_offset + 5), (0, 0, 0), -1)
            cv2.putText(annotated, line, (8, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                       0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y_offset += 25
        
        for idx, det in enumerate(detections_data, start=1):
            x1, y1, x2, y2 = det['bbox']
            lat, lon = det['gps']
            conf = det['confidence']
            north_offset = det['north_offset']
            east_offset = det['east_offset']
            
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)
            
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            cv2.line(annotated, (cx - 10, cy), (cx + 10, cy), (0, 255, 0), 2)
            cv2.line(annotated, (cx, cy - 10), (cx, cy + 10), (0, 255, 0), 2)
            
            label_text = f"Person {idx} [CONFIRMED]"
            label_y = max(y1 - 100, 30)
            gps_text = f"{lat:.7f}N, {lon:.7f}E"
            offset_text = f"N={north_offset:+.1f}m E={east_offset:+.1f}m"
            conf_text = f"Conf: {conf*100:.1f}%"
            
            label_lines = [label_text, gps_text, offset_text, conf_text]
            label_x = x2 + 10 if x2 + 200 < frame.shape[1] else x1 - 200
            
            for i, line in enumerate(label_lines):
                ly = label_y + i * 20
                (w, h), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                cv2.rectangle(annotated, (label_x - 3, ly - 13),
                            (label_x + w + 3, ly + 5), (0, 200, 0), -1)
                cv2.putText(annotated, line, (label_x, ly),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        
        filename = os.path.join(OUT_DIR, "images",
                               f"detection_{person_count}_{int(time.time())}.jpg")
        cv2.imwrite(filename, annotated)
        print(f"[DETECT] Saved: {filename}")
        
        return filename
        
    except Exception as e:
        print(f"[DETECT] Error saving image: {e}")
        return None

# =========================================================
# DETECTION THREAD
# =========================================================
def detection_thread():
    """Production detection with tiled inference + temporal voting"""
    global drone_state, detected_points, frame_counter
    
    try:
        cam = Picamera2()
        config = cam.create_video_configuration(
            main={"size": (IMAGE_W, IMAGE_H), "format": "BGR888"}
        )
        cam.configure(config)
        cam.start()
        print(f"[DETECT] Camera initialized: {IMAGE_W}x{IMAGE_H}")
        time.sleep(1.5)
    except Exception as e:
        print(f"[DETECT] Camera failed: {e}")
        return
    
    try:
        model = YOLO(MODEL_PATH)
        print("[DETECT] YOLO model loaded")
    except Exception as e:
        print(f"[DETECT] YOLO failed: {e}")
        cam.stop()
        return
    
    person_count = 0
    
    while not stop_detection.is_set() and not emergency_land_flag.is_set():
        try:
            altitude = drone_state.get('rel_alt', 0)
            
            if altitude < DETECTION_START_ALT:
                time.sleep(0.2)
                continue
            
            frame = cam.capture_array()
            frame_counter += 1
            
            # ALTITUDE-AWARE DETECTION MODE
            if altitude > HIGH_ALT_THRESHOLD:
                # HIGH ALTITUDE: Use tiled inference
                raw_detections = run_tiled_inference(model, frame)
                print(f"[DETECT] Tiled mode: {len(raw_detections)} raw detections")
            else:
                # LOW ALTITUDE: Standard full-frame detection
                results = model(frame, conf=0.3, verbose=False)
                raw_detections = []
                
                if results[0].boxes and len(results[0].boxes) > 0:
                    for box in results[0].boxes:
                        xyxy = box.xyxy[0].cpu().numpy()
                        conf = float(box.conf[0])
                        x1, y1, x2, y2 = map(int, xyxy[:4])
                        raw_detections.append((x1, y1, x2, y2, conf))
            
            # GEOMETRIC FILTERING
            filtered_detections = []
            for det in raw_detections:
                x1, y1, x2, y2, conf = det
                passes, reason = passes_geometric_filter((x1, y1, x2, y2), altitude)
                
                if passes:
                    # Add GPS placeholder for temporal voting
                    filtered_detections.append((x1, y1, x2, y2, conf, None))
                else:
                    print(f"[FILTER] Rejected: {reason}")
            
            # TEMPORAL VOTING
            confirmed_detections = update_temporal_voting(filtered_detections, frame_counter)
            
            # GEOTAGGING CONFIRMED DETECTIONS
            if len(confirmed_detections) > 0:
                detections_data = []
                
                for det in confirmed_detections:
                    x1, y1, x2, y2, conf, _ = det
                    cx_px = (x1 + x2) / 2.0
                    cy_px = (y1 + y2) / 2.0
                    
                    gps = pixel_to_gps_enhanced(cx_px, cy_px)
                    
                    if gps and not is_duplicate(*gps):
                        detected_points.append(gps)
                        person_count += 1
                        
                        north_offset = (gps[0] - drone_state['lat']) * METERS_PER_DEG
                        east_offset = (gps[1] - drone_state['lon']) * METERS_PER_DEG * math.cos(math.radians(drone_state['lat']))
                        
                        detections_data.append({
                            'bbox': (x1, y1, x2, y2),
                            'gps': gps,
                            'confidence': conf,
                            'north_offset': north_offset,
                            'east_offset': east_offset
                        })
                        
                        print(f"\n[DETECT] ✅ Person #{person_count} CONFIRMED")
                        print(f"[DETECT] GPS: {gps[0]:.8f}, {gps[1]:.8f}")
                        print(f"[DETECT] Confidence: {conf*100:.1f}%")
                        
                        # Send via telemetry
                        if telem_serial:
                            try:
                                detection_msg = {
                                    "type": "detection",
                                    "person_id": person_count,
                                    "lat": round(gps[0], 8),
                                    "lon": round(gps[1], 8),
                                    "confidence": round(conf, 3),
                                    "drone_lat": round(drone_state['lat'], 8),
                                    "drone_lon": round(drone_state['lon'], 8),
                                    "drone_alt": round(drone_state['rel_alt'], 2),
                                    "timestamp": datetime.now().isoformat()
                                }
                                msg = json.dumps(detection_msg) + "\n"
                                telem_serial.write(msg.encode())
                                telem_serial.flush()
                            except Exception as e:
                                print(f"[DETECT] Telemetry error: {e}")
                
                if detections_data:
                    save_annotated_detection(frame, detections_data, drone_state, person_count)
            
            time.sleep(0.05)
            
        except Exception as e:
            print(f"[DETECT] Error: {e}")
            time.sleep(0.1)
    
    cam.stop()
    print("[DETECT] stopped")

# =========================================================
# LAWNMOWER PATH
# =========================================================
def sanitize_boundary(boundary):
    if len(boundary) >= 2 and boundary[0] == boundary[-1]:
        return boundary[:-1]
    return boundary

def generate_lawnmower(boundary_ll):
    """Generate lawnmower pattern"""
    boundary_ll = sanitize_boundary(boundary_ll)
    
    to_m = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
    to_ll = Transformer.from_crs("EPSG:3857", "EPSG:4326", always_xy=True)
    
    poly = Polygon([to_m.transform(lon, lat) for lat, lon in boundary_ll])
    minx, miny, maxx, maxy = poly.bounds
    
    print(f"[PATH] Boundary: {maxx-minx:.1f}m x {maxy-miny:.1f}m")
    
    ground_width_m = (IMAGE_W / FX) * MISSION_ALT * 2
    ground_height_m = (IMAGE_H / FY) * MISSION_ALT * 2
    
    print(f"[PATH] Camera coverage at {MISSION_ALT}m: {ground_width_m:.2f}m x {ground_height_m:.2f}m")
    
    SIDE_OVERLAP = 0.20
    spacing = ground_width_m * (1 - SIDE_OVERLAP)
    
    print(f"[PATH] Line spacing: {spacing:.2f}m ({SIDE_OVERLAP*100:.0f}% overlap)")
    
    width = maxx - minx
    height = maxy - miny
    
    if width > height:
        sweep_along_x = True
        num_lines = int(np.ceil(height / spacing)) + 1
        print(f"[PATH] Sweeping East-West, {num_lines} lines")
    else:
        sweep_along_x = False
        num_lines = int(np.ceil(width / spacing)) + 1
        print(f"[PATH] Sweeping North-South, {num_lines} lines")
    
    path = []
    direction = 1
    
    for i in range(num_lines):
        if sweep_along_x:
            y = miny + i * spacing
            if y > maxy:
                y = maxy
            sweep = LineString([(minx - 10, y), (maxx + 10, y)])
        else:
            x = minx + i * spacing
            if x > maxx:
                x = maxx
            sweep = LineString([(x, miny - 10), (x, maxy + 10)])
        
        clipped = sweep.intersection(poly)
        
        if clipped.is_empty:
            continue
        
        lines = []
        if clipped.geom_type == "LineString":
            lines = [clipped]
        elif clipped.geom_type == "MultiLineString":
            lines = list(clipped.geoms)
        elif clipped.geom_type == "Point":
            continue
        
        for line in lines:
            if line.length < 1:
                continue
            
            pts = list(line.coords)
            
            if direction < 0:
                pts.reverse()
            
            for x, y in pts:
                lon, lat = to_ll.transform(x, y)
                path.append((lat, lon))
        
        direction *= -1
    
    print(f"[PATH] Generated {len(path)} waypoints")
    
    if len(path) == 0:
        print("[PATH] ERROR: No waypoints! Using boundary fallback")
        path = boundary_ll[:-1] if boundary_ll[0] == boundary_ll[-1] else boundary_ll
        return path
    
    # Save KML
    kml = simplekml.Kml()
    
    ls = kml.newlinestring(name="Scout Path")
    ls.coords = [(lon, lat, MISSION_ALT) for lat, lon in path]
    ls.altitudemode = simplekml.AltitudeMode.relativetoground
    ls.style.linestyle.color = simplekml.Color.red
    ls.style.linestyle.width = 3
    
    boundary_ring = kml.newlinestring(name="Boundary")
    boundary_ring.coords = [(lon, lat, MISSION_ALT) for lat, lon in boundary_ll + [boundary_ll[0]]]
    boundary_ring.altitudemode = simplekml.AltitudeMode.relativetoground
    boundary_ring.style.linestyle.color = simplekml.Color.yellow
    boundary_ring.style.linestyle.width = 2
    
    for i, (lat, lon) in enumerate(path):
        pnt = kml.newpoint(name=f"WP{i+1}", coords=[(lon, lat, MISSION_ALT)])
        pnt.style.iconstyle.color = simplekml.Color.green
        pnt.style.iconstyle.scale = 0.5
    
    kml.save(KML_OUT)
    print(f"[PATH] Saved {KML_OUT}")
    
    if len(path) > 1:
        total_distance = 0
        for i in range(len(path) - 1):
            lat1, lon1 = path[i]
            lat2, lon2 = path[i + 1]
            dlat = (lat2 - lat1) * METERS_PER_DEG
            dlon = (lon2 - lon1) * METERS_PER_DEG * math.cos(math.radians(lat1))
            total_distance += math.sqrt(dlat**2 + dlon**2)
        
        est_time_min = (total_distance / CRUISE_SPEED) / 60
        print(f"[PATH] Distance: {total_distance:.0f}m, Time: {est_time_min:.1f} min")
    
    return path

# =========================================================
# TELEMETRY RX
# =========================================================
def telemetry_rx():
    global boundary, start_flag, telem_serial
    
    try:
        telem_serial = serial.Serial(TELEM_PORT, TELEM_BAUD, timeout=0.1)
        print(f"[COMM] Telemetry active on {TELEM_PORT}")
    except Exception as e:
        print(f"[COMM] Cannot open {TELEM_PORT}: {e}")
        print("[COMM] Using fallback boundary")
        boundary = [
            [19.04799, 72.9123],
            [19.04799, 72.91245],
            [19.04818, 72.91245],
            [19.04818, 72.9123],
            [19.04799, 72.9123]
        ]
        time.sleep(2)
        start_flag = True
        return
    
    buffer = ""
    
    while not emergency_land_flag.is_set():
        try:
            data = telem_serial.read(256)
            if not data:
                time.sleep(0.05)
                continue
            
            buffer += data.decode(errors="ignore")
            
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                
                if not line or not (line.startswith("{") and line.endswith("}")):
                    continue
                
                try:
                    msg = json.loads(line)
                except:
                    continue
                
                print("[RX]", msg)
                
                if msg.get("type") == "boundary":
                    pts = msg.get("points")
                    if isinstance(pts, list) and len(pts) >= 3:
                        boundary = pts
                        print(f"[COMM] Boundary received ({len(boundary)} points)")
                
                elif msg.get("cmd") == "START":
                    start_flag = True
                    print("[COMM] START received")
        
        except Exception as e:
            print(f"[COMM] Error: {e}")
            time.sleep(0.2)

# =========================================================
# MAIN
# =========================================================
def main():
    print("="*60)
    print("SCOUT PRODUCTION SAR SYSTEM")
    print(f"Bench Test: {BENCH_TEST}")
    print("="*60)
    
    threading.Thread(target=telemetry_rx, daemon=True).start()
    
    print("[WAIT] Waiting for boundary...")
    while not boundary:
        time.sleep(0.5)
    
    waypoints = generate_lawnmower(boundary)
    
    print("[WAIT] Waiting for START...")
    while not start_flag:
        time.sleep(0.5)
    
    print(f"[MAV] Connecting to {PIXHAWK_PORT}...")
    try:
        master = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
        master.wait_heartbeat(timeout=10)
        print("[MAV] ✓ Connected")
    except Exception as e:
        print(f"[MAV] Connection error: {e}")
        return
    
    try:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
        )
        time.sleep(0.5)
    except:
        pass
    
    print("[SAFETY] Waiting for GPS lock...")
    gps_timeout = time.time() + 60
    gps_locked = False
    
    while time.time() < gps_timeout and not gps_locked:
        try:
            msg = master.recv_match(blocking=True, timeout=1)
        except:
            continue
        
        if not msg:
            continue
        
        msg_type = msg.get_type()
        
        if msg_type == "GPS_RAW_INT":
            fix = msg.fix_type
            sats = msg.satellites_visible
            print(f"\rGPS: Fix={fix}, Sats={sats}    ", end="", flush=True)
            if fix >= 3:
                gps_locked = True
                print(f"\n[MAV] ✓ GPS locked")
        
        elif msg_type == "GLOBAL_POSITION_INT":
            gps_locked = True
            print("\n[MAV] ✓ GPS telemetry active")
    
    if not gps_locked:
        print("\n[MAV] ❌ GPS lock timeout")
        return
    
    for attempt in range(3):
        gps_ok, gps_msg = check_gps_quality(master)
        if gps_ok:
            print(f"[SAFETY] ✓ {gps_msg}")
            break
        print(f"[SAFETY] ⚠️ Attempt {attempt+1}/3: {gps_msg}")
        if attempt == 2:
            print("[SAFETY] ❌ GPS quality insufficient")
            return
        time.sleep(2)
    
    print("[MAV] Switching to GUIDED mode...")
    try:
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4
        )
    except Exception as e:
        print(f"[MAV] set_mode error: {e}")
        return
    time.sleep(2)
    
    print("[MAV] Arming motors...")
    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
    except Exception as e:
        print(f"[MAV] arm error: {e}")
        return
    
    timeout_time = time.time() + 20
    armed = False
    while not armed and time.time() < timeout_time:
        try:
            hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        except:
            hb = None
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            print("[MAV] ✓ Armed")
        time.sleep(0.5)
    
    if not armed:
        print("[MAV] ❌ Arming failed")
        return
    
    print(f"[MAV] Taking off to {TAKEOFF_ALT}m...")
    try:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, TAKEOFF_ALT
        )
    except Exception as e:
        print(f"[MAV] takeoff error: {e}")
        return
    
    if not BENCH_TEST:
        print("[MAV] Climbing...")
        target_reached = False
        timeout_time = time.time() + 45
        
        while not target_reached and time.time() < timeout_time:
            try:
                vfr = master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
            except:
                vfr = None
            if vfr:
                alt = vfr.alt
                print(f"\r[MAV] Altitude: {alt:.1f}m / {TAKEOFF_ALT}m", end="", flush=True)
                if alt >= TAKEOFF_ALT - 0.5:
                    target_reached = True
                    print(f"\n[MAV] ✓ Reached {alt:.1f}m")
            time.sleep(0.5)
        
        if not target_reached:
            print("\n[MAV] ❌ Takeoff timeout")
            emergency_land_now(master, "Takeoff altitude not reached")
            return
    else:
        print("[BENCH TEST] Simulating takeoff...")
        time.sleep(3)
        print("[MAV] ✓ Altitude confirmed (simulated)")
    
    time.sleep(2.0)
    
    mav_thread = threading.Thread(target=mavlink_reader, args=(master, stop_detection), daemon=True)
    det_thread = threading.Thread(target=detection_thread, daemon=True)
    mav_thread.start()
    det_thread.start()
    
    print(f"\n[MAV] Flying {len(waypoints)} waypoints...")
    
    for i, (lat, lon) in enumerate(waypoints):
        if emergency_land_flag.is_set():
            print("[MISSION] ⚠️ Emergency abort")
            return
        
        print(f"\n→ Waypoint {i+1}/{len(waypoints)}: ({lat:.6f}, {lon:.6f})")
        
        try:
            master.mav.mission_item_int_send(
                master.target_system, master.target_component,
                0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                2, 1, 0, 0, 0, 0,
                int(lat * 1e7), int(lon * 1e7), MISSION_ALT, 0
            )
        except Exception as e:
            print(f"[MAV] waypoint send error: {e}")
        
        if BENCH_TEST:
            print("[BENCH TEST] Simulating flight...")
            start_time = time.time()
            while time.time() - start_time < BENCH_TEST_WAYPOINT_DELAY:
                if emergency_land_flag.is_set():
                    return
                time.sleep(0.3)
            print(f"  ✓ Waypoint {i+1} reached (simulated)")
        else:
            target_reached = False
            timeout_time = time.time() + 60
            
            while not target_reached and time.time() < timeout_time:
                if emergency_land_flag.is_set():
                    return
                
                cur_lat = drone_state.get('lat')
                cur_lon = drone_state.get('lon')
                cur_alt = drone_state.get('rel_alt', 0.0)
                
                if cur_lat and cur_lon:
                    dlat = (lat - cur_lat) * METERS_PER_DEG
                    dlon = (lon - cur_lon) * METERS_PER_DEG * math.cos(math.radians(cur_lat))
                    distance = math.sqrt(dlat*dlat + dlon*dlon)
                    
                    print(f"\r  Distance: {distance:.1f}m | Alt: {cur_alt:.1f}m    ", end="", flush=True)
                    
                    if distance < WAYPOINT_RADIUS:
                        target_reached = True
                        print("\n  ✓ Waypoint reached!")
                        break
                
                time.sleep(0.5)
            
            if not target_reached:
                print(f"\n⚠ Waypoint {i+1} timeout")
        
        time.sleep(1.0)
    
    stop_detection.set()
    
    if not emergency_land_flag.is_set():
        if RTL_AFTER_MISSION:
            print("\n[MAV] Mission complete - RTL...")
            try:
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
            except:
                pass
            
            print("[MAV] Returning to home...")
            rtl_complete = False
            timeout_time = time.time() + 120
            
            while not rtl_complete and time.time() < timeout_time:
                try:
                    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                except:
                    hb = None
                
                if hb:
                    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    cur_alt = drone_state.get('rel_alt', 0.0)
                    
                    print(f"\r[MAV] Alt: {cur_alt:.1f}m | Armed: {armed}    ", end="", flush=True)
                    
                    if not armed:
                        rtl_complete = True
                        print("\n[MAV] ✓ RTL complete")
                
                time.sleep(0.5)
        
        else:
            print("\n[MAV] Mission complete - Landing...")
            try:
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
            except:
                pass
            
            landed = False
            timeout_time = time.time() + 60
            
            while not landed and time.time() < timeout_time:
                try:
                    hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                except:
                    hb = None
                if hb:
                    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    cur_alt = drone_state.get('rel_alt', 0.0)
                    print(f"\r[MAV] Alt: {cur_alt:.1f}m | Armed: {armed}    ", end="", flush=True)
                    if not armed:
                        landed = True
                        print("\n[MAV] ✓ Landed")
                time.sleep(0.5)
    
    print("\n✓ Mission complete")
    print(f"\n📊 MISSION SUMMARY:")
    print(f"Total detections: {len(detected_points)}")
    print(f"Detection mode: Tiled + Temporal Voting")
    print(f"Geotagging: Enhanced with camera tilt compensation")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[ABORT] Ctrl+C")
        if 'master' in locals():
            emergency_land_now(master, "User abort")
    except Exception as e:
        print(f"\n[ABORT] Error: {e}")
        if 'master' in locals():
            emergency_land_now(master, f"Error: {e}")