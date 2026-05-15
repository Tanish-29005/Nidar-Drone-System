#!/usr/bin/env python3
"""
Pixhawk KML Mission - GUIDED Mode Execution
- Executes waypoints in GUIDED mode (no AUTO needed)
- Python 3.10 Compatible
- TAKEOFF → Waypoints → LAND all in GUIDED
"""

import time
import xml.etree.ElementTree as ET
from pymavlink import mavutil

# ------------------------------
# CONFIG
# ------------------------------
COM_PORT = "COM14" ### for Raspi dev//tty//ACM0
BAUD = 57600  ### 115200
KML_FILE = "newpath.kml"
TAKEOFF_ALT = 5
WAYPOINT_ALT = 5
WAYPOINT_RADIUS = 2  # meters - how close to get to each waypoint


# --------------------------------
# PARSE KML FILE
# --------------------------------
def parse_kml(path):
    try:
        tree = ET.parse(path)
    except:
        print("❌ ERROR: Could not load KML file!")
        return []

    root = tree.getroot()
    ns = {"kml": "http://www.opengis.net/kml/2.2"}

    coords = []
    for node in root.findall(".//kml:coordinates", ns):
        entries = node.text.strip().split()
        for e in entries:
            lon, lat, *_ = e.split(",")
            coords.append((float(lat), float(lon)))

    return coords


print("Loading KML...")
waypoints = parse_kml(KML_FILE)
print(f"✓ {len(waypoints)} waypoints parsed")

if len(waypoints) == 0:
    print("❌ No waypoints found in KML. Cannot continue.")
    exit()


# --------------------------------
# CONNECT
# --------------------------------
print(f"Connecting to {COM_PORT}...")
master = mavutil.mavlink_connection(COM_PORT, baud=BAUD)
master.wait_heartbeat()
print("✓ Connected (Heartbeat received)")


# --------------------------------
# WAIT FOR GPS LOCK
# --------------------------------
print("Waiting for GPS lock...")
timeout = time.time() + 60
gps_locked = False

while not gps_locked:
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
    if msg:
        fix_type = msg.fix_type
        sats = msg.satellites_visible
        print(f"GPS: Fix={fix_type}, Sats={sats}    ", end="\r")
        if fix_type >= 3:  # Require 3D fix
            gps_locked = True
            print(f"\n✓ GPS locked: Fix type {fix_type} with {sats} satellites")
    
    if time.time() > timeout:
        print("\n❌ GPS lock timeout")
        exit()


# --------------------------------
# SWITCH TO GUIDED & ARM
# --------------------------------
print("Switching to GUIDED mode...")
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    4  # GUIDED mode
)
time.sleep(2)

print("Arming motors...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

timeout = time.time() + 20
armed = False
while not armed and time.time() < timeout:
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        armed = True
        print("✓ Armed")
    time.sleep(0.5)

if not armed:
    print("❌ Arming failed")
    exit()


# --------------------------------
# TAKEOFF
# --------------------------------
print(f"Taking off to {TAKEOFF_ALT}m...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0,  # pitch, empty, empty, yaw
    0, 0,  # lat, lon (0 = use current)
    TAKEOFF_ALT
)

# Wait for takeoff to complete
print("Climbing...")
target_reached = False
timeout = time.time() + 30

while not target_reached and time.time() < timeout:
    msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
    if msg:
        alt = msg.alt
        print(f"Altitude: {alt:.1f}m / {TAKEOFF_ALT}m", end="\r")
        if alt >= TAKEOFF_ALT - 0.5:
            target_reached = True
            print(f"\n✓ Reached takeoff altitude ({alt:.1f}m)")
    time.sleep(0.5)

if not target_reached:
    print("\n❌ Takeoff timeout")
    print("Attempting to land...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    exit()

time.sleep(2)


# --------------------------------
# FLY WAYPOINTS
# --------------------------------
print(f"\nFlying {len(waypoints)} waypoints at {WAYPOINT_ALT}m altitude...")

for i, (lat, lon) in enumerate(waypoints):
    print(f"\n→ Waypoint {i+1}/{len(waypoints)}: ({lat:.6f}, {lon:.6f})")
    
    # Send waypoint command
    master.mav.mission_item_int_send(
        master.target_system,
        master.target_component,
        0,  # seq
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,  # current (2 = guided mode waypoint)
        1,  # autocontinue
        0, 0, 0, 0,  # params
        int(lat * 1e7),
        int(lon * 1e7),
        WAYPOINT_ALT,
        0  # mission_type
    )
    
    # Wait for waypoint to be reached
    target_reached = False
    timeout = time.time() + 60
    
    while not target_reached and time.time() < timeout:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if msg:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0
            
            # Calculate distance to target
            dlat = (lat - current_lat) * 111320  # meters
            dlon = (lon - current_lon) * 111320 * 0.88  # rough correction for latitude
            distance = (dlat**2 + dlon**2)**0.5
            
            print(f"  Distance: {distance:.1f}m | Alt: {current_alt:.1f}m    ", end="\r")
            
            if distance < WAYPOINT_RADIUS:
                target_reached = True
                print(f"\n  ✓ Waypoint reached!")
        
        # Check if still armed
        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("\n⚠ Vehicle disarmed unexpectedly!")
            exit()
        
        time.sleep(0.5)
    
    if not target_reached:
        print(f"\n⚠ Waypoint {i+1} timeout - moving to next")
    
    time.sleep(1)


# --------------------------------
# LAND
# --------------------------------
print(f"\n→ Landing at final waypoint...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0,  # abort alt, precision landing mode, empty, yaw
    0, 0,  # lat, lon (0 = use current)
    0
)

print("Descending...")
landed = False
timeout = time.time() + 60

while not landed and time.time() < timeout:
    msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        mode = msg.custom_mode
        
        alt_msg = master.recv_match(type='VFR_HUD', blocking=False)
        alt = alt_msg.alt if alt_msg else 0
        
        print(f"Altitude: {alt:.1f}m | Armed: {bool(armed)}    ", end="\r")
        
        if not armed:
            landed = True
            print(f"\n✓ Landed and disarmed")
    
    time.sleep(0.5)

if not landed:
    print("\n⚠ Landing timeout")

print("\n✓ Mission complete!")