#!/usr/bin/env python3
import serial
import json
import time
from datetime import datetime
from pymavlink import mavutil

# -----------------------------
# PORTS
# -----------------------------
PIXHAWK_PORT = '/dev/ttyACM0'
PIXHAWK_BAUD = 57600

TELEMETRY_PORT = '/dev/ttyUSB0'
TELEMETRY_BAUD = 57600

UPDATE_RATE = 1.0  # send every 1 second

print("\nConnecting to Pixhawk...")
master = mavutil.mavlink_connection(PIXHAWK_PORT, baud=PIXHAWK_BAUD)
master.wait_heartbeat()
print("✓ Connected to Pixhawk")

# Force GPS stream
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    5,
    1
)

print("\nConnecting to telemetry...")
tx = serial.Serial(TELEMETRY_PORT, TELEMETRY_BAUD, timeout=1)
print("✓ Telemetry connected\n")

# variables
lat = lon = alt = 0.0
fix = sat = 0

last_send = time.time()

print("Receiving → Printing → Transmitting...\n")

while True:
    msg = master.recv_match(blocking=False)

    if msg:
        mtype = msg.get_type()

        if mtype == "GPS_RAW_INT":
            fix = msg.fix_type
            sat = msg.satellites_visible

        elif mtype == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0

        # PRINT
        print(f"Lat: {lat:.7f}   Lon: {lon:.7f}")
        print(f"Alt (MSL): {alt:.2f} m")
        print(f"Fix type: {fix}   Satellites: {sat}\n")

        # SEND EVERY 1 SECOND
        if time.time() - last_send >= UPDATE_RATE:
            payload = {
                "type": "coords",
                "drone_id": "scout_01",
                "lat": round(lat, 8),
                "lon": round(lon, 8),
                "alt": round(alt, 2),
                "fix": fix,
                "sat": sat,
                "timestamp": datetime.now().isoformat()
            }

            packet = json.dumps(payload) + "\n"
            tx.write(packet.encode())

            print("→ SENT", payload, "\n")

            last_send = time.time()

    time.sleep(0.01)
