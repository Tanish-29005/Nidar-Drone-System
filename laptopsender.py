#!/usr/bin/env python3
"""
send_three_coords_keypress.py

- Runs on Windows laptop
- Sends exactly 3 GPS coordinates over telemetry
- Sends NEXT coordinate ONLY when user presses 's'
- JSON format compatible with delivery / scout drone scripts
"""

import serial
import json
from datetime import datetime

# -----------------------------
# TELEMETRY CONFIG (WINDOWS)
# -----------------------------
TELEMETRY_PORT = "COM14"      # <-- CHANGE THIS
TELEMETRY_BAUD = 57600

# -----------------------------
# HARD-CODED COORDINATES (3)
# -----------------------------
COORDS = [
    {"lat": 19.0480339, "lon": 72.9123266},
    {"lat": 19.0480732, "lon": 72.9123530},
    {"lat": 19.0480600, "lon": 72.9123000},
]

DRONE_ID = "scout_01"

# -----------------------------
# MAIN
# -----------------------------
print("=" * 60)
print("WINDOWS TELEMETRY COORD SENDER (PRESS 's')")
print("=" * 60)

print(f"[COMM] Opening telemetry port {TELEMETRY_PORT}...")
try:
    ser = serial.Serial(TELEMETRY_PORT, TELEMETRY_BAUD, timeout=1)
except Exception as e:
    print(f"[ERROR] Cannot open telemetry: {e}")
    exit(1)

print("[COMM] ✓ Telemetry connected")
print("\nPress 's' + Enter to send NEXT coordinate")
print("Press Ctrl+C to exit\n")

sent_count = 0

try:
    while sent_count < len(COORDS):
        key = input().strip().lower()

        if key != "s":
            print("[INFO] Press 's' to send next coordinate")
            continue

        c = COORDS[sent_count]

        payload = {
            "type": "coords",
            "drone_id": DRONE_ID,
            "lat": round(c["lat"], 8),
            "lon": round(c["lon"], 8),
            "timestamp": datetime.now().isoformat()
        }

        packet = json.dumps(payload) + "\n"
        ser.write(packet.encode())
        ser.flush()

        sent_count += 1
        print(f"[SEND] #{sent_count} → {payload}")

    print("\n[DONE] All 3 coordinates sent.")

except KeyboardInterrupt:
    print("\n[EXIT] User interrupted")

finally:
    ser.close()
    print("[COMM] Telemetry closed")
