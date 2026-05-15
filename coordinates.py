#!/usr/bin/env python3

import time
import collections

# Python 3.10+ fix
if not hasattr(collections, "MutableMapping"):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect

PORT = "COM14"
BAUD = 57600

print(f"Connecting to {PORT} @ {BAUD}...")
vehicle = connect(PORT, baud=BAUD, wait_ready=False, timeout=30)
print("✓ Connected")

print("\nReading GPS data...\n")

while True:
    gps = vehicle.gps_0
    loc = vehicle.location.global_frame

    if loc.lat is None or loc.lon is None:
        print("⚠ NO GPS FIX YET")
        print(f"Fix type: {gps.fix_type}   Satellites: {gps.satellites_visible}")
        print("-" * 40)
        time.sleep(1)
        continue

    print(f"Lat: {loc.lat:.7f}   Lon: {loc.lon:.7f}")
    print(f"Alt (MSL): {loc.alt:.2f} m")
    print(f"Fix type: {gps.fix_type}   Satellites: {gps.satellites_visible}")
    print("-" * 40)

    time.sleep(1)
