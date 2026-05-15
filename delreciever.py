from pymavlink import mavutil

# Connect to incoming GPS stream
conn = mavutil.mavlink_connection('COM17', baud=57600)

print("Waiting for GPS data on COM15...")

# Wait until connection is alive
conn.wait_heartbeat()
print("Heartbeat received!")

while True:
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.relative_alt / 1000  # meters

        print(f"LAT: {lat:.7f}, LON: {lon:.7f}, ALT: {alt:.2f} m")
