import serial
import json
import time

# -------------------------------------------------
# CONFIG
# -------------------------------------------------
PORT = "COM18"          # change if needed
BAUD = 57600

# -------------------------------------------------
# BOUNDARY COORDINATES (lat, lon), 
# -------------------------------------------------
boundary_points = [
    [28.4163866303, 77.5248487585],
    [28.4162031219, 77.5249590213],
    [28.4164152378, 77.5254725707],
    [28.4165894535, 77.5253594554],
    [28.4163866303, 77.5248487585]
]

# -------------------------------------------------
# OPEN SERIAL
# -------------------------------------------------
ser = serial.Serial(
    port=PORT,
    baudrate=BAUD,
    timeout=1,
    write_timeout=1
)

time.sleep(2.5)  # let telemetry radio stabilize

print(f"[TX] Connected to {PORT}")

# -------------------------------------------------
# SAFE SEND FUNCTION
# -------------------------------------------------
def send_json(obj, label=""):
    msg = json.dumps(obj, separators=(",", ":"))  # compact, strict JSON
    packet = msg + "\n"

    ser.write(packet.encode("utf-8"))
    ser.flush()

    print(f"[TX] Sent {label}: {msg}")
    time.sleep(0.3)  # allow receiver to parse

# -------------------------------------------------
# SEND BOUNDARY
# -------------------------------------------------
boundary_msg = {
    "type": "boundary",
    "points": boundary_points
}

send_json(boundary_msg, "BOUNDARY")

# -------------------------------------------------
# WAIT BEFORE START (IMPORTANT)
# -------------------------------------------------
print("[TX] Waiting before START...")
time.sleep(3.0)

# -------------------------------------------------
# SEND START
# -------------------------------------------------
start_msg = {
    "cmd": "START"
}

send_json(start_msg, "START")

# -------------------------------------------------
# CLOSE
# -------------------------------------------------
time.sleep(0.5)
ser.close()
print("[TX] Done")
