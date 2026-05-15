# import serial
# import json
# import time

# # ================= CONFIG =================
# PORT = "COM14"        # Windows: COMx | Linux/Mac: /dev/ttyUSB0
# BAUD = 57600
# SEND_DELAY = 1.0     # seconds between targets
# # =========================================

# # 🔹 HARD-CODED DROP TARGETS (lat, lon)
# DROP_TARGETS = [
#     (19.04811, 72.91233),
#     (19.04809, 72.91240),
#     (19.04817, 72.91241),
#     (19.04818, 72.91235),
#     (19.04811, 72.91233),
#     (19.04811, 72.91233),
#     (19.04809, 72.91240),
#     (19.04817, 72.91241),
#     (19.04818, 72.91235),
#     (19.04811, 72.91233),
# ]

# def main():
#     try:
#         ser = serial.Serial(PORT, BAUD, timeout=1)
#         print(f"[OK] Connected to {PORT}")
#     except Exception as e:
#         print(f"[ERROR] Serial open failed: {e}")
#         return

#     print(f"\nSending {len(DROP_TARGETS)} drop targets...\n")

#     for idx, (lat, lon) in enumerate(DROP_TARGETS, start=1):
#         packet = {
#             "type": "drop_target",
#             "lat": round(lat, 8),
#             "lon": round(lon, 8)
#         }

#         msg = json.dumps(packet) + "\n"
#         ser.write(msg.encode())
#         ser.flush()

#         print(f"[TX {idx}] {packet}")
#         time.sleep(SEND_DELAY)

#     print("\n✓ All targets sent")
#     ser.close()

# if __name__ == "__main__":
#     main()

import serial
import json

# ================= CONFIG =================
PORT = "COM16"        # Windows: COMx | Linux/Mac: /dev/ttyUSB0
BAUD = 57600
# =========================================

# 🔹 HARD-CODED DROP TARGETS (lat, lon)
DROP_TARGETS = [
    (28.4164982, 77.5253261),
    (28.4164340, 77.5251744),
    (28.4164982, 77.5253261),
    (28.4164340, 77.5251744),
    (28.4164982, 77.5253261),
    (28.4164340, 77.5251744),
    (28.4164982, 77.5253261),
    (28.4164340, 77.5251744),
    (28.4164982, 77.5253261),
    (28.4164340, 77.5251744)
]

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"[OK] Connected to {PORT}")
    except Exception as e:
        print(f"[ERROR] Serial open failed: {e}")
        return

    print("\nPress ENTER to send next drop target")
    print("Type q + ENTER to quit\n")

    index = 0

    while True:
        cmd = input("> ")

        if cmd.lower() == "q":
            print("Exiting sender")
            break

        if index >= len(DROP_TARGETS):
            print("All drop targets already sent")
            continue

        lat, lon = DROP_TARGETS[index]

        packet = {
            "type": "drop_target",
            "lat": round(lat, 8),
            "lon": round(lon, 8)
        }

        msg = json.dumps(packet) + "\n"
        ser.write(msg.encode())
        ser.flush()

        print(f"[TX {index+1}] {packet}")
        index += 1

    ser.close()

if __name__ == "__main__":
    main()
