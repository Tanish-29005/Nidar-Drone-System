


import serial
import json
import time
import sys
from datetime import datetime
from pymavlink import mavutil
import threading

# ============================================================================
# CONFIGURATION
# ============================================================================

# 915MHz Telemetry module connection (Receiver)
TELEMETRY_PORT = 'COM14'  # Windows: COM3, COM4, etc.
# TELEMETRY_PORT = '/dev/ttyUSB0'  # Linux: /dev/ttyUSB0, /dev/ttyUSB1, etc.
# TELEMETRY_PORT = '/dev/tty.usbserial-0001'  # macOS: /dev/tty.usbserial-*
TELEMETRY_BAUD = 57600

# Pixhawk connection (Delivery Drone) - OPTIONAL
# Set to None if not connected
PIXHAWK_PORT = None  # Change to 'COM4' or '/dev/ttyACM0' if connected
PIXHAWK_BAUD = 57600

# Drone identifier
DRONE_ID = "delivery_01"

# Save received data to file
SAVE_TO_FILE = True
LOG_FILE = "coordinates_received.csv"

# ============================================================================
# COLORS FOR TERMINAL OUTPUT
# ============================================================================

class Colors:
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    OKBLUE = '\033[94m'
    RESET = '\033[0m'
    BOLD = '\033[1m'
    CYAN = '\033[96m'
    YELLOW = '\033[33m'
    
    # Windows compatibility
    @staticmethod
    def disable_windows():
        """Disable colors on Windows if not supported"""
        Colors.OKGREEN = ''
        Colors.WARNING = ''
        Colors.FAIL = ''
        Colors.OKBLUE = ''
        Colors.RESET = ''
        Colors.BOLD = ''
        Colors.CYAN = ''
        Colors.YELLOW = ''

# Try to detect Windows and disable colors
if sys.platform == 'win32':
    try:
        import os
        os.system('color')  # Enable ANSI colors on Windows 10+
    except:
        Colors.disable_windows()

# ============================================================================
# TELEMETRY MODULE (915MHz) - RECEIVER
# ============================================================================

class TelemetryReceiver:
    """Receive data via 915MHz telemetry module"""
    
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.connection = None
        self.connected = False
        self.packets_received = 0
        self.bytes_received = 0
        self.read_buffer = ""
        self.last_packet_time = None
        
    def connect(self):
        """Connect to telemetry module"""
        try:
            print(f"\n{Colors.OKBLUE}[TELEMETRY-RX]{Colors.RESET} Connecting to {self.port} @ {self.baudrate} bps...")
            self.connection = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(1)  # Wait for module to initialize
            print(f"{Colors.OKGREEN}[TELEMETRY-RX]{Colors.RESET} ✓ Connected to telemetry module")
            self.connected = True
            return True
        except Exception as e:
            print(f"{Colors.FAIL}[TELEMETRY-RX]{Colors.RESET} ✗ Connection failed: {e}")
            print(f"{Colors.WARNING}Hint: Check COM port and try different ports (COM1, COM2, COM3, COM4, etc.){Colors.RESET}")
            return False
    
    def receive_coordinates(self):
        """Receive and parse coordinates from telemetry"""
        if not self.connected or not self.connection:
            return None
        
        try:
            # Read available data
            if self.connection.in_waiting > 0:
                data = self.connection.read(self.connection.in_waiting)
                self.read_buffer += data.decode('utf-8', errors='ignore')
                self.bytes_received += len(data)
            
            # Check for complete packet (ends with newline)
            if '\n' in self.read_buffer:
                packet, self.read_buffer = self.read_buffer.split('\n', 1)
                
                if packet.strip():
                    try:
                        # Parse JSON
                        coords = json.loads(packet)
                        self.packets_received += 1
                        self.last_packet_time = time.time()
                        return coords
                    except json.JSONDecodeError as e:
                        print(f"{Colors.WARNING}[PARSE]{Colors.RESET} ⚠️  JSON parse error: {e}")
                        return None
            
            return None
        
        except Exception as e:
            print(f"{Colors.WARNING}[TELEMETRY-RX]{Colors.RESET} ⚠️  Receive error: {e}")
            return None
    
    def close(self):
        """Close telemetry connection"""
        if self.connection:
            self.connection.close()

# ============================================================================
# PIXHAWK CONNECTION (DELIVERY DRONE) - OPTIONAL
# ============================================================================

class PixhawkWriter:
    """Write waypoints to Pixhawk"""
    
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.connection = None
        self.connected = False
        self.waypoints_sent = 0
        
    def connect(self):
        """Connect to Pixhawk"""
        try:
            print(f"\n{Colors.OKBLUE}[PIXHAWK]{Colors.RESET} Connecting to {self.port} @ {self.baudrate} bps...")
            self.connection = mavutil.mavlink_connection(self.port, baud=self.baudrate)
            self.connection.wait_heartbeat(timeout=5)
            print(f"{Colors.OKGREEN}[PIXHAWK]{Colors.RESET} ✓ Connected to Pixhawk")
            self.connected = True
            return True
        except Exception as e:
            print(f"{Colors.WARNING}[PIXHAWK]{Colors.RESET} ⚠️  Connection failed (optional): {e}")
            return False
    
    def send_waypoint(self, coords):
        """Send waypoint to Pixhawk"""
        if not self.connected:
            return False
        
        try:
            # Create waypoint
            lat = coords['lat']
            lon = coords['lon']
            alt = coords['alt'] if 'alt' in coords else 10  # Default 10m if not specified
            
            # Send SET_POSITION_TARGET_GLOBAL_INT command
            self.connection.mav.send(
                mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                    time_boot_ms=0,
                    target_system=self.connection.target_system,
                    target_component=self.connection.target_component,
                    coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask=0x0FFF,  # Position fields only
                    lat_int=int(lat * 1e7),
                    lon_int=int(lon * 1e7),
                    alt=alt,
                    vx=0, vy=0, vz=0,
                    afx=0, afy=0, afz=0,
                    yaw=0, yaw_rate=0
                )
            )
            
            self.waypoints_sent += 1
            return True
        
        except Exception as e:
            print(f"{Colors.WARNING}[PIXHAWK]{Colors.RESET} ⚠️  Waypoint send error: {e}")
            return False

# ============================================================================
# FILE LOGGING
# ============================================================================

class DataLogger:
    """Log received coordinates to CSV file"""
    
    def __init__(self, filename):
        self.filename = filename
        self.file = None
        self.initialized = False
        
    def init_file(self):
        """Initialize CSV file with header"""
        try:
            self.file = open(self.filename, 'w')
            # Write header
            self.file.write("Timestamp,Drone_ID,Latitude,Longitude,Altitude,Heading,Packet_Number\n")
            self.file.flush()
            self.initialized = True
            print(f"{Colors.OKGREEN}[LOG]{Colors.RESET} ✓ Logging to {self.filename}")
            return True
        except Exception as e:
            print(f"{Colors.WARNING}[LOG]{Colors.RESET} ⚠️  Failed to create log file: {e}")
            return False
    
    def log_coordinates(self, coords, packet_num):
        """Log coordinates to file"""
        if not self.initialized:
            return False
        
        try:
            timestamp = datetime.now().isoformat()
            drone_id = coords.get('drone_id', 'unknown')
            lat = coords.get('lat', 0)
            lon = coords.get('lon', 0)
            alt = coords.get('alt', 0)
            heading = coords.get('heading', 0)
            
            line = f"{timestamp},{drone_id},{lat:.8f},{lon:.8f},{alt:.2f},{heading:.1f},{packet_num}\n"
            self.file.write(line)
            self.file.flush()
            return True
        except Exception as e:
            print(f"{Colors.WARNING}[LOG]{Colors.RESET} ⚠️  Log error: {e}")
            return False
    
    def close(self):
        """Close log file"""
        if self.file:
            self.file.close()

# ============================================================================
# STATUS DISPLAY
# ============================================================================

def print_header():
    """Print application header"""
    print("\n" + "=" * 90)
    print(f"{Colors.BOLD}MULTI-DRONE COMM TEST - DELIVERY DRONE (RECEIVER) - LAPTOP VERSION{Colors.RESET}")
    print("=" * 90)
    print(f"Telemetry: {TELEMETRY_PORT} @ {TELEMETRY_BAUD} bps (Receiver)")
    
    if PIXHAWK_PORT:
        print(f"Pixhawk:   {PIXHAWK_PORT} @ {PIXHAWK_BAUD} bps (optional waypoint sending)")
    else:
        print(f"Pixhawk:   DISABLED (only monitoring telemetry)")
    
    print(f"Drone ID:  {DRONE_ID}")
    print(f"Data Logging: {'Enabled' if SAVE_TO_FILE else 'Disabled'}")
    print("=" * 90 + "\n")

def print_received(coords, packet_num):
    """Print received coordinates"""
    print(f"\n{Colors.CYAN}{'='*90}{Colors.RESET}")
    print(f"{Colors.OKGREEN}[RECEIVED #{packet_num}]{Colors.RESET} Scout drone coordinates:")
    print(f"  {Colors.BOLD}From:{Colors.RESET} {coords.get('drone_id', 'unknown')}")
    print(f"  {Colors.BOLD}GPS:{Colors.RESET} {coords.get('lat', 0):.8f}°N, {coords.get('lon', 0):.8f}°E")
    print(f"  {Colors.BOLD}Altitude:{Colors.RESET} {coords.get('alt', 0):.2f}m")
    print(f"  {Colors.BOLD}Heading:{Colors.RESET} {coords.get('heading', 0):.1f}°")
    print(f"  {Colors.BOLD}Scout Timestamp:{Colors.RESET} {coords.get('timestamp', 'unknown')}")
    print(f"  {Colors.BOLD}Received Time:{Colors.RESET} {datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
    print(f"{Colors.CYAN}{'='*90}{Colors.RESET}")

def print_statistics(telemetry, pixhawk, elapsed_time):
    """Print statistics"""
    print(f"\n{Colors.BOLD}{'─'*90}{Colors.RESET}")
    print(f"{Colors.BOLD}Statistics:{Colors.RESET}")
    print(f"  Packets received: {telemetry.packets_received}")
    print(f"  Bytes received: {telemetry.bytes_received}")
    print(f"  Average packet size: {telemetry.bytes_received / max(telemetry.packets_received, 1):.1f} bytes")
    print(f"  Data rate: {(telemetry.bytes_received * 8) / max(elapsed_time, 1):.1f} bps")
    
    if pixhawk and pixhawk.connected:
        print(f"  Waypoints sent: {pixhawk.waypoints_sent}")
    
    print(f"  Elapsed time: {elapsed_time:.1f}s")
    print(f"{Colors.BOLD}{'─'*90}{Colors.RESET}\n")

# ============================================================================
# FIND AVAILABLE PORTS
# ============================================================================

def find_serial_ports():
    """Find available serial ports"""
    print(f"\n{Colors.BOLD}Available Serial Ports:{Colors.RESET}\n")
    
    import os
    
    if sys.platform == 'win32':
        # Windows
        for i in range(256):
            try:
                s = serial.Serial(f'COM{i}')
                print(f"  ✓ COM{i} - Available")
                s.close()
            except:
                pass
    else:
        # Linux/Mac
        if sys.platform == 'darwin':
            ports = os.listdir('/dev/')
            usbports = [x for x in ports if 'usbserial' in x or 'wchusbserial' in x]
            for port in usbports:
                print(f"  ✓ /dev/{port}")
        else:
            ports = os.listdir('/dev/')
            ttyports = [x for x in ports if x.startswith('ttyUSB') or x.startswith('ttyACM')]
            for port in ttyports:
                print(f"  ✓ /dev/{port}")
    
    print()

# ============================================================================
# MAIN LOOP
# ============================================================================

def main():
    print_header()
    
    # Initialize telemetry receiver
    telemetry = TelemetryReceiver(TELEMETRY_PORT, TELEMETRY_BAUD)
    if not telemetry.connect():
        print(f"\n{Colors.FAIL}Failed to connect to telemetry receiver{Colors.RESET}")
        print(f"\n{Colors.WARNING}Trying to find available ports...{Colors.RESET}")
        find_serial_ports()
        
        # Suggest common ports
        if sys.platform == 'win32':
            print(f"{Colors.YELLOW}On Windows, try: COM1, COM3, COM4, COM5...{Colors.RESET}")
            print(f"{Colors.YELLOW}Edit line: TELEMETRY_PORT = 'COM3'  (or your port){Colors.RESET}")
        else:
            print(f"{Colors.YELLOW}On Linux, try: /dev/ttyUSB0, /dev/ttyUSB1...{Colors.RESET}")
            print(f"{Colors.YELLOW}On Mac, try: /dev/tty.usbserial-0001, etc.{Colors.RESET}")
        
        return 1
    
    # Initialize Pixhawk connection (optional)
    pixhawk = None
    if PIXHAWK_PORT:
        pixhawk = PixhawkWriter(PIXHAWK_PORT, PIXHAWK_BAUD)
        pixhawk.connect()  # Non-fatal if fails
    
    # Initialize data logger (optional)
    logger = None
    if SAVE_TO_FILE:
        logger = DataLogger(LOG_FILE)
        logger.init_file()
    
    print(f"\n{Colors.BOLD}Waiting for scout drone coordinates...{Colors.RESET}\n")
    
    last_received = None
    start_time = time.time()
    
    try:
        while True:
            # Receive coordinates from telemetry
            coords = telemetry.receive_coordinates()
            
            if coords:
                print_received(coords, telemetry.packets_received)
                last_received = coords
                
                # Log to file
                if logger:
                    logger.log_coordinates(coords, telemetry.packets_received)
                
                # Send waypoint to Pixhawk (if connected)
                if pixhawk and pixhawk.connected:
                    success = pixhawk.send_waypoint(coords)
                    if success:
                        print(f"{Colors.OKGREEN}[ACTION]{Colors.RESET} ✓ Waypoint sent to Pixhawk")
                    else:
                        print(f"{Colors.WARNING}[ACTION]{Colors.RESET} ⚠️  Failed to send waypoint")
                else:
                    print(f"{Colors.OKBLUE}[STATUS]{Colors.RESET} Pixhawk not connected (skipping waypoint)")
                
                # Print current statistics
                elapsed = time.time() - start_time
                print_statistics(telemetry, pixhawk, elapsed)
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print(f"\n\n{Colors.BOLD}Stopping...{Colors.RESET}")
    
    finally:
        # Cleanup
        elapsed = time.time() - start_time
        
        telemetry.close()
        print(f"{Colors.OKBLUE}[CLEANUP]{Colors.RESET} Closed telemetry connection")
        
        if logger:
            logger.close()
            print(f"{Colors.OKBLUE}[CLEANUP]{Colors.RESET} Closed log file")
        
        # Print final statistics
        print(f"\n{Colors.BOLD}Final Statistics:{Colors.RESET}")
        print(f"  Total packets received: {telemetry.packets_received}")
        print(f"  Total bytes received: {telemetry.bytes_received}")
        
        if telemetry.packets_received > 0:
            print(f"  Average packet size: {telemetry.bytes_received / telemetry.packets_received:.1f} bytes")
            print(f"  Data rate: {(telemetry.bytes_received * 8) / elapsed:.1f} bps")
            print(f"  Packet rate: {telemetry.packets_received / elapsed:.1f} Hz")
        
        if pixhawk and pixhawk.connected:
            print(f"  Total waypoints sent: {pixhawk.waypoints_sent}")
        
        if last_received:
            print(f"\n{Colors.OKGREEN}Last received coordinates:{Colors.RESET}")
            print(f"  GPS: {last_received.get('lat', 0):.8f}°N, {last_received.get('lon', 0):.8f}°E")
            print(f"  Altitude: {last_received.get('alt', 0):.2f}m")
            print(f"  Heading: {last_received.get('heading', 0):.1f}°")
        
        if SAVE_TO_FILE and logger and logger.initialized:
            print(f"\n{Colors.OKGREEN}Data saved to: {LOG_FILE}{Colors.RESET}")
        
        print(f"\n{Colors.OKGREEN}✓ Delivery drone receiver test complete{Colors.RESET}\n")
        return 0

if __name__ == "__main__":
    sys.exit(main())