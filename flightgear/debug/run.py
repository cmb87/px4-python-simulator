import socket
import struct
import math
import time

# Configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 5503

# Munich Center (EDDM)
BASE_LAT = 48.353865
BASE_LON = 11.781591
BASE_ALT = 600.0  # Fly a bit higher to see the turn

# Circle Settings
RADIUS_METERS = 500.0  # 500 meter radius
CIRCULAR_SPEED = 0.5   # Radians per second

def pack_fdm_packet(lat_deg, lon_deg, alt_m, roll_deg, pitch_deg, yaw_deg):
    """
    Matched to your working simulation: Lon, Lat, Alt
    """
    version = 24
    fmt = "!IIdddffff"
    
    # Convert to Radians for the binary packet
    header = struct.pack(fmt,
        version, 0,
        math.radians(lon_deg), # Lon First
        math.radians(lat_deg), # Lat Second
        alt_m,
        alt_m,                 # AGL
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg)
    )
    return header + (b'\x00' * (408 - len(header)))

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start_time = time.time()
    
    print(f"Flying circles around Munich... Radius: {RADIUS_METERS}m")

    try:
        while True:
            t = (time.time() - start_time) * CIRCULAR_SPEED
            
            # 1. Calculate Position
            # We use 111,111 meters per degree of latitude as a standard approximation
            lat_offset = (RADIUS_METERS * math.sin(t)) / 111111.0
            lon_offset = (RADIUS_METERS * math.cos(t)) / (111111.0 * math.cos(math.radians(BASE_LAT)))
            
            current_lat = BASE_LAT + lat_offset
            current_lon = BASE_LON + lon_offset
            
            # 2. Calculate Tangent Orientation (Yaw)
            # In math, angle t starts East and goes CCW. 
            # In aviation, 0 is North and goes CW.
            # To be tangent to the circle moving forward:
            yaw_deg = 360 - math.degrees(t)
            
            # Optional: Add a slight bank (roll) into the turn for realism
            # Banking toward the center of the circle
            roll_deg = 20.0 

            # 3. Send Packet
            packet = pack_fdm_packet(
                current_lat, 
                current_lon, 
                BASE_ALT, 
                roll_deg, 
                0,             # Pitch 0 (level flight)
                yaw_deg
            )
            sock.sendto(packet, (UDP_IP, UDP_PORT))
            
            time.sleep(1/30)

    except KeyboardInterrupt:
        print("\nLanding...")

if __name__ == "__main__":
    main()