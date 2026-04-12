import socket
import struct
import math

def decode_fg_packet(raw_data):
    """
    Decodes the header of an FGNetFDM Version 24 packet.
    """
    # FG expects: 
    # Version (4 bytes), Padding (4 bytes), 
    # Lat (8), Lon (8), Alt (8), 
    # AGL (4), Roll (4), Pitch (4), Yaw (4)
    # Total header decoded here: 48 bytes
    fmt = "!IIdddffff"
    
    try:
        header_size = struct.calcsize(fmt)
        if len(raw_data) < header_size:
            return "Packet too short"

        unpacked = struct.unpack(fmt, raw_data[:header_size])
        
        return {
            "version": unpacked[0],
            "lat_deg": math.degrees(unpacked[2]),
            "lon_deg": math.degrees(unpacked[3]),
            "alt_m":   unpacked[4],
            "roll":    math.degrees(unpacked[6]),
            "pitch":   math.degrees(unpacked[7]),
            "yaw":     math.degrees(unpacked[8])
        }
    except Exception as e:
        return f"Decode Error: {e}"

def start_listener(ip="127.0.0.1", port=5503):
    # Create the UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Bind the socket to the port (Like FlightGear does with 'in')
    try:
        sock.bind((ip, port))
        print(f"Listening for simulation data on {ip}:{port}...")
        print("Press Ctrl+C to stop.\n")
    except Exception as e:
        print(f"Could not bind to port: {e}")
        return

    while True:
        data, addr = sock.recvfrom(1024) # Buffer size 1024
        
        result = decode_fg_packet(data)
        
        if isinstance(result, dict):
            print(f"Source: {addr}")
            print(f"  Pos: {result['lat_deg']:.6f}, {result['lon_deg']:.6f}")
            print(f"  Alt: {result['alt_m']:.2f}m")
            print(f"  Att: R:{result['roll']:.2f} P:{result['pitch']:.2f} Y:{result['yaw']:.2f}")
            print("-" * 40)
        else:
            print(result)

if __name__ == "__main__":
    start_listener()