import time
from typing import List, Optional, Tuple
from math import degrees

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

from .abstract_vehicle import AbstractVehicle     # your vehicle interface
from .sensors import Sensors, GNSSReport
from .rotation import quaternion_from_euler       # you need to implement/choose one





if __name__ == '__main__':



    conn = mavutil.mavlink_connection('udp:127.0.0.1:14560')
    hil = MAVLinkHILSystem(conn, sysid=1, vehicle=your_vehicle_sim)

    while True:
        msg = conn.recv_match(blocking=True, timeout=0.05)
        if msg:
            hil.handle_message(msg)

        hil.update(current_sim_time_us=..., paused=False)