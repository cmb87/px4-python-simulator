import time
import numpy as np
from typing import List, Optional, Tuple
from math import degrees

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


from ..sim.abstractVehicle import AbstractVehicle     # your vehicle interface
from ..sim.quaternion import Quaternion

class MAVLinkHILSystem:
    """
    MAVLink HIL (Hardware-in-the-Loop / Software-in-the-Loop) bridge
    between simulated vehicle and real autopilot (Pixhawk, etc.)
    """

    def __init__(self,
                 connection: mavutil.mavlink_connection,
                 sysid: int,
                 compid: int = 191,          # usually 191 = SIMULATION, 192 = Gazebo, etc.
                 vehicle: AbstractVehicle = None):

        self.connection = connection
        self.sysid = sysid
        self.compid = compid
        self.vehicle = vehicle

        self.got_heartbeat = False
        self.inited = False
        self.stopped = False
        self.got_hil_actuator_controls = False

        # Publishing intervals (-1 = disabled)
        self.hil_state_interval_us = -1          # HIL_STATE_QUATERNION
        self.next_hil_state_pub_us = 0

        self.last_heartbeat_ms = 0
        self.time_throttle_counter = 0

        # For reset detection
        self._reset_pending = False


    def got_hil_actuator_controls(self) -> bool:
        return self.got_hil_actuator_controls


    def handle_message(self, msg: mavlink2.MAVLink_message):
        """Main message handler - called when new message arrives"""
        if msg is None:
            return

        msg_type = msg.get_type()
        t_now_ms = self.vehicle.get_sim_time_ms()   # you need to implement this

        # ------------------------------
        if msg_type == "HIL_ACTUATOR_CONTROLS":
            self.got_hil_actuator_controls = True

            controls = [float(v) for v in msg.controls[:8]]

            # Try to read arming state from mode field (very old PX4 style)
            armed = True
            if msg.mode != 0:
                armed = bool(msg.mode & 0x80)   # bit 7 = armed flag in old convention

            if self.vehicle is not None:
                self.vehicle.u = controls
                self.vehicle.armed = armed

        elif msg_type == "COMMAND_LONG":
            if msg.command == mavlink2.MAV_CMD_SET_MESSAGE_INTERVAL:
                msg_id = int(msg.param1 + 0.5)
                interval_us = int(msg.param2 + 0.5)

                if msg_id == mavlink2.MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
                    self.hil_state_interval_us = interval_us
                    print(f"HIL_STATE_QUATERNION interval set to {interval_us} µs")

        elif msg_type == "HEARTBEAT":
            now_real_ms = int(time.time() * 1000)

            # Timeout detection → reset connection state
            if self.got_heartbeat and (now_real_ms - self.last_heartbeat_ms > 3000):
                print(f"HEARTBEAT timeout ({now_real_ms - self.last_heartbeat_ms} ms) → reset")
                self.reset()

            if not self.got_heartbeat and not self.stopped:
                if self.sysid < 0 or self.sysid == msg.system:
                    self.sysid = msg.system
                    self.got_heartbeat = True
                    print(f"MAVLink connection established (sysid={self.sysid})")
                    self.init_mavlink()
                elif self.sysid != msg.system:
                    print(f"WARNING: Got heartbeat from foreign system {msg.system} "
                          f"(expected {self.sysid})")

            # Disarm when autopilot disarms (old behavior)
            if not (msg.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED):
                if self.vehicle is not None:
                    self.vehicle.set_control([])

            self.last_heartbeat_ms = now_real_ms

        elif msg_type == "STATUSTEXT":
            print(f"STATUSTEXT: {msg.text.decode(errors='ignore').rstrip('\x00')}")


    def init_mavlink(self):
        """Called when first valid heartbeat received"""
        if self.vehicle is None:
            return

        if self.vehicle.sensors.gps_start_time < 0:
            self.vehicle.sensors.gps_start_time = self.get_sim_time_ms() + 1000

        self.stopped = False
        self.inited = True

        # Optional default initial attitude (example: nose up 90°)
        # You can remove/comment this block
        # rot = quaternion_from_euler(0.0,  # roll
        #                             90.0, # pitch
        #                             0.0)  # yaw
        # self.vehicle.set_attitude_quaternion(rot)


    def end_simulation(self):
        if not self.inited:
            return

        self.inited = False
        self.got_heartbeat = False
        self.stopped = True
        if self.vehicle is not None:
            self.vehicle.sensors.gps_start_time = -1


    def update(self, current_sim_time_us: int, paused: bool = False):
        """
        Main update function - should be called in simulation loop
        current_sim_time_us → simulation time in microseconds
        """
        if paused or not self.inited or self.vehicle is None:
            return

        sensors = self.vehicle.sensors


        # ── HIL_SENSOR ───────────────────────────────────────────────────────
        msg = self.connection.mav.hil_sensor_encode(
            time_usec=current_sim_time_us,
            xacc=sensors['accelerometer'][0],
            yacc=sensors['accelerometer'][1],
            zacc=sensors['accelerometer'][2],
            xgyro=sensors['gyroscope'][0],
            ygyro=sensors['gyroscope'][1],
            zgyro=sensors['gyroscope'][2],
            xmag=sensors['magnetometer'][0],
            ymag=sensors['magnetometer'][1],
            zmag=sensors['magnetometer'][2],
            abs_pressure=sensors['barometer']["staticAbsolute"]*0.01,  # Pa → mbar
            pressure_alt=sensors['barometer']["static"]*0.01,
            temperature=0,                               # usually not used
            fields_updated=(0xFFF if not sensors.reset else (1 << 31))
        )
        self.connection.mav.send(msg)

        if sensors.reset:
            sensors.reset = False

        # ── HIL_STATE_QUATERNION (optional / on request) ─────────────────────
        if (self.hil_state_interval_us > 0 and current_sim_time_us >= self.next_hil_state_pub_us):


            pos = self.vehicle.y[0:3]            # ned
            quaternions = self.vehicle.y[3:7]
            vel = self.vehicle.y[7:10]           # in Body frame
            rates = self.vehicle.y[10:13]        # Rates
            acc_body = self.vehicle.ydot[7:10]   


            wind = self.vehicle.wind[:3]
            airspeed_vec = vel - wind
            true_airspeed = np.linalg.norm(airspeed_vec)

            self.connection.mav.hil_state_quaternion_encode(
                time_usec=current_sim_time_us,
                attitude_quaternion=quaternions,
                rollspeed=rates[0],
                pitchspeed=rates[1],
                yawspeed=rates[2],
                lat=int(sensors['gps']['latitude_deg'] * 1e7),
                lon=int(sensors['gps']['longitude_deg'] * 1e7),
                alt=int(pos[2] * 1000),   # mm
                vx=int(vel[0] * 100),
                vy=int(vel[1] * 100),
                vz=int(vel[2] * 100),
                ind_airspeed=0,                     # usually not used
                true_airspeed=int(true_airspeed * 100),
                xacc=int(acc_body[0] * 1000),
                yacc=int(acc_body[1] * 1000),
                zacc=int(acc_body[2] * 1000)
            )

            self.next_hil_state_pub_us = current_sim_time_us + self.hil_state_interval_us

        # ── HIL_GPS ───────────────────────────────────────────────────────────
        if sensors.gps_updated:
            gps = self.vehicle.sensors['gps']

            if gps is not None and gps.position is not None:
                self.connection.mav.hil_gps_encode(
                    time_usec=current_sim_time_us,
                    lat=int(gps['latitude_deg'] * 1e7),
                    lon=int(gps['longitude_deg'] * 1e7),
                    alt=int(gps['longitude_deg'] * 1e3),   # mm
                    eph=int(gps.eph * 100),
                    epv=int(gps.epv * 100),
                    vel=int(gps.speed_ms * 100),
                    vn=int(gps.velocity.x * 100),
                    ve=int(gps.velocity.y * 100),
                    vd=int(gps.velocity.z * 100),
                    cog=int(degrees(gps.cog) * 100),
                    fix_type=gps.fix,
                    satellites_visible=10           # fake value
                )
            sensors.gps_updated = False

        # ── SYSTEM_TIME (every ~1000 cycles) ─────────────────────────────────
        self.time_throttle_counter += 1
        if self.time_throttle_counter % 1000 == 0:
            self.connection.mav.system_time_encode(
                time_unix_usec=int(time.time() * 1e6),
                time_boot_ms=current_sim_time_us // 1000
            )


    def reset(self):
        self.got_heartbeat = False
        self.inited = False
        self.stopped = False
        self.got_hil_actuator_controls = False
        self.next_hil_state_pub_us = 0



    

if __name__ == '__main__':



    conn = mavutil.mavlink_connection('udp:127.0.0.1:14560')
    hil = MAVLinkHILSystem(conn, sysid=1, vehicle=your_vehicle_sim)

    while True:
        msg = conn.recv_match(blocking=True, timeout=0.05)
        if msg:
            hil.handle_message(msg)

        hil.update(current_sim_time_us=..., paused=False)