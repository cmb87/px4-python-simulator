#!/usr/bin/env python3
"""MAVLink lockstep simulation endpoint backed by vehicle.World.

Compared to dummy_mavlink_loop.py, this loop uses the real 6DOF world model and
its integrated sensor suite for HIL message publication.
"""

import os
import sys
import time
from typing import Any

import numpy as np
from pymavlink import mavutil

vehicle_dir = os.path.join(os.path.dirname(__file__), "vehicle")
if vehicle_dir not in sys.path:
    sys.path.insert(0, vehicle_dir)

from parameters import Parameters
from world import World



RATE_HZ = 250
DT_US = int(1e6 / RATE_HZ)
HEARTBEAT_INTERVAL_US = 1_000_000
SYSTEM_TIME_INTERVAL_US = 1_000_000
GPS_INTERVAL_US = 52_000

CHECK_FACTOR = 2


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def get_sim_millis(sim_time_us: int) -> int:
    return sim_time_us // 1000


def build_initial_state() -> np.ndarray:
    y0 = np.zeros(13)
    y0[0:3] = np.array([0.0, 0.0, -3.0])
    y0[3] = 1.0
    y0[7:10] = np.array([0.0, 0.0, 0.0])
    y0[10:13] = np.array([0.0, 0.0, 0.0])
    return y0


def controls_to_u(latest_controls: tuple[float, ...] | None, armed: bool) -> np.ndarray:
    u = np.zeros(4)
    if (not armed) or latest_controls is None:
        return u

    if len(latest_controls) > 0:
        u[0] = float(latest_controls[0])
    if len(latest_controls) > 1:
        u[1] = float(latest_controls[1])
    if len(latest_controls) > 2:
        u[2] = float(latest_controls[2])
    if len(latest_controls) > 3:
        u[3] = clamp(float(latest_controls[3]), 0.0, 1.0)

    return u


def build_hil_sensor_sample(sim_time_us: int, z: dict[str, Any]) -> dict[str, float | int]:
    acc = np.asarray(z["accelerometer"], dtype=float)
    gyro = np.asarray(z["gyroscope"], dtype=float)
    mag = np.asarray(z["magnetometer"], dtype=float)
    baro = z["barometer"]
    gps = np.asarray(z["gps"], dtype=float)

    return {
        "time_usec": int(sim_time_us),
        "xacc": float(acc[0]),
        "yacc": float(acc[1]),
        "zacc": float(acc[2]),
        "xgyro": float(gyro[0]),
        "ygyro": float(gyro[1]),
        "zgyro": float(gyro[2]),
        "xmag": float(mag[0]),
        "ymag": float(mag[1]),
        "zmag": float(mag[2]),
        "abs_pressure": float(baro["staticAbsolute"]) * 0.01,
        "diff_pressure": float(baro["dynamic"]) * 0.01,
        "pressure_alt": float(gps[2]),
        "temperature": 15.0,
        "fields_updated": 7167,
    }


def build_hil_state_quaternion_sample(sim_time_us: int, world_out: dict[str, Any]) -> dict[str, Any]:
    y = np.asarray(world_out["y"], dtype=float)
    z = world_out["sensors"]
    gps = np.asarray(z["gps"], dtype=float)
    accel = np.asarray(z["accelerometer"], dtype=float)

    vel_north = float(gps[3])
    vel_east = float(gps[4])
    vel_up = float(gps[5])
    vel_down = -vel_up

    horiz_speed_m_s = float(np.hypot(vel_north, vel_east))
    m_s2_to_mg = 1000.0 / 9.80665

    return {
        "time_usec": int(sim_time_us),
        "attitude_quaternion": [float(v) for v in y[3:7]],
        "rollspeed": float(y[10]),
        "pitchspeed": float(y[11]),
        "yawspeed": float(y[12]),
        "lat": int(round(float(gps[0]) * 1e7)),
        "lon": int(round(float(gps[1]) * 1e7)),
        "alt": int(round(float(gps[2]) * 1000.0)),
        "vx": int(round(vel_north * 100.0)),
        "vy": int(round(vel_east * 100.0)),
        "vz": int(round(vel_down * 100.0)),
        "ind_airspeed": int(round(horiz_speed_m_s * 100.0)),
        "true_airspeed": int(round(horiz_speed_m_s * 100.0)),
        "xacc": int(round(float(accel[0]) * m_s2_to_mg)),
        "yacc": int(round(float(accel[1]) * m_s2_to_mg)),
        "zacc": int(round(float(accel[2]) * m_s2_to_mg)),
    }


def build_hil_gps_sample(sim_time_us: int, z: dict[str, Any]) -> dict[str, int]:
    gps = np.asarray(z["gps"], dtype=float)
    vel_north = float(gps[3])
    vel_east = float(gps[4])
    vel_up = float(gps[5])
    vel_down = -vel_up

    vel_3d = float(np.linalg.norm(np.array([vel_north, vel_east, vel_down])))
    cog_rad = float(np.arctan2(vel_east, vel_north))
    if cog_rad < 0.0:
        cog_rad += 2.0 * np.pi

    return {
        "time_usec": int(sim_time_us),
        "fix_type": 3,
        "lat": int(round(float(gps[0]) * 1e7)),
        "lon": int(round(float(gps[1]) * 1e7)),
        "alt": int(round(float(gps[2]) * 1000.0)),
        "eph": 100,
        "epv": 100,
        "vel": int(round(vel_3d * 100.0)),
        "vn": int(round(vel_north * 100.0)),
        "ve": int(round(vel_east * 100.0)),
        "vd": int(round(vel_down * 100.0)),
        "cog": int(round(np.degrees(cog_rad) * 100.0)),
        "satellites_visible": 10,
    }


def sim_side() -> None:

    conn: Any = mavutil.mavlink_connection("tcpin:0.0.0.0:4560", source_component=51)

    print("Waiting for Heartbeat ...")
    first_hb = conn.wait_heartbeat()
    px4_sysid = first_hb.get_srcSystem()
    if px4_sysid > 0:
        conn.source_system = px4_sysid
        conn.mav.srcSystem = px4_sysid
        print(f"Locked simulator SYSID to PX4 SYSID={px4_sysid}")

    p = Parameters()
    world = World(parameters=p, y0=build_initial_state(), u0=np.zeros(4), wind0=np.zeros(6))

    sim_time_us = 0
    next_heartbeat_time_us = 0
    next_system_time_us = 0
    next_gps_time_us = 0
    last_time_ran_ms = 0
    slow_down_counter = 0
    got_hil_actuator_controls = False
    latest_controls: tuple[float, ...] | None = None
    armed = False
    was_armed = False
    ever_armed = False
    last_rx_wall_s = time.time()
    next_rx_warn_wall_s = last_rx_wall_s + 2.0

    print("PX4 connected starting sim")
    while True:

        while True:
            msg = conn.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            last_rx_wall_s = time.time()

            if msg_type == "HEARTBEAT":
                print("SIM <= HEARTBEAT")

            elif msg_type == "HIL_ACTUATOR_CONTROLS":
                got_hil_actuator_controls = True
                controls = getattr(msg, "controls", None)
                if isinstance(controls, (list, tuple)):
                    latest_controls = tuple(float(v) for v in controls)
                else:
                    latest_controls = None

                mode = int(getattr(msg, "mode", 0))
                if mode != 0:
                    armed = (mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    if armed:
                        ever_armed = True



        if armed and (not was_armed):
            print("SIM => ARM transition: dynamics enabled")
        if (not armed) and was_armed:
            print("SIM => DISARM transition: dynamics remain enabled")

        was_armed = armed

        now_wall_s = time.time()
        if now_wall_s >= next_rx_warn_wall_s and (now_wall_s - last_rx_wall_s) > 2.0:
            print("SIM: no MAVLink RX for >2s")
            next_rx_warn_wall_s = now_wall_s + 2.0

        sim_time_us += DT_US

        io_run_only = (slow_down_counter % CHECK_FACTOR) != 0
        #if (not got_hil_actuator_controls) and (not io_run_only):
        #    print("advancing without hilActuators")

        now_ms = get_sim_millis(sim_time_us)
        needs_to_pause = (last_time_ran_ms == now_ms) or io_run_only

        world.set_controls(controls_to_u(latest_controls, armed))
        world_out = world.update(sim_time_us, needs_to_pause, freeze_dynamics=(not ever_armed))

        if (not needs_to_pause) and world_out is not None:
            z = world_out["sensors"]

            sensor = build_hil_sensor_sample(sim_time_us, z)
            conn.mav.hil_sensor_send(
                sensor["time_usec"],
                sensor["xacc"],
                sensor["yacc"],
                sensor["zacc"],
                sensor["xgyro"],
                sensor["ygyro"],
                sensor["zgyro"],
                sensor["xmag"],
                sensor["ymag"],
                sensor["zmag"],
                sensor["abs_pressure"],
                sensor["diff_pressure"],
                sensor["pressure_alt"],
                sensor["temperature"],
                sensor["fields_updated"],
            )

            hil_state = build_hil_state_quaternion_sample(sim_time_us, world_out)
            conn.mav.hil_state_quaternion_send(
                hil_state["time_usec"],
                hil_state["attitude_quaternion"],
                hil_state["rollspeed"],
                hil_state["pitchspeed"],
                hil_state["yawspeed"],
                hil_state["lat"],
                hil_state["lon"],
                hil_state["alt"],
                hil_state["vx"],
                hil_state["vy"],
                hil_state["vz"],
                hil_state["ind_airspeed"],
                hil_state["true_airspeed"],
                hil_state["xacc"],
                hil_state["yacc"],
                hil_state["zacc"],
            )

            if sim_time_us >= next_system_time_us:
                conn.mav.system_time_send(int(time.time() * 1_000_000), int(sim_time_us / 1000))
                next_system_time_us = sim_time_us + SYSTEM_TIME_INTERVAL_US

            if sim_time_us >= next_gps_time_us:
                gps = build_hil_gps_sample(sim_time_us, z)
                conn.mav.hil_gps_send(
                    int(time.time() * 1_000_000),
                    gps["fix_type"],
                    gps["lat"],
                    gps["lon"],
                    gps["alt"],
                    gps["eph"],
                    gps["epv"],
                    gps["vel"],
                    gps["vn"],
                    gps["ve"],
                    gps["vd"],
                    gps["cog"],
                    gps["satellites_visible"],
                    0,
                    0,
                )
                next_gps_time_us = sim_time_us + GPS_INTERVAL_US

            last_time_ran_ms = now_ms

        if sim_time_us >= next_heartbeat_time_us:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            next_heartbeat_time_us = sim_time_us + HEARTBEAT_INTERVAL_US

        slow_down_counter += 1
        time.sleep(1.0 / RATE_HZ)


def main() -> None:
    print("Listening on tcpin:0.0.0.0:4560")
    print("Press Ctrl+C to stop")
    try:
        sim_side()
    except KeyboardInterrupt:
        print("Stopped.")


if __name__ == "__main__":
    main()
