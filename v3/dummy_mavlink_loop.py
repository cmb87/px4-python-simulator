#!/usr/bin/env python3
"""Very small MAVLink dummy simulation endpoint.

Mirrors the Java lockstep/checkFactor behavior at a high level:
- IO runs every loop
- every second loop can be IO-only (no sim publish)
- simulated time advances on actuator controls, with fallback when needed
"""

import time
import math
from random import random
from typing import Any

from pymavlink import mavutil


RATE_HZ = 20
DT_US = int(1e6 / RATE_HZ)
SYSTEM_TIME_INTERVAL_US = 1_000_000
GPS_INTERVAL_US = 52_000

LOCKSTEP_ENABLED = True
DISPLAY_ONLY = False
CHECK_FACTOR = 2


def crandom() -> float:
    return random() - 0.5


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (w, x, y, z)


def build_initial_world_state() -> dict[str, Any]:
    return {
        "lat_deg": 51.2538,
        "lon_deg": 6.7802,
        "alt_m": 488.0,
        "vx_m_s": 0.0,
        "vy_m_s": 0.0,
        "vz_m_s": 0.0,
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": 0.0,
        "rollspeed_rad_s": 0.0,
        "pitchspeed_rad_s": 0.0,
        "yawspeed_rad_s": 0.0,
        "xacc_m_s2": 0.0,
        "yacc_m_s2": 0.0,
        "zacc_m_s2": -9.81,
        "attitude_quaternion": (1.0, 0.0, 0.0, 0.0),
    }


def step_physical_world(
    world: dict[str, Any],
    controls: tuple[float, ...] | None,
    dt_s: float,
) -> None:
    throttle = 0.0
    if controls is not None and len(controls) > 3:
        throttle = clamp(float(controls[3]), 0.0, 1.0)

    target_vx_m_s = throttle * 22.0
    accel_forward_m_s2 = (target_vx_m_s - float(world["vx_m_s"])) * 1.5
    world["vx_m_s"] = float(world["vx_m_s"]) + accel_forward_m_s2 * dt_s

    target_vz_m_s = (0.45 - throttle) * 3.0
    world["vz_m_s"] = float(world["vz_m_s"]) + (target_vz_m_s - float(world["vz_m_s"])) * 1.0 * dt_s
    world["alt_m"] = max(0.0, float(world["alt_m"]) - float(world["vz_m_s"]) * dt_s)

    time_like = float(world["yaw_rad"])
    target_roll_rad = clamp((throttle - 0.5) * 0.25 + math.sin(time_like * 0.3) * 0.05, -0.45, 0.45)
    target_pitch_rad = clamp((0.5 - throttle) * 0.12 + math.cos(time_like * 0.2) * 0.03, -0.25, 0.25)
    world["rollspeed_rad_s"] = (target_roll_rad - float(world["roll_rad"])) / max(dt_s, 1e-6)
    world["pitchspeed_rad_s"] = (target_pitch_rad - float(world["pitch_rad"])) / max(dt_s, 1e-6)
    world["roll_rad"] = float(world["roll_rad"]) + (target_roll_rad - float(world["roll_rad"])) * 0.25
    world["pitch_rad"] = float(world["pitch_rad"]) + (target_pitch_rad - float(world["pitch_rad"])) * 0.2

    yaw_rate = (throttle - 0.5) * 0.15 + crandom() * 0.01
    world["yawspeed_rad_s"] = yaw_rate
    world["yaw_rad"] = float(world["yaw_rad"]) + yaw_rate * dt_s

    north_m = float(world["vx_m_s"]) * dt_s
    east_m = float(world["vy_m_s"]) * dt_s
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = max(1.0, 111_320.0 * math.cos(math.radians(float(world["lat_deg"]))))
    world["lat_deg"] = float(world["lat_deg"]) + north_m / meters_per_deg_lat
    world["lon_deg"] = float(world["lon_deg"]) + east_m / meters_per_deg_lon

    world["xacc_m_s2"] = accel_forward_m_s2 + crandom() * 0.05
    world["yacc_m_s2"] = crandom() * 0.05
    world["zacc_m_s2"] = -9.81 + crandom() * 0.05
    world["attitude_quaternion"] = euler_to_quaternion(
        float(world["roll_rad"]),
        float(world["pitch_rad"]),
        float(world["yaw_rad"]),
    )


def build_hil_state_quaternion_sample(
    sim_time_us: int,
    world: dict[str, Any],
) -> dict[str, Any]:
    horiz_speed_m_s = math.hypot(float(world["vx_m_s"]), float(world["vy_m_s"]))
    m_s2_to_mg = 1000.0 / 9.80665
    return {
        "time_usec": sim_time_us,
        "attitude_quaternion": [float(v) for v in world["attitude_quaternion"]],
        "rollspeed": float(world["rollspeed_rad_s"]),
        "pitchspeed": float(world["pitchspeed_rad_s"]),
        "yawspeed": float(world["yawspeed_rad_s"]),
        "lat": int(round(float(world["lat_deg"]) * 1e7)),
        "lon": int(round(float(world["lon_deg"]) * 1e7)),
        "alt": int(round(float(world["alt_m"]) * 1000.0)),
        "vx": int(round(float(world["vx_m_s"]) * 100.0)),
        "vy": int(round(float(world["vy_m_s"]) * 100.0)),
        "vz": int(round(float(world["vz_m_s"]) * 100.0)),
        "ind_airspeed": int(round(horiz_speed_m_s * 100.0)),
        "true_airspeed": int(round((horiz_speed_m_s + 0.2) * 100.0)),
        "xacc": int(round(float(world["xacc_m_s2"]) * m_s2_to_mg)),
        "yacc": int(round(float(world["yacc_m_s2"]) * m_s2_to_mg)),
        "zacc": int(round(float(world["zacc_m_s2"]) * m_s2_to_mg)),
    }


def build_sensor_sample() -> dict[str, float | int]:
    return {
        "xacc": float((0 + crandom() * 0.2) * 1),
        "yacc": float((0 + crandom() * 0.2) * 1),
        "zacc": float((-9.81 + crandom() * 0.2) * 1),
        "xgyro": float((0 + crandom() * 0.04) * 1),
        "ygyro": float((0 + crandom() * 0.04) * 1),
        "zgyro": float((0 + crandom() * 0.04) * 1),
        "xmag": float((0.215 + crandom() * 0.02) * 1),
        "ymag": float((0.01 + crandom() * 0.02) * 1),
        "zmag": float((0.43 + crandom() * 0.02) * 1),
        "abs_pressure": float((95598 + crandom() * 4) * 0.01),
        "diff_pressure": float((0 + crandom() * 0) * 0.01),
        "pressure_alt": float((488 + crandom() * 0.5) * 1),
        "temperature": float((0 + crandom() * 0) * 1),
        "fields_updated": 7167,
    }


def build_gps_sample() -> dict[str, int]:
    return {
        "lat": round((51.2538 + crandom() * 5e-7) * 1e7),
        "lon": round((6.7802 + crandom() * 5e-7) * 1e7),
        "alt": round((488 + crandom() * 0.05) * 1000),
        "eph": round((0.3 + random() * 0.001) * 100),
        "epv": round((0.4 + random() * 0.001) * 100),
        "vel": round((0 + random() * 0.001) * 100),
        "vn": round((0 + random() * 0.001) * 100),
        "ve": round((0 + random() * 0.001) * 100),
        "vd": round((0 + random() * 0.001) * 100),
        "cog": round((0 + crandom() * 0.001) * 100),
    }


def advance_time(sim_time_us: int) -> int:
    if LOCKSTEP_ENABLED:
        return sim_time_us + DT_US
    return sim_time_us


def get_sim_millis(sim_time_us: int) -> int:
    if LOCKSTEP_ENABLED:
        return sim_time_us // 1000
    return int(time.time() * 1000)


def sim_side() -> None:
    conn: Any = mavutil.mavlink_connection("tcpin:0.0.0.0:4560", source_system=51, source_component=51)
    sim_time_us = 0
    next_system_time_us = 0
    next_gps_time_us = 0
    last_time_ran_ms = 0
    slow_down_counter = 0
    got_hil_actuator_controls = False
    latest_controls: tuple[float, ...] | None = None
    world = build_initial_world_state()

    while True:
        hb = conn.recv_match(type="HEARTBEAT", blocking=False)
        if hb:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(f"SIM <= HEARTBEAT armed={armed}")

        ctrl = conn.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=False)
        if ctrl:
            got_hil_actuator_controls = True
            controls = getattr(ctrl, "controls", None)
            if isinstance(controls, (list, tuple)):
                latest_controls = tuple(float(v) for v in controls)
            else:
                latest_controls = None
            sim_time_us = advance_time(sim_time_us)
            step_physical_world(world, latest_controls, DT_US / 1e6)
            if isinstance(controls, (list, tuple)) and len(controls) > 3:
                print(f"SIM <= HIL_ACTUATOR_CONTROLS throttle={controls[3]:.2f} t={sim_time_us}")
            else:
                print(f"SIM <= HIL_ACTUATOR_CONTROLS t={sim_time_us}")

        needs_to_pause = False
        if LOCKSTEP_ENABLED and not DISPLAY_ONLY:
            io_run_only = (slow_down_counter % CHECK_FACTOR) != 0

            if (not got_hil_actuator_controls) and (not io_run_only):
                sim_time_us = advance_time(sim_time_us)
                step_physical_world(world, latest_controls, DT_US / 1e6)
                print("advancing without hilActuators")

            now_ms = get_sim_millis(sim_time_us)
            needs_to_pause = (last_time_ran_ms == now_ms) or io_run_only

            if needs_to_pause:
                print("NeedsToPause True")
        else:
            now_ms = get_sim_millis(sim_time_us)

        if not needs_to_pause:
            sensor = build_sensor_sample()
            conn.mav.hil_sensor_send(
                sim_time_us,
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

            hil_state = build_hil_state_quaternion_sample(sim_time_us, world)
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
                conn.mav.system_time_send(
                    int(time.time() * 1_000_000),
                    int(sim_time_us / 1000),
                )
                next_system_time_us = sim_time_us + SYSTEM_TIME_INTERVAL_US

            if sim_time_us >= next_gps_time_us:
                gps = build_gps_sample()
                conn.mav.hil_gps_send(
                    int(time.time() * 1_000_000),
                    3,
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
                    10,
                    0,
                    0,
                )
                next_gps_time_us = sim_time_us + GPS_INTERVAL_US

            last_time_ran_ms = now_ms

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
