#!/usr/bin/env python3
"""MAVLink lockstep simulation endpoint backed by vehicle.World."""

import os
import sys
import time
import logging
from typing import Any

import numpy as np
from pymavlink import mavutil

vehicle_dir = os.path.join(os.path.dirname(__file__), "vehicle")
if vehicle_dir not in sys.path:
    sys.path.insert(0, vehicle_dir)

from world import World
from visualizer.websockerPublisher import GroundTruthWebSocketPublisher



RATE_HZ = 250
DT_US = int(1e6 / RATE_HZ)
HEARTBEAT_INTERVAL_US = 1_000_000
WEBSOCKET_INTERVAL_US = 5_000_0
SYSTEM_TIME_INTERVAL_US = 1_000_000
GPS_START_DELAY_US = 1_000_000
MAVLINK_MSG_ID_HIL_STATE_QUATERNION = 115
CHECK_FACTOR = 2

GT_WS_HOST = os.getenv("SIM_GT_WS_HOST", "0.0.0.0")
GT_WS_PORT = int(os.getenv("SIM_GT_WS_PORT", "8765"))
VEHICLE_MODEL = os.getenv("SIM_VEHICLE_MODEL", "ts04").strip().lower()
TS04_PITCH90_START = os.getenv("SIM_TS04_PITCH90_START", "1").strip().lower() in {"1", "true", "yes", "on"}
TS04_MOTOR_MAP = os.getenv("SIM_TS04_MOTOR_MAP", "0,1,2,3")

LOG_FORMAT = "%(asctime)s | %(levelname)s | %(name)s | %(message)s"
logger = logging.getLogger(__name__)




def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def get_sim_millis(sim_time_us: int) -> int:
    return sim_time_us // 1000


def parse_motor_map(raw: str) -> tuple[int, int, int, int]:
    tokens = [tok.strip() for tok in raw.split(",") if tok.strip()]
    if len(tokens) != 4:
        raise ValueError(f"Expected 4 entries in SIM_TS04_MOTOR_MAP, got '{raw}'")
    try:
        values = tuple(int(tok) for tok in tokens)
    except ValueError as exc:
        raise ValueError(f"SIM_TS04_MOTOR_MAP must contain integers, got '{raw}'") from exc
    if sorted(values) != [0, 1, 2, 3]:
        raise ValueError(f"SIM_TS04_MOTOR_MAP must be a permutation of 0,1,2,3, got '{raw}'")
    return (values[0], values[1], values[2], values[3])


def controls_to_u(
    latest_controls: tuple[float, ...] | None,
    armed: bool,
    ts04_motor_map: tuple[int, int, int, int] = (0, 1, 2, 3),
    vehicle_model: str = "x8",
) -> np.ndarray:
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

    if vehicle_model == "ts04":
        u = u[list(ts04_motor_map)]

    return u


def controls_to_u8(latest_controls: tuple[float, ...] | None) -> list[float]:
    if latest_controls is None:
        return [0.0] * 8
    out = [0.0] * 8
    n = min(len(latest_controls), 8)
    for i in range(n):
        out[i] = float(latest_controls[i])
    return out


def simulation_main() -> None:
    conn: Any = mavutil.mavlink_connection("tcpin:0.0.0.0:4560", source_component=51)

    logger.info("Waiting for Heartbeat ...")
    ts04_motor_map = parse_motor_map(TS04_MOTOR_MAP)
    if VEHICLE_MODEL == "ts04":
        logger.info("TS04 motor map (sim motor idx -> HIL control idx): %s", ts04_motor_map)
    first_hb = conn.wait_heartbeat()
    px4_sysid = first_hb.get_srcSystem()
    if px4_sysid > 0:
        conn.source_system = px4_sysid
        conn.mav.srcSystem = px4_sysid
        logger.info("Locked simulator SYSID to PX4 SYSID=%s", px4_sysid)

    y0 = np.zeros(13)
    y0[0:3] = np.array([0.0, 0.0, -3.0])
    if VEHICLE_MODEL == "ts04" and TS04_PITCH90_START:
        y0[3:7] = np.array([np.sqrt(0.5), 0.0, np.sqrt(0.5), 0.0])
    else:
        y0[3] = 1.0

    world = World(
        vehicle_model=VEHICLE_MODEL,
        y0=y0,
        u0=np.zeros(4),
        wind0=np.zeros(6),
        ts04_pitch90_start=TS04_PITCH90_START,
    )
    gt_ws = GroundTruthWebSocketPublisher(host=GT_WS_HOST, port=GT_WS_PORT)
    gt_ws.start()

    sim_time_us = 0
    next_heartbeat_time_us = 0
    next_websocket_time_us = 0
    next_system_time_us = 0
    gps_start_time_us = GPS_START_DELAY_US
    hil_state_interval_us = -1
    next_hil_state_time_us = 0
    last_time_ran_ms = 0
    slow_down_counter = 0
    latest_controls: tuple[float, ...] | None = None
    armed = False
    was_armed = False
    ever_armed = False
    last_rx_wall_s = time.time()
    next_rx_warn_wall_s = last_rx_wall_s + 2.0

    logger.info("PX4 connected starting sim")
    try:
        while True:
            while True:
                msg = conn.recv_match(blocking=False)
                if msg is None:
                    break

                msg_type = msg.get_type()
                last_rx_wall_s = time.time()

                if msg_type == "HEARTBEAT":
                    logger.info("SIM <= HEARTBEAT")

                elif msg_type == "HIL_ACTUATOR_CONTROLS":
                    controls = getattr(msg, "controls", None)
                    latest_controls = tuple(float(v) for v in controls) if isinstance(controls, (list, tuple)) else None

                    mode = int(getattr(msg, "mode", 0))
                    if mode != 0:
                        armed = (mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        if armed:
                            ever_armed = True

                elif msg_type == "COMMAND_LONG":
                    command = int(getattr(msg, "command", 0))
                    if command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                        msg_id = int(float(getattr(msg, "param1", 0.0)) + 0.5)
                        interval_us = int(float(getattr(msg, "param2", -1.0)) + 0.5)
                        if msg_id == MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
                            hil_state_interval_us = interval_us if interval_us > 0 else -1
                            next_hil_state_time_us = sim_time_us
                            if hil_state_interval_us > 0:
                                logger.info("SIM <= set HIL_STATE_QUAT interval to %s us", hil_state_interval_us)
                            else:
                                logger.info("SIM <= disable HIL_STATE_QUAT")

            if armed and (not was_armed):
                logger.info("SIM => ARM transition: dynamics enabled")
            if (not armed) and was_armed:
                logger.info("SIM => DISARM transition: dynamics remain enabled")
            was_armed = armed

            now_wall_s = time.time()
            if now_wall_s >= next_rx_warn_wall_s and (now_wall_s - last_rx_wall_s) > 2.0:
                logger.warning("SIM: no MAVLink RX for >2s")
                next_rx_warn_wall_s = now_wall_s + 2.0

            sim_time_us += DT_US
            io_run_only = (slow_down_counter % CHECK_FACTOR) != 0
            now_ms = get_sim_millis(sim_time_us)
            needs_to_pause = (last_time_ran_ms == now_ms) or io_run_only

            world.set_controls(
                controls_to_u(
                    latest_controls,
                    armed,
                    ts04_motor_map=ts04_motor_map,
                    vehicle_model=VEHICLE_MODEL,
                )
            )
            world_out = world.update(sim_time_us, needs_to_pause, freeze_dynamics=(not ever_armed))

            if (not needs_to_pause) and world_out is not None:
                z = world_out["sensors"]
                y = np.asarray(world_out["y"], dtype=float)
                gps = np.asarray(z["gps"], dtype=float)
                acc = np.asarray(z["accelerometer"], dtype=float)
                gyro = np.asarray(z["gyroscope"], dtype=float)
                mag = np.asarray(z["magnetometer"], dtype=float)
                baro = z["barometer"]

                conn.mav.hil_sensor_send(
                    int(sim_time_us),
                    float(acc[0]),
                    float(acc[1]),
                    float(acc[2]),
                    float(gyro[0]),
                    float(gyro[1]),
                    float(gyro[2]),
                    float(mag[0]),
                    float(mag[1]),
                    float(mag[2]),
                    float(baro["staticAbsolute"]) * 0.01,
                    float(baro["dynamic"]) * 0.01,
                    float(baro.get("pressure_altitude_m", gps[2])),
                    15.0,
                    8191,
                    )

                if hil_state_interval_us > 0 and sim_time_us >= next_hil_state_time_us:
                    vel_north = float(gps[3])
                    vel_east = float(gps[4])
                    vel_down = float(-gps[5])
                    horiz_speed_m_s = float(np.hypot(vel_north, vel_east))
                    m_s2_to_mg = 1000.0 / 9.80665
                    conn.mav.hil_state_quaternion_send(
                        int(sim_time_us),
                        [float(v) for v in y[3:7]],
                        float(y[10]),
                        float(y[11]),
                        float(y[12]),
                        int(round(float(gps[0]) * 1e7)),
                        int(round(float(gps[1]) * 1e7)),
                        int(round(float(gps[2]) * 1000.0)),
                        int(round(vel_north * 100.0)),
                        int(round(vel_east * 100.0)),
                        int(round(vel_down * 100.0)),
                        int(round(horiz_speed_m_s * 100.0)),
                        int(round(horiz_speed_m_s * 100.0)),
                        int(round(float(acc[0]) * m_s2_to_mg)),
                        int(round(float(acc[1]) * m_s2_to_mg)),
                        int(round(float(acc[2]) * m_s2_to_mg)),
                    )
                    next_hil_state_time_us = sim_time_us + hil_state_interval_us

                if sim_time_us >= next_system_time_us:
                    conn.mav.system_time_send(int(time.time() * 1_000_000), int(sim_time_us / 1000))
                    next_system_time_us = sim_time_us + SYSTEM_TIME_INTERVAL_US

                if sim_time_us >= gps_start_time_us and bool(z.get("gps_updated", False)):
                    vel_north = float(gps[3])
                    vel_east = float(gps[4])
                    vel_down = float(-gps[5])
                    vel_3d = float(np.linalg.norm(np.array([vel_north, vel_east, vel_down])))
                    cog_rad = float(np.arctan2(vel_east, vel_north))
                    if cog_rad < 0.0:
                        cog_rad += 2.0 * np.pi

                    conn.mav.hil_gps_send(
                        int(sim_time_us),
                        3,
                        int(round(float(gps[0]) * 1e7)),
                        int(round(float(gps[1]) * 1e7)),
                        int(round(float(gps[2]) * 1000.0)),
                        100,
                        100,
                        int(round(vel_3d * 100.0)),
                        int(round(vel_north * 100.0)),
                        int(round(vel_east * 100.0)),
                        int(round(vel_down * 100.0)),
                        int(round(np.degrees(cog_rad) * 100.0)),
                        10,
                        0,
                        0,
                    )

                if sim_time_us >= next_websocket_time_us:
                    gt_ws.publish(
                        {
                            "system_id": int(px4_sysid),
                            "time_usec": int(sim_time_us),
                            "u": controls_to_u8(latest_controls),
                            "position_ned_m": [float(y[0]), float(y[1]), float(y[2])],
                            "quaternion_wxyz": [float(y[3]), float(y[4]), float(y[5]), float(y[6])],
                            "velocity_body_mps": [float(y[7]), float(y[8]), float(y[9])],
                            "angular_rate_body_rps": [float(y[10]), float(y[11]), float(y[12])],
                            "lla": {
                                "lat_deg": float(gps[0]),
                                "lon_deg": float(gps[1]),
                                "alt_m": float(gps[2]),
                            },
                        }
                    )
                    next_websocket_time_us = sim_time_us + WEBSOCKET_INTERVAL_US


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
    finally:
        gt_ws.stop()


def main() -> None:
    logging.basicConfig(
        level=getattr(logging, os.getenv("SIM_LOG_LEVEL", "INFO").upper(), logging.INFO),
        format=LOG_FORMAT,
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger.info("Listening on tcpin:0.0.0.0:4560")
    logger.info("Press Ctrl+C to stop")
    try:
        simulation_main()
    except KeyboardInterrupt:
        logger.info("Stopped.")


if __name__ == "__main__":
    main()
