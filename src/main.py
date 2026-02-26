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
from transfer_alignment import (
    TransferAlignmentMasterLink,
    TransferAlignmentSlaveLink,
    quat_from_euler_deg_wxyz,
    transform_master_to_slave_state,
)
from visualizer.websockerPublisher import GroundTruthWebSocketPublisher



RATE_HZ = 250
DT_US = int(1e6 / RATE_HZ)
HEARTBEAT_INTERVAL_US = 1_000_000
WEBSOCKET_INTERVAL_US = 5_000_0
SYSTEM_TIME_INTERVAL_US = 1_000_000
GPS_START_DELAY_US = 1_000_000
MAVLINK_MSG_ID_HIL_STATE_QUATERNION = 115
MAV_CMD_TRANSFER_CUTOVER = int(getattr(mavutil.mavlink, "MAV_CMD_USER_1", 31000))
CHECK_FACTOR = 2

SIM_ROLE = os.getenv("SIM_ROLE", "standalone").strip().lower()
MAVLINK_BIND_HOST = os.getenv("SIM_MAVLINK_BIND_HOST", "0.0.0.0")
MAVLINK_BIND_PORT = int(os.getenv("SIM_MAVLINK_BIND_PORT", "4560"))

TRANSFER_UDP_TARGET_HOST = os.getenv("SIM_TRANSFER_UDP_TARGET_HOST", "127.0.0.1")
TRANSFER_UDP_TARGET_PORT = int(os.getenv("SIM_TRANSFER_UDP_TARGET_PORT", "18000"))
TRANSFER_UDP_BIND_HOST = os.getenv("SIM_TRANSFER_UDP_BIND_HOST", "0.0.0.0")
TRANSFER_UDP_BIND_PORT = int(os.getenv("SIM_TRANSFER_UDP_BIND_PORT", "18000"))
TRANSFER_ARM_M = os.getenv("SIM_TRANSFER_ARM_M", "0.0,0.0,0.0")
TRANSFER_ARM_FRAME = os.getenv("SIM_TRANSFER_ARM_FRAME", "master_body").strip().lower()
TRANSFER_REL_EULER_DEG = os.getenv("SIM_TRANSFER_REL_EULER_DEG", "0.0,0.0,0.0")
TRANSFER_TIMEOUT_S = float(os.getenv("SIM_TRANSFER_TIMEOUT_S", "1.0"))
TRANSFER_CUTOVER_MODE = os.getenv("SIM_TRANSFER_CUTOVER_MODE", "mavlink_cmd").strip().lower()
TRANSFER_CUTOVER_TIME_S = float(os.getenv("SIM_TRANSFER_CUTOVER_TIME_S", "10.0"))

GT_WS_HOST = os.getenv("SIM_GT_WS_HOST", "0.0.0.0")
GT_WS_PORT = int(os.getenv("SIM_GT_WS_PORT", "8765"))
GT_WS_ENABLED = os.getenv("SIM_GT_WS_ENABLED", "auto").strip().lower()
VEHICLE_MODEL = os.getenv("SIM_VEHICLE_MODEL", "ts04").strip().lower()
TS04_PITCH90_START = os.getenv("SIM_TS04_PITCH90_START", "1").strip().lower() in {"1", "true", "yes", "on"}
TS04_MOTOR_MAP = os.getenv("SIM_TS04_MOTOR_MAP", "0,1,2,3")

LOG_FORMAT = "%(asctime)s | %(levelname)s | %(name)s | %(message)s"
logger = logging.getLogger(__name__)
AVAILABLE_VEHICLE_MODELS = ("x8", "iris", "ts04")




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


def parse_env_float(name: str, default: float) -> float:
    raw = os.getenv(name, str(default)).strip()
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a float, got '{raw}'") from exc


def parse_vec3(raw: str, name: str) -> np.ndarray:
    tokens = [tok.strip() for tok in str(raw).split(",") if tok.strip()]
    if len(tokens) != 3:
        raise ValueError(f"{name} must contain 3 comma-separated floats, got '{raw}'")
    try:
        return np.array([float(tokens[0]), float(tokens[1]), float(tokens[2])], dtype=float)
    except ValueError as exc:
        raise ValueError(f"{name} must contain floats, got '{raw}'") from exc


def parse_sim_role(raw: str) -> str:
    role = str(raw).strip().lower()
    if role not in {"standalone", "master", "slave"}:
        raise ValueError(f"SIM_ROLE must be one of standalone|master|slave, got '{raw}'")
    return role


def parse_cutover_mode(raw: str) -> str:
    mode = str(raw).strip().lower()
    if mode not in {"never", "time", "mavlink_cmd"}:
        raise ValueError(f"SIM_TRANSFER_CUTOVER_MODE must be one of never|time|mavlink_cmd, got '{raw}'")
    return mode


def parse_vehicle_model(raw: str) -> str:
    model = str(raw).strip().lower()
    if model not in AVAILABLE_VEHICLE_MODELS:
        choices = "|".join(AVAILABLE_VEHICLE_MODELS)
        raise ValueError(f"SIM_VEHICLE_MODEL must be one of {choices}, got '{raw}'")
    return model


def parse_arm_frame(raw: str) -> str:
    value = str(raw).strip().lower()
    if value not in {"world_ned", "master_body"}:
        raise ValueError(f"SIM_TRANSFER_ARM_FRAME must be one of world_ned|master_body, got '{raw}'")
    return value


def parse_gt_ws_enabled(role: str, raw: str) -> bool:
    value = str(raw).strip().lower()
    if value == "auto":
        return role != "slave"
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"SIM_GT_WS_ENABLED must be auto|true|false, got '{raw}'")


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


def compute_aero_angles_deg(y: np.ndarray, wind: np.ndarray) -> tuple[float | None, float | None]:
    vel_rel = np.asarray(y[7:10], dtype=float) - np.asarray(wind[:3], dtype=float)
    u_r, v_r, w_r = vel_rel
    Va = float(np.linalg.norm(vel_rel))
    if Va <= 1e-5:
        return None, None

    alpha_deg = float(np.rad2deg(np.arctan2(w_r, u_r)))
    beta_deg = float(np.rad2deg(np.arcsin(np.clip(v_r / Va, -1.0, 1.0))))
    return alpha_deg, beta_deg


def simulation_main() -> None:
    role = parse_sim_role(SIM_ROLE)
    cutover_mode = parse_cutover_mode(TRANSFER_CUTOVER_MODE)
    transfer_arm_frame = parse_arm_frame(TRANSFER_ARM_FRAME)
    gt_ws_enabled = parse_gt_ws_enabled(role, GT_WS_ENABLED)
    vehicle_model = parse_vehicle_model(VEHICLE_MODEL)

    mavlink_endpoint = f"tcpin:{MAVLINK_BIND_HOST}:{MAVLINK_BIND_PORT}"
    conn: Any = mavutil.mavlink_connection(mavlink_endpoint, source_component=51)

    logger.info("Waiting for Heartbeat ...")
    logger.info("Running in role: %s", role)
    logger.info("Using vehicle model: %s", vehicle_model)
    gps_origin = {
        "lat": parse_env_float("SIM_GPS_ORIGIN_LAT", 47.397742),
        "lon": parse_env_float("SIM_GPS_ORIGIN_LON", 8.545594),
        "alt": parse_env_float("SIM_GPS_ORIGIN_ALT", 470.0),
    }
    logger.info(
        "Using GPS origin: lat=%.6f lon=%.6f alt=%.2f",
        gps_origin["lat"],
        gps_origin["lon"],
        gps_origin["alt"],
    )
    ts04_motor_map = parse_motor_map(TS04_MOTOR_MAP)
    if vehicle_model == "ts04":
        logger.info("TS04 motor map (sim motor idx -> HIL control idx): %s", ts04_motor_map)
    first_hb = conn.wait_heartbeat()
    px4_sysid = first_hb.get_srcSystem()
    if px4_sysid > 0:
        conn.source_system = px4_sysid
        conn.mav.srcSystem = px4_sysid
        logger.info("Locked simulator SYSID to PX4 SYSID=%s", px4_sysid)

    y0 = np.zeros(13)
    y0[0:3] = np.array([0.0, 0.0, -3.0])
    if vehicle_model == "ts04" and TS04_PITCH90_START:
        y0[3:7] = np.array([np.sqrt(0.5), 0.0, np.sqrt(0.5), 0.0])
    else:
        y0[3] = 1.0

    world = World(
        vehicle_model=vehicle_model,
        y0=y0,
        u0=np.zeros(4),
        wind0=np.zeros(6),
        ts04_pitch90_start=TS04_PITCH90_START,
    )
    world.P.gps_origin = dict(gps_origin)
    gt_ws = GroundTruthWebSocketPublisher(host=GT_WS_HOST, port=GT_WS_PORT, enabled=gt_ws_enabled)
    gt_ws.start()
    if gt_ws_enabled:
        logger.info("Ground-truth WS target: ws://%s:%s", GT_WS_HOST, GT_WS_PORT)
    else:
        logger.info("Ground-truth WS disabled")

    transfer_master_link: TransferAlignmentMasterLink | None = None
    transfer_slave_link: TransferAlignmentSlaveLink | None = None

    slave_arm_master_body_m = np.zeros(3)
    slave_q_from_master = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    if role == "master":
        transfer_master_link = TransferAlignmentMasterLink(
            target_host=TRANSFER_UDP_TARGET_HOST,
            target_port=TRANSFER_UDP_TARGET_PORT,
        )
        logger.info(
            "Master transfer stream enabled: udp://%s:%s",
            TRANSFER_UDP_TARGET_HOST,
            TRANSFER_UDP_TARGET_PORT,
        )
    elif role == "slave":
        transfer_slave_link = TransferAlignmentSlaveLink(
            bind_host=TRANSFER_UDP_BIND_HOST,
            bind_port=TRANSFER_UDP_BIND_PORT,
            timeout_s=TRANSFER_TIMEOUT_S,
        )
        slave_arm_master_body_m = parse_vec3(TRANSFER_ARM_M, "SIM_TRANSFER_ARM_M")
        rel_euler = parse_vec3(TRANSFER_REL_EULER_DEG, "SIM_TRANSFER_REL_EULER_DEG")
        slave_q_from_master = quat_from_euler_deg_wxyz(rel_euler[0], rel_euler[1], rel_euler[2])
        logger.info(
            "Slave transfer input bound on udp://%s:%s",
            TRANSFER_UDP_BIND_HOST,
            TRANSFER_UDP_BIND_PORT,
        )
        logger.info(
            "Slave transfer params: arm[m]=%s arm_frame=%s rel_euler[deg]=%s cutover_mode=%s",
            slave_arm_master_body_m.tolist(),
            transfer_arm_frame,
            rel_euler.tolist(),
            cutover_mode,
        )

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
    next_transfer_warn_wall_s = last_rx_wall_s + 2.0

    slave_coupled = role == "slave"
    cutover_requested = False
    cutover_time_us = int(max(0.0, TRANSFER_CUTOVER_TIME_S) * 1e6)
    last_slave_transformed_state: tuple[np.ndarray, np.ndarray] | None = None
    last_transfer_seq = -1

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
                    elif (
                        role == "slave"
                        and slave_coupled
                        and cutover_mode == "mavlink_cmd"
                        and command == MAV_CMD_TRANSFER_CUTOVER
                    ):
                        cutover_requested = True
                        logger.info("SIM <= transfer cutover command received")

            if armed and (not was_armed):
                logger.info("SIM => ARM transition: dynamics enabled")
            if (not armed) and was_armed:
                logger.info("SIM => DISARM transition: dynamics remain enabled")
            was_armed = armed

            now_wall_s = time.time()
            if now_wall_s >= next_rx_warn_wall_s and (now_wall_s - last_rx_wall_s) > 2.0:
                logger.warning("SIM: no MAVLink RX for >2s")
                next_rx_warn_wall_s = now_wall_s + 2.0

            world.set_controls(
                controls_to_u(
                    latest_controls,
                    armed,
                    ts04_motor_map=ts04_motor_map,
                    vehicle_model=vehicle_model,
                )
            )

            world_out = None
            needs_to_pause = False
            now_ms = get_sim_millis(sim_time_us)

            if slave_coupled:
                packet = transfer_slave_link.poll_latest() if transfer_slave_link is not None else None

                if packet is None or packet.seq == last_transfer_seq:
                    if transfer_slave_link is not None and transfer_slave_link.timed_out() and now_wall_s >= next_transfer_warn_wall_s:
                        logger.warning("SIM: no master transfer packet for >%.1fs", TRANSFER_TIMEOUT_S)
                        next_transfer_warn_wall_s = now_wall_s + 2.0
                    slow_down_counter += 1
                    time.sleep(1.0 / RATE_HZ)
                    continue

                last_transfer_seq = int(packet.seq)
                sim_time_us = int(packet.time_us)
                now_ms = get_sim_millis(sim_time_us)

                y_slave, ydot_slave = transform_master_to_slave_state(
                    y_master=packet.y,
                    ydot_master=packet.ydot,
                    arm_m=slave_arm_master_body_m,
                    q_slave_from_master_wxyz=slave_q_from_master,
                    arm_frame=transfer_arm_frame,
                )
                world_out = world.observe_external_state(sim_time_us, y_slave, ydot_slave)
                last_slave_transformed_state = (y_slave.copy(), ydot_slave.copy())

                if cutover_mode == "time" and sim_time_us >= cutover_time_us:
                    cutover_requested = True

                if cutover_requested and last_slave_transformed_state is not None and cutover_mode != "never":
                    world.set_state(last_slave_transformed_state[0])
                    world.sync_time(sim_time_us)
                    slave_coupled = False
                    logger.info("SIM => transfer cutover at t=%.3fs, switching to local dynamics", sim_time_us / 1e6)

            else:
                sim_time_us += DT_US
                io_run_only = (slow_down_counter % CHECK_FACTOR) != 0
                now_ms = get_sim_millis(sim_time_us)
                needs_to_pause = (last_time_ran_ms == now_ms) or io_run_only
                world_out = world.update(sim_time_us, needs_to_pause, freeze_dynamics=(not ever_armed))

                if role == "master" and (not needs_to_pause) and world_out is not None and transfer_master_link is not None:
                    transfer_master_link.send(sim_time_us, world_out["y"], world_out["ydot"])

            should_publish = world_out is not None and ((not needs_to_pause) or slave_coupled)

            if should_publish:
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
                    alpha_deg, beta_deg = compute_aero_angles_deg(y, world.wind)
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
                            "aero": {
                                "alpha_deg": alpha_deg,
                                "beta_deg": beta_deg,
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
        if transfer_master_link is not None:
            transfer_master_link.close()
        if transfer_slave_link is not None:
            transfer_slave_link.close()
        gt_ws.stop()


def main() -> None:
    logging.basicConfig(
        level=getattr(logging, os.getenv("SIM_LOG_LEVEL", "INFO").upper(), logging.INFO),
        format=LOG_FORMAT,
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    logger.info("Listening on tcpin:%s:%s", MAVLINK_BIND_HOST, MAVLINK_BIND_PORT)
    logger.info("Press Ctrl+C to stop")
    try:
        simulation_main()
    except KeyboardInterrupt:
        logger.info("Stopped.")


if __name__ == "__main__":
    main()
