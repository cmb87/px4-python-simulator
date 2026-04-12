#!/usr/bin/env python3
"""MAVLink lockstep simulation endpoint backed by vehicle.World."""

import os
import time
import logging
import math
from typing import Any

import numpy as np
from pymavlink import mavutil

from vehicle.sim_utils import (
    compute_aero_angles_deg,
    controls_to_u,
    get_sim_millis,
    parse_arm_frame,
    parse_cutover_mode,
    parse_env_float,
    parse_gt_output_mode,
    parse_positive_float,
    parse_sim_role,
    parse_vec3,
    parse_vehicle_model,
)
from vehicle.quaternion import Quaternion
from vehicle.world import World
from vehicle.transfer_alignment import (
    TransferAlignmentMasterLink,
    TransferAlignmentSlaveLink,
    quat_from_euler_deg_wxyz,
    transform_master_to_slave_state,
)
from vehicle.vehicle_catalog import list_vehicle_models
from visualizer.websockerPublisher import GroundTruthWebSocketPublisher
from visualizer.flightgearUdpPublisher import FlightGearUdpPublisher



RATE_HZ = 250
DT_US = int(1e6 / RATE_HZ)
HEARTBEAT_INTERVAL_US = 1_000_000
SYSTEM_TIME_INTERVAL_US = 1_000_000
GPS_START_DELAY_US = 1_000_000
MAVLINK_MSG_ID_HIL_STATE_QUATERNION = 115
MAV_CMD_TRANSFER_CUTOVER = int(getattr(mavutil.mavlink, "MAV_CMD_USER_1", 31000))
HIL_SENSOR_UPDATED_DIFF_PRESSURE_BIT = 1 << 10
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
GT_OUTPUT_MODE = os.getenv("SIM_GT_OUTPUT_MODE", "websocket").strip().lower()
GT_OUTPUT_RATE_HZ_RAW = os.getenv("SIM_GT_OUTPUT_RATE_HZ", "30.0")
FG_UDP_HOST = os.getenv("SIM_FG_UDP_HOST", "127.0.0.1")
FG_UDP_PORT = int(os.getenv("SIM_FG_UDP_PORT", "5503"))
VEHICLE_MODEL = os.getenv("SIM_VEHICLE_MODEL", "ts04").strip().lower()
TS04_PITCH90_START = os.getenv("SIM_TS04_PITCH90_START", "1").strip().lower() in {"1", "true", "yes", "on"}

LOG_FORMAT = "%(asctime)s | %(levelname)s | %(name)s | %(message)s"
logger = logging.getLogger(__name__)


def ned_to_lla_from_origin(pos_ned_m: np.ndarray, lat_home_deg: float, lon_home_deg: float, alt_home_m: float) -> tuple[float, float, float]:
    earth_radius_m = 6371000.0
    x_rad = float(pos_ned_m[0]) / earth_radius_m
    y_rad = float(pos_ned_m[1]) / earth_radius_m
    c = math.sqrt(x_rad * x_rad + y_rad * y_rad)
    lat_home = math.radians(lat_home_deg)
    lon_home = math.radians(lon_home_deg)

    if c > 0.0:
        sin_c = math.sin(c)
        cos_c = math.cos(c)
        sin_lat0 = math.sin(lat_home)
        cos_lat0 = math.cos(lat_home)
        lat = math.asin(cos_c * sin_lat0 + (x_rad * sin_c * cos_lat0) / c)
        lon = lon_home + math.atan2(y_rad * sin_c, c * cos_lat0 * cos_c - x_rad * sin_lat0 * sin_c)
    else:
        lat = lat_home
        lon = lon_home

    alt_m = float(alt_home_m) - float(pos_ned_m[2])
    return math.degrees(lat), math.degrees(lon), alt_m


def simulation_main() -> None:
    role = parse_sim_role(SIM_ROLE)
    cutover_mode = parse_cutover_mode(TRANSFER_CUTOVER_MODE)
    transfer_arm_frame = parse_arm_frame(TRANSFER_ARM_FRAME)
    gt_output_mode = parse_gt_output_mode(GT_OUTPUT_MODE)
    gt_output_rate_hz = parse_positive_float(GT_OUTPUT_RATE_HZ_RAW, "SIM_GT_OUTPUT_RATE_HZ")
    gt_output_interval_us = max(1, int(1e6 / gt_output_rate_hz))
    available_vehicle_models = list_vehicle_models()
    vehicle_model = parse_vehicle_model(VEHICLE_MODEL, available_vehicle_models)

    mavlink_endpoint = f"tcpin:{MAVLINK_BIND_HOST}:{MAVLINK_BIND_PORT}"
    conn: Any = mavutil.mavlink_connection(mavlink_endpoint, source_component=51)

    logger.info("Waiting for Heartbeat ...")
    logger.info("Running in role: %s", role)
    logger.info("Using vehicle model: %s", vehicle_model)
    first_hb = conn.wait_heartbeat()
    px4_sysid = first_hb.get_srcSystem()
    if px4_sysid > 0:
        conn.source_system = px4_sysid
        conn.mav.srcSystem = px4_sysid
        logger.info("Detected PX4 SYSID=%s", px4_sysid)
        logger.info("MAVLink simulator source set to SYSID=%s COMPID=%s", conn.source_system, conn.source_component)

    world = World(
        vehicle_model=vehicle_model,
        u0=np.zeros(4),
        wind0=np.zeros(6),
        ts04_pitch90_start=TS04_PITCH90_START,
    )
    catapult_enabled = bool(getattr(world, "rail_launch_enabled", False))
    catapult_countdown_s = max(0.0, parse_env_float("SIM_CATAPULT_LAUNCH_COUNTDOWN_S", 3.0))
    catapult_countdown_us = int(catapult_countdown_s * 1e6)
    catapult_countdown_active = False
    catapult_release_time_us: int | None = None
    next_countdown_announce_us = 0
    if catapult_enabled:
        logger.info("Catapult launch enabled with countdown %.1f s", catapult_countdown_s)

    gps_origin = getattr(world.P, "gps_origin", {})
    origin_lat_deg = float(gps_origin.get("lat", 48.35386539065191))
    origin_lon_deg = float(gps_origin.get("lon", 11.78159133408772))
    origin_alt_m = float(gps_origin.get("alt", 447.0))

    gt_ws = GroundTruthWebSocketPublisher(host=GT_WS_HOST, port=GT_WS_PORT, enabled=(gt_output_mode == "websocket"))
    gt_ws.start()
    fg_udp: FlightGearUdpPublisher | None = None
    if gt_output_mode == "websocket":
        logger.info("Ground-truth WS target: ws://%s:%s", GT_WS_HOST, GT_WS_PORT)
    elif gt_output_mode == "flightgear_udp":
        fg_udp = FlightGearUdpPublisher(host=FG_UDP_HOST, port=FG_UDP_PORT)
        logger.info("FlightGear UDP target: udp://%s:%s", FG_UDP_HOST, FG_UDP_PORT)
    else:
        logger.info("External ground-truth output disabled")
    logger.info("Ground-truth output mode=%s rate=%.2f Hz", gt_output_mode, gt_output_rate_hz)

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
    next_output_time_us = 0
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
                if catapult_enabled and catapult_countdown_us > 0:
                    catapult_countdown_active = True
                    catapult_release_time_us = sim_time_us + catapult_countdown_us
                    next_countdown_announce_us = sim_time_us
                    logger.info("SIM => ARM transition: catapult launch in %.1f s", catapult_countdown_s)
                else:
                    logger.info("SIM => ARM transition: dynamics enabled")
            if (not armed) and was_armed:
                logger.info("SIM => DISARM transition: dynamics remain enabled")
                if catapult_countdown_active:
                    catapult_countdown_active = False
                    catapult_release_time_us = None
                    logger.info("SIM => Catapult countdown cancelled (disarmed)")
            was_armed = armed

            now_wall_s = time.time()
            if now_wall_s >= next_rx_warn_wall_s and (now_wall_s - last_rx_wall_s) > 2.0:
                logger.warning("SIM: no MAVLink RX for >2s")
                next_rx_warn_wall_s = now_wall_s + 2.0

            world.set_controls(
                controls_to_u(
                    latest_controls,
                    armed,
                )
            )

            world_out = None
            needs_to_pause = False
            now_ms = get_sim_millis(sim_time_us)

            if slave_coupled:
                packet = transfer_slave_link.poll_latest() if transfer_slave_link is not None else None
                if transfer_slave_link is not None:
                    master_endpoint = transfer_slave_link.consume_master_connected_endpoint()
                    if master_endpoint is not None:
                        logger.info(
                            "SIM: transfer connection established to master udp://%s:%s",
                            master_endpoint[0],
                            master_endpoint[1],
                        )

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
                if catapult_countdown_active and catapult_release_time_us is not None:
                    if sim_time_us >= catapult_release_time_us:
                        catapult_countdown_active = False
                        catapult_release_time_us = None
                        logger.info("SIM => Catapult launch release")
                    elif sim_time_us >= next_countdown_announce_us:
                        remaining_s = max(0.0, (catapult_release_time_us - sim_time_us) / 1e6)
                        logger.info("SIM => Catapult countdown: %.1f s", remaining_s)
                        next_countdown_announce_us = sim_time_us + 1_000_000

                freeze_for_countdown = catapult_enabled and catapult_countdown_active
                world_out = world.update(
                    sim_time_us,
                    needs_to_pause,
                    freeze_dynamics=((not ever_armed) or freeze_for_countdown),
                )

                if role == "master" and (not needs_to_pause) and world_out is not None and transfer_master_link is not None:
                    transfer_master_link.send(sim_time_us, world_out["y"], world_out["ydot"])
                    connected_slaves = transfer_master_link.poll_new_slave_connections()
                    for endpoint in connected_slaves:
                        logger.info("SIM: transfer slave connected from udp://%s:%s", endpoint[0], endpoint[1])

            should_publish = world_out is not None and ((not needs_to_pause) or slave_coupled)

            if should_publish:
                z = world_out["sensors"]
                y = np.asarray(world_out["y"], dtype=float)
                gps = np.asarray(z["gps"], dtype=float)
                acc = np.asarray(z["accelerometer"], dtype=float)
                gyro = np.asarray(z["gyroscope"], dtype=float)
                mag = np.asarray(z["magnetometer"], dtype=float)
                baro = z["barometer"]
                fields_updated = 8191
                has_airspeed_sensor = bool(getattr(world.P, "has_airspeed_sensor", False))
                if not has_airspeed_sensor:
                    fields_updated &= ~HIL_SENSOR_UPDATED_DIFF_PRESSURE_BIT

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
                    int(fields_updated),
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

                if gt_output_mode != "off" and sim_time_us >= next_output_time_us:
                    if gt_output_mode == "websocket":
                        alpha_deg, beta_deg = compute_aero_angles_deg(y, world.wind)
                        gt_ws.publish(
                            {
                                "system_id": int(px4_sysid),
                                "time_usec": int(sim_time_us),
                                "u": controls_to_u(latest_controls, armed=True, size=8, clamp_throttle=False).tolist(),
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
                    elif fg_udp is not None:
                        lat_deg, lon_deg, alt_m = ned_to_lla_from_origin(
                            y[0:3],
                            lat_home_deg=origin_lat_deg,
                            lon_home_deg=origin_lon_deg,
                            alt_home_m=origin_alt_m,
                        )
                        quat = np.asarray(y[3:7], dtype=float)
                        quat_norm = float(np.linalg.norm(quat))
                        quat_wxyz = quat / quat_norm if quat_norm > 0.0 else np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
                        euler_deg = np.rad2deg(Quaternion.quat2Euler(quat_wxyz))
                        fg_udp.publish(
                            lat_deg=lat_deg,
                            lon_deg=lon_deg,
                            alt_m=alt_m,
                            roll_deg=float(euler_deg[0]),
                            pitch_deg=float(euler_deg[1]),
                            yaw_deg=float(euler_deg[2]),
                        )

                    next_output_time_us = sim_time_us + gt_output_interval_us


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
        if fg_udp is not None:
            fg_udp.close()


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
