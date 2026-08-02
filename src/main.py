#!/usr/bin/env python3
"""MAVLink lockstep simulation endpoint backed by vehicle.World."""

import os
import time
import logging
from typing import Any

import numpy as np
from pymavlink import mavutil

from vehicles.sim_utils import (
    compute_aero_angles_deg,
    controls_to_u,
    get_sim_millis,
    ned_to_lla_from_origin,
    parse_env_float,
    parse_gt_output_mode,
    parse_positive_float,
    parse_vehicle_model,
)
from dynamics.quaternion import Quaternion
from dynamics.world import World
from vehicles.vehicle_catalog import list_vehicle_models
from networking.websocket_publisher import GroundTruthWebSocketPublisher
from networking.flightgear_udp_publisher import FlightGearUdpPublisher
from networking.mavlink_simulator import MavlinkSimulator



RATE_HZ = 250
DT_US = int(1e6 / RATE_HZ)
HEARTBEAT_INTERVAL_US = 1_000_000
SYSTEM_TIME_INTERVAL_US = 1_000_000
GPS_START_DELAY_US = 1_000_000
MAVLINK_MSG_ID_HIL_STATE_QUATERNION = 115
CHECK_FACTOR = 2

MAVLINK_BIND_HOST = os.getenv("SIM_MAVLINK_BIND_HOST", "0.0.0.0")
MAVLINK_BIND_PORT = int(os.getenv("SIM_MAVLINK_BIND_PORT", "4560"))

GT_WS_HOST = os.getenv("SIM_GT_WS_HOST", "0.0.0.0")
GT_WS_PORT = int(os.getenv("SIM_GT_WS_PORT", "8765"))
GT_OUTPUT_MODE = os.getenv("SIM_GT_OUTPUT_MODE", "websocket").strip().lower()
GT_OUTPUT_RATE_HZ_RAW = os.getenv("SIM_GT_OUTPUT_RATE_HZ", "30.0")
FG_UDP_HOST = os.getenv("SIM_FG_UDP_HOST", "127.0.0.1")
FG_UDP_PORT = int(os.getenv("SIM_FG_UDP_PORT", "5503"))
VEHICLE_MODEL = os.getenv("SIM_VEHICLE_MODEL", "x8").strip().lower()

LOG_FORMAT = "%(asctime)s | %(levelname)s | %(name)s | %(message)s"
logger = logging.getLogger(__name__)
DEFAULT_GPS_ORIGIN = {
    "lat": 0.0,
    "lon": 0.0,
    "alt": 0.0,
}


def simulation_main() -> None:
    gt_output_mode = parse_gt_output_mode(GT_OUTPUT_MODE)
    gt_output_rate_hz = parse_positive_float(GT_OUTPUT_RATE_HZ_RAW, "SIM_GT_OUTPUT_RATE_HZ")
    gt_output_interval_us = max(1, int(1e6 / gt_output_rate_hz))
    available_vehicle_models = list_vehicle_models()
    vehicle_model = parse_vehicle_model(VEHICLE_MODEL, available_vehicle_models)

    mavlink_endpoint = f"tcpin:{MAVLINK_BIND_HOST}:{MAVLINK_BIND_PORT}"
    conn: Any = mavutil.mavlink_connection(mavlink_endpoint, source_component=51)
    mav_sim = MavlinkSimulator(conn)

    logger.info("Waiting for Heartbeat ...")
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
    )
    catapult_enabled = bool(getattr(world, "rail_launch_enabled", False))
    catapult_countdown_s = max(0.0, parse_env_float("SIM_CATAPULT_LAUNCH_COUNTDOWN_S", 3.0))
    catapult_countdown_us = int(catapult_countdown_s * 1e6)
    catapult_countdown_active = False
    catapult_release_time_us: int | None = None
    next_countdown_announce_us = 0
    if catapult_enabled:
        logger.info("Catapult launch enabled with countdown %.1f s", catapult_countdown_s)

    gps_origin = getattr(world.P, "gps_origin", DEFAULT_GPS_ORIGIN)
    origin_lat_deg = float(gps_origin.get("lat", DEFAULT_GPS_ORIGIN["lat"]))
    origin_lon_deg = float(gps_origin.get("lon", DEFAULT_GPS_ORIGIN["lon"]))
    origin_alt_m = float(gps_origin.get("alt", DEFAULT_GPS_ORIGIN["alt"]))

    # Override GPS starting origin from environment variables if set
    env_lat = os.getenv("SIM_GPS_ORIGIN_LAT")
    env_lon = os.getenv("SIM_GPS_ORIGIN_LON")
    env_alt = os.getenv("SIM_GPS_ORIGIN_ALT")
    if env_lat is not None:
        try:
            origin_lat_deg = float(env_lat)
            logger.info("Overriding GPS Origin Latitude from environment: %s", origin_lat_deg)
        except ValueError:
            logger.warning("Invalid SIM_GPS_ORIGIN_LAT: '%s', ignoring override.", env_lat)
    if env_lon is not None:
        try:
            origin_lon_deg = float(env_lon)
            logger.info("Overriding GPS Origin Longitude from environment: %s", origin_lon_deg)
        except ValueError:
            logger.warning("Invalid SIM_GPS_ORIGIN_LON: '%s', ignoring override.", env_lon)
    if env_alt is not None:
        try:
            origin_alt_m = float(env_alt)
            logger.info("Overriding GPS Origin Altitude from environment: %s", origin_alt_m)
        except ValueError:
            logger.warning("Invalid SIM_GPS_ORIGIN_ALT: '%s', ignoring override.", env_alt)

    # Write back the overridden coordinates to world.P.gps_origin so sensors pick them up
    world.P.gps_origin = {
        "lat": origin_lat_deg,
        "lon": origin_lon_deg,
        "alt": origin_alt_m,
    }

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

            
            io_run_only = (slow_down_counter % CHECK_FACTOR) != 0
            needs_to_pause = (last_time_ran_ms == now_ms) or io_run_only
            sim_time_us += DT_US
            now_ms = get_sim_millis(sim_time_us)

                
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

            should_publish = world_out is not None and (not needs_to_pause)

            if should_publish:
                z = world_out["sensors"]
                y = np.asarray(world_out["y"], dtype=float)
                has_airspeed_sensor = bool(getattr(world.P, "has_airspeed_sensor", False))

                mav_sim.send_hil_sensor(sim_time_us, z, has_airspeed_sensor)

                if hil_state_interval_us > 0 and sim_time_us >= next_hil_state_time_us:
                    mav_sim.send_hil_state_quaternion(sim_time_us, y, z)
                    next_hil_state_time_us = sim_time_us + hil_state_interval_us

                if sim_time_us >= next_system_time_us:
                    mav_sim.send_system_time(sim_time_us)
                    next_system_time_us = sim_time_us + SYSTEM_TIME_INTERVAL_US

                if sim_time_us >= gps_start_time_us and bool(z.get("gps_updated", False)):
                    mav_sim.send_hil_gps(sim_time_us, z)

                if gt_output_mode != "off" and sim_time_us >= next_output_time_us:
                    if gt_output_mode == "websocket":
                        gps = np.asarray(z["gps"], dtype=float)
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
                mav_sim.send_heartbeat()
                next_heartbeat_time_us = sim_time_us + HEARTBEAT_INTERVAL_US

            slow_down_counter += 1
            time.sleep(1.0 / RATE_HZ)
    finally:
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
