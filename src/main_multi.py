#!/usr/bin/env python3
"""Multi-vehicle MAVLink lockstep bridge: N drones in ONE process, one shared clock.

This is the multi-vehicle sibling of `main.py`. It bridges N PX4 SITL instances
(none_iris), each on TCP port BASE_PORT+i, to N `World` models advanced on a single
shared simulation clock. One process on one clock keeps every vehicle in the same
time base, which is what a co-simulation coupling the drones together would need.

Frames: NED nav / FRD body. All vehicles share one GPS origin (lat/lon/alt), so every
PX4 EKF anchors to the same global reference and the per-drone NED spawn offsets place
them in one common world frame.

Env:
  SIM_NUM_VEHICLES=4            number of drones
  SIM_MAVLINK_BASE_PORT=4560    drone i binds BASE_PORT+i
  SIM_VEHICLE_MODEL=iris        vehicle model
  SIM_FORMATION_SPACING=2.0     grid spacing [m] between spawn points
  SIM_GPS_ORIGIN_LAT/LON/ALT    shared GPS origin
  SIM_GT_OUTPUT_MODE=websocket  per-drone ground-truth output (websocket|off)
  SIM_GT_WS_PORT=8765           one shared websocket; drones keyed by system_id
  SIM_GT_OUTPUT_RATE_HZ=30
  SIM_VIZ_ENABLE=1              scene stream for support/tools/viz/drone_viewer.html
  SIM_VIZ_WS_PORT=8766
  SIM_VIZ_RECORD=path.jsonl     also record every scene frame for offline playback
  SIM_TELEM=path.csv            per-drone truth CSV (diagnostics)
  SIM_SPEED=1.0                 wall-clock pacing: 1.0 = real time, 2.0 = 2x, max = free-run
                                (PX4 lockstep follows the HIL sensor clock, so faster/slower
                                than wall time stays consistent; GT/viz output stays keyed
                                to SIM time)
"""

import os
import time
import logging
import math

import numpy as np
from pymavlink import mavutil

from vehicles.sim_utils import (
    compute_aero_angles_deg,
    controls_to_u,
    get_sim_millis,
    parse_env_float,
    quat_wxyz_to_rot,
)
from dynamics.world import World
from networking.websocket_publisher import GroundTruthWebSocketPublisher
from networking.viz_stream import VizStream
from networking.mavlink_simulator import MavlinkSimulator


RATE_HZ = 250
DT_US = int(1e6 / RATE_HZ)
HEARTBEAT_INTERVAL_US = 1_000_000
SYSTEM_TIME_INTERVAL_US = 1_000_000
GPS_START_DELAY_US = 1_000_000
MAVLINK_MSG_ID_HIL_STATE_QUATERNION = 115
CHECK_FACTOR = 2

LOG_FORMAT = "%(asctime)s | %(levelname)s | %(name)s | %(message)s"
logger = logging.getLogger("main_multi")


def _formation_ned(n: int, spacing: float):
    """Spawn offsets (north, east) for n drones in a compact square-ish grid,
    centered on the origin, `spacing` metres apart."""
    cols = max(1, int(math.ceil(math.sqrt(n))))
    rows = int(math.ceil(n / cols))
    out = []
    for k in range(n):
        r, c = divmod(k, cols)
        north = (r - (rows - 1) / 2.0) * spacing
        east = (c - (cols - 1) / 2.0) * spacing
        out.append((north, east))
    return out


def _make_vehicle(i, model, port, spawn_xy, gps_origin):
    world = World(vehicle_model=model, u0=np.zeros(4), wind0=np.zeros(6))
    world.y[0] = float(spawn_xy[0])  # NED north
    world.y[1] = float(spawn_xy[1])  # NED east
    world.P.gps_origin = dict(gps_origin)
    conn = mavutil.mavlink_connection(f"tcpin:0.0.0.0:{port}", source_component=51)
    return {
        "i": i,
        "port": port,
        "world": world,
        "conn": conn,
        "mav": MavlinkSimulator(conn),
        "latest_controls": None,
        "armed": False,
        "ever_armed": False,
        "hil_state_interval_us": -1,
        "next_hil_state_time_us": 0,
        "sysid": 0,
    }


def _drain_rx(v, sim_time_us):
    """Consume all pending MAVLink from this vehicle's PX4 (actuator controls, arm
    state, HIL_STATE_QUATERNION interval requests)."""
    conn = v["conn"]
    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            break
        mtype = msg.get_type()
        if mtype == "HIL_ACTUATOR_CONTROLS":
            controls = getattr(msg, "controls", None)
            v["latest_controls"] = (
                tuple(float(x) for x in controls)
                if isinstance(controls, (list, tuple))
                else None
            )
            mode = int(getattr(msg, "mode", 0))
            if mode != 0:
                v["armed"] = (mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                if v["armed"]:
                    v["ever_armed"] = True
        elif mtype == "COMMAND_LONG":
            command = int(getattr(msg, "command", 0))
            if command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                msg_id = int(float(getattr(msg, "param1", 0.0)) + 0.5)
                interval_us = int(float(getattr(msg, "param2", -1.0)) + 0.5)
                if msg_id == MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
                    v["hil_state_interval_us"] = interval_us if interval_us > 0 else -1
                    v["next_hil_state_time_us"] = sim_time_us


def simulation_main() -> None:
    n = int(os.getenv("SIM_NUM_VEHICLES", "4"))
    base_port = int(os.getenv("SIM_MAVLINK_BASE_PORT", "4560"))
    model = os.getenv("SIM_VEHICLE_MODEL", "iris").strip().lower()
    spacing = parse_env_float("SIM_FORMATION_SPACING", 2.0)
    gps_origin = {
        "lat": parse_env_float("SIM_GPS_ORIGIN_LAT", 47.397742),
        "lon": parse_env_float("SIM_GPS_ORIGIN_LON", 8.545594),
        "alt": parse_env_float("SIM_GPS_ORIGIN_ALT", 488.0),
    }
    gt_mode = os.getenv("SIM_GT_OUTPUT_MODE", "websocket").strip().lower()
    gt_ws_host = os.getenv("SIM_GT_WS_HOST", "0.0.0.0")
    gt_ws_port = int(os.getenv("SIM_GT_WS_PORT", "8765"))
    gt_rate_hz = parse_env_float("SIM_GT_OUTPUT_RATE_HZ", 30.0)
    gt_interval_us = max(1, int(1e6 / gt_rate_hz))

    speed_raw = os.getenv("SIM_SPEED", "1.0").strip().lower()
    speed = None if speed_raw in {"max", "0", "inf"} else float(speed_raw)

    telem_path = os.getenv("SIM_TELEM", "").strip()        # per-drone truth CSV
    telem_every = int(os.getenv("SIM_TELEM_EVERY", "5"))   # write every Nth published tick

    spawns = _formation_ned(n, spacing)
    vehicles = [
        _make_vehicle(i, model, base_port + i, spawns[i], gps_origin)
        for i in range(n)
    ]
    logger.info(
        "Multi-vehicle bridge: %d x %s, ports %d..%d, grid spacing %.2f m, shared origin (%.6f, %.6f, %.1f)",
        n, model, base_port, base_port + n - 1, spacing,
        gps_origin["lat"], gps_origin["lon"], gps_origin["alt"],
    )

    # Wait for each PX4 to connect + heartbeat (all ports already listening).
    for v in vehicles:
        logger.info("drone %d: waiting for PX4 heartbeat on TCP %d ...", v["i"], v["port"])
        hb = v["conn"].wait_heartbeat()
        sysid = hb.get_srcSystem()
        v["sysid"] = sysid
        if sysid > 0:
            v["conn"].source_system = sysid
            v["conn"].mav.srcSystem = sysid
        logger.info("drone %d: connected, PX4 SYSID=%s", v["i"], sysid)

    gt_ws = GroundTruthWebSocketPublisher(
        host=gt_ws_host, port=gt_ws_port, enabled=(gt_mode == "websocket")
    )
    gt_ws.start()
    if gt_mode == "websocket":
        logger.info("Ground-truth WS: ws://%s:%s (drones keyed by system_id)", gt_ws_host, gt_ws_port)

    viz_enable = os.getenv("SIM_VIZ_ENABLE", "1").strip().lower() not in {"0", "false", "off"}
    viz = VizStream(port=int(os.getenv("SIM_VIZ_WS_PORT", "8766")),
                    record_path=os.getenv("SIM_VIZ_RECORD", "").strip(),
                    source="main_multi", enabled=viz_enable)
    viz.set_meta(n_drones=n, spacing=spacing, extra=dict(stack=[
        "bridge: MAVLink HIL lockstep @ 250 Hz",
        "control: REAL PX4 firmware x%d — EKF2 (from simulated noisy IMU/GPS/baro/mag) "
        "+ stock position/attitude/rate cascade" % n,
        "commands: offboard setpoints via MAVLink (offboard_demo.py) or your GCS",
        "plant: %dx World 6DOF (%s)" % (n, model),
        "pacing: SIM_SPEED=%s" % speed_raw,
    ]))
    viz.start()

    logger.info("All %d PX4 connected; starting sim.", n)

    telem_f = None
    if telem_path:
        telem_f = open(telem_path, "w")
        cols = ["t_s"]
        for i in range(n):
            cols += [f"d{i}_{c}" for c in ("N", "E", "D", "vN", "vE", "vD", "tilt")]
        telem_f.write(",".join(cols) + "\n")
        logger.info("telemetry -> %s (every %d ticks)", telem_path, telem_every)

    sim_time_us = 0
    next_heartbeat_us = 0
    next_system_time_us = 0
    next_gt_us = 0
    last_time_ran_ms = 0
    slow_down_counter = 0
    wall0 = time.perf_counter()   # deadline pacing: sim_time/speed anchored here (no drift)

    try:
        while True:
            # 1) RX from every PX4 + set its controls
            for v in vehicles:
                _drain_rx(v, sim_time_us)
                v["world"].set_controls(controls_to_u(v["latest_controls"], v["armed"]))

            # 2) shared pacing (one clock for all drones)
            io_run_only = (slow_down_counter % CHECK_FACTOR) != 0
            now_ms = get_sim_millis(sim_time_us)
            needs_to_pause = (last_time_ran_ms == now_ms) or io_run_only
            sim_time_us += DT_US
            now_ms = get_sim_millis(sim_time_us)

            # 3) step + publish each vehicle
            do_hb = sim_time_us >= next_heartbeat_us
            do_sys = sim_time_us >= next_system_time_us
            do_gt = (gt_mode != "off") and (sim_time_us >= next_gt_us)
            published = False
            for v in vehicles:
                out = v["world"].update(
                    sim_time_us, needs_to_pause, freeze_dynamics=(not v["ever_armed"])
                )
                should_publish = (out is not None) and (not needs_to_pause)
                if should_publish:
                    z = out["sensors"]
                    y = np.asarray(out["y"], dtype=float)
                    v["mav"].send_hil_sensor(sim_time_us, z, False)
                    if v["hil_state_interval_us"] > 0 and sim_time_us >= v["next_hil_state_time_us"]:
                        v["mav"].send_hil_state_quaternion(sim_time_us, y, z)
                        v["next_hil_state_time_us"] = sim_time_us + v["hil_state_interval_us"]
                    if do_sys:
                        v["mav"].send_system_time(sim_time_us)
                    if sim_time_us >= GPS_START_DELAY_US and bool(z.get("gps_updated", False)):
                        v["mav"].send_hil_gps(sim_time_us, z)
                    if do_gt:
                        gps = np.asarray(z["gps"], dtype=float)
                        alpha_deg, beta_deg = compute_aero_angles_deg(y, v["world"].wind)
                        gt_ws.publish(
                            {
                                "system_id": int(v["sysid"]),
                                "time_usec": int(sim_time_us),
                                "position_ned_m": [float(y[0]), float(y[1]), float(y[2])],
                                "quaternion_wxyz": [float(y[3]), float(y[4]), float(y[5]), float(y[6])],
                                "velocity_body_mps": [float(y[7]), float(y[8]), float(y[9])],
                                "angular_rate_body_rps": [float(y[10]), float(y[11]), float(y[12])],
                                "lla": {"lat_deg": float(gps[0]), "lon_deg": float(gps[1]), "alt_m": float(gps[2])},
                                "aero": {"alpha_deg": alpha_deg, "beta_deg": beta_deg},
                            }
                        )
                    if do_hb:
                        v["mav"].send_heartbeat()
                    published = True

            if published:
                viz.frame(sim_time_us / 1e6, [v["world"] for v in vehicles])
                last_time_ran_ms = now_ms
                if do_sys:
                    next_system_time_us = sim_time_us + SYSTEM_TIME_INTERVAL_US
                if do_gt:
                    next_gt_us = sim_time_us + gt_interval_us
            if telem_f is not None and published and (slow_down_counter % telem_every == 0):
                row = [sim_time_us / 1e6]
                for v in vehicles:
                    y = v["world"].y
                    R = quat_wxyz_to_rot(y[3:7])
                    wv = R @ y[7:10]                                   # body vel -> world NED
                    tilt = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
                    row += [y[0], y[1], y[2], wv[0], wv[1], wv[2], tilt]
                telem_f.write(",".join(f"{x:.4f}" for x in row) + "\n")

            if do_hb:
                cmd = viz.pop("cmd")
                if cmd is not None and cmd.get("cmd") == "shutdown":
                    logger.info("shutdown requested from the viewer; exiting bridge "
                                "(PX4 instances keep running; support/tools/stop.sh for full teardown).")
                    break
                next_heartbeat_us = sim_time_us + HEARTBEAT_INTERVAL_US
                if telem_f is not None:
                    telem_f.flush()

            slow_down_counter += 1
            if speed is not None:
                lag = (sim_time_us / 1e6) / speed - (time.perf_counter() - wall0)
                if lag > 0:
                    time.sleep(lag)
    finally:
        gt_ws.stop()
        viz.stop()
        if telem_f is not None:
            telem_f.close()


def main() -> None:
    logging.basicConfig(
        level=getattr(logging, os.getenv("SIM_LOG_LEVEL", "INFO").upper(), logging.INFO),
        format=LOG_FORMAT,
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    try:
        simulation_main()
    except KeyboardInterrupt:
        logger.info("Stopped.")


if __name__ == "__main__":
    main()
