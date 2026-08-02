#!/usr/bin/env python3
"""Minimal MAVLink offboard demo: arm N PX4 instances, take off, hover (or orbit).

A dependency-light commander so the multi-vehicle bridge has something to fly. It
talks plain MAVLink to each PX4 SITL instance and drives it with PX4's own stock
controllers, no custom control law. One thread per drone owns its link and streams
SET_POSITION_TARGET_LOCAL_NED setpoints; offboard mode needs a setpoint already
streaming before the mode switch, so we warm up first, then request OFFBOARD + arm.

Each PX4's local NED origin is its own EKF home (its spawn point), so commanding
(0, 0, -alt) lifts every drone straight up from where it started and the formation
takes off together.

Env:
  SIM_NUM_VEHICLES=4          number of drones (matches the bridge)
  SIM_OFFBOARD_BASE_PORT=18570  drone i GCS link at BASE_PORT+i (PX4 SITL Normal-mode UDP;
                                this link adopts our sender as partner and streams back,
                                unlike the onboard link on 14580 which only receives)
  SIM_DEMO_ALT=3.0            hover altitude [m]
  SIM_DEMO=hover|orbit        hover in place, or fly a slow circle
  SIM_DEMO_RADIUS=1.5         orbit radius [m]
  SIM_DEMO_PERIOD=20.0        orbit period [s]
"""

import math
import os
import threading
import time

from pymavlink import mavutil

# SET_POSITION_TARGET_LOCAL_NED type_mask: use position (x,y,z) + yaw, ignore the rest.
IGN_VX, IGN_VY, IGN_VZ = 8, 16, 32
IGN_AX, IGN_AY, IGN_AZ = 64, 128, 256
IGN_YAW_RATE = 2048
TYPE_MASK_POS_YAW = IGN_VX | IGN_VY | IGN_VZ | IGN_AX | IGN_AY | IGN_AZ | IGN_YAW_RATE

SETPOINT_HZ = 20.0
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


def _send_setpoint(conn, tgt_sys, north, east, down, yaw=0.0):
    conn.mav.set_position_target_local_ned_send(
        0,                       # time_boot_ms (0 = let PX4 stamp)
        tgt_sys, 1,              # target system, component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK_POS_YAW,
        north, east, down,       # position NED [m]
        0, 0, 0,                 # velocity (ignored)
        0, 0, 0,                 # accel (ignored)
        yaw, 0.0,                # yaw [rad], yaw_rate (ignored)
    )


def _set_offboard(conn, tgt_sys):
    conn.mav.command_long_send(
        tgt_sys, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD, 0, 0, 0, 0, 0,
    )


def _arm(conn, tgt_sys):
    conn.mav.command_long_send(
        tgt_sys, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0,
    )


def _is_armed(conn):
    hb = conn.recv_match(type="HEARTBEAT", blocking=False)
    if hb is None:
        return None
    return (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0


def _send_heartbeat(conn):
    conn.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)


def fly_one(i, port, alt, demo, radius, period, stop_evt):
    conn = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}")
    tgt = i + 1              # PX4 SITL system id = instance + 1 (refined once we hear one)
    dt = 1.0 / SETPOINT_HZ

    # 1) handshake + warm-up. On a udpout link PX4 only streams back once it has heard
    # from us, and offboard mode needs a setpoint already flowing before the switch,
    # so we send our heartbeat + a zero setpoint until PX4 starts replying.
    print(f"[offboard] drone {i}: handshaking on udp {port} ...", flush=True)
    heard = False
    t0 = time.perf_counter()
    while not stop_evt.is_set():
        _send_heartbeat(conn)
        _send_setpoint(conn, tgt, 0.0, 0.0, 0.0)
        hb = conn.recv_match(type="HEARTBEAT", blocking=False)
        if hb is not None and hb.get_srcSystem() > 0:
            tgt = hb.get_srcSystem()
            heard = True
        now = time.perf_counter()
        if heard and now - t0 > 1.0:
            break                                # ~1 s of established two-way stream
        if now - t0 > 15.0:
            print(f"[offboard] drone {i}: no heartbeat on {port} after 15 s "
                  "(is the bridge/PX4 up?)", flush=True)
            t0 = now
        time.sleep(dt)
    print(f"[offboard] drone {i}: connected (sysid {tgt})", flush=True)

    # 2) request offboard + arm, keeping the setpoint stream and our heartbeat alive
    armed = False
    t_last_cmd = t_last_hb = 0.0
    t0 = time.perf_counter()
    while not armed and not stop_evt.is_set():
        _send_setpoint(conn, tgt, 0.0, 0.0, 0.0)
        now = time.perf_counter()
        if now - t_last_hb > 1.0:
            _send_heartbeat(conn); t_last_hb = now
        if now - t_last_cmd > 1.0:
            _set_offboard(conn, tgt)
            _arm(conn, tgt)
            t_last_cmd = now
        if _is_armed(conn):
            armed = True
        if now - t0 > 40.0:
            print(f"[offboard] drone {i}: still not armed after 40 s "
                  "(check PX4 preflight / EKF)", flush=True)
            t0 = now
        time.sleep(dt)
    if not armed:
        return
    print(f"[offboard] drone {i}: armed, taking off to {alt:.1f} m", flush=True)

    # 3) ramp altitude, then hold (hover) or orbit
    tstart = t_last_hb = time.perf_counter()
    ramp_s = 4.0
    while not stop_evt.is_set():
        t = time.perf_counter() - tstart
        climb = min(1.0, t / ramp_s)
        down = -alt * climb
        if demo == "orbit" and t > ramp_s and period > 0:
            ang = 2.0 * math.pi * (t - ramp_s) / period
            north = radius * math.cos(ang)
            east = radius * math.sin(ang)
            yaw = ang + math.pi / 2.0            # face along the tangent
        else:
            north = east = 0.0
            yaw = 0.0
        _send_setpoint(conn, tgt, north, east, down, yaw)
        now = time.perf_counter()
        if now - t_last_hb > 1.0:
            _send_heartbeat(conn); t_last_hb = now
        conn.recv_match(blocking=False)          # drain rx so the socket buffer stays clear
        time.sleep(dt)


def main():
    n = int(os.getenv("SIM_NUM_VEHICLES", "4"))
    base_port = int(os.getenv("SIM_OFFBOARD_BASE_PORT", "18570"))
    alt = float(os.getenv("SIM_DEMO_ALT", "3.0"))
    demo = os.getenv("SIM_DEMO", "hover").strip().lower()
    radius = float(os.getenv("SIM_DEMO_RADIUS", "1.5"))
    period = float(os.getenv("SIM_DEMO_PERIOD", "20.0"))

    print(f"[offboard] demo={demo} N={n} alt={alt} m, ports {base_port}..{base_port + n - 1}",
          flush=True)
    stop_evt = threading.Event()
    threads = [
        threading.Thread(target=fly_one,
                         args=(i, base_port + i, alt, demo, radius, period, stop_evt),
                         daemon=True)
        for i in range(n)
    ]
    for th in threads:
        th.start()
    try:
        while any(th.is_alive() for th in threads):
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[offboard] stopping (drones stay armed; land/disarm via your GCS or stop.sh)",
              flush=True)
        stop_evt.set()
        for th in threads:
            th.join(timeout=1.0)


if __name__ == "__main__":
    main()
