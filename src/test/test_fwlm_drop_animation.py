import logging

import numpy as np

from dynamics.world import World
from vehicle_animation_common import animate_vehicle, plot_sensor_suite_overview


def build_drop_initial_state():
    y0 = np.zeros(13)
    y0[0:3] = np.array([0.0, 0.0, -300.0], dtype=float)
    y0[3] = 1.0
    y0[7:10] = np.array([60.0, 0.0, 0.0], dtype=float)
    return y0


def run_fwlm_drop_sim(total_time_s: float = 20.0, dt_s: float = 0.01, stop_on_ground_contact: bool = True):
    y0 = build_drop_initial_state()
    u0 = np.zeros(4, dtype=float)
    u0 =  np.array([0.0, -0.1, 0.1, 0.0], dtype=float)
    wind0 = np.zeros(6, dtype=float)
    world = World(vehicle_model="fwlm", y0=y0, u0=u0, wind0=wind0)
    world.P.debug_alpha_beta = True
    world.P.debug_alpha_beta_stride = 10

    steps = int(total_time_s / dt_s)
    dt_us = int(dt_s * 1e6)

    y_hist = np.zeros((steps, 13), dtype=float)
    t_hist = np.zeros(steps, dtype=float)
    left_rail_hist = np.zeros(steps, dtype=bool)
    accel_hist = np.zeros((steps, 3), dtype=float)
    gyro_hist = np.zeros((steps, 3), dtype=float)
    mag_hist = np.zeros((steps, 3), dtype=float)
    gps_hist = np.zeros((steps, 7), dtype=float)
    baro_static_hist = np.zeros(steps, dtype=float)
    baro_dynamic_hist = np.zeros(steps, dtype=float)
    gps_updated_hist = np.zeros(steps, dtype=bool)
    alpha_deg_hist = np.zeros(steps, dtype=float)
    beta_deg_hist = np.zeros(steps, dtype=float)

    ground_latched = False
    was_airborne = False

    for k in range(steps):
        t_us = (k + 1) * dt_us
        if ground_latched and k > 0:
            y_hist[k] = y_hist[k - 1]
            t_hist[k] = t_us / 1e6
            left_rail_hist[k] = left_rail_hist[k - 1]
            accel_hist[k] = accel_hist[k - 1]
            gyro_hist[k] = gyro_hist[k - 1]
            mag_hist[k] = mag_hist[k - 1]
            gps_hist[k] = gps_hist[k - 1]
            baro_static_hist[k] = baro_static_hist[k - 1]
            baro_dynamic_hist[k] = baro_dynamic_hist[k - 1]
            gps_updated_hist[k] = gps_updated_hist[k - 1]
            alpha_deg_hist[k] = alpha_deg_hist[k - 1]
            beta_deg_hist[k] = beta_deg_hist[k - 1]
            continue

        out = world.update(t_us, paused=False, freeze_dynamics=False)
        y_hist[k] = out["y"]
        t_hist[k] = t_us / 1e6
        left_rail_hist[k] = bool(getattr(world.P, "left_rail", False))

        sensors = out["sensors"]
        accel_hist[k] = np.asarray(sensors["accelerometer"], dtype=float)
        gyro_hist[k] = np.asarray(sensors["gyroscope"], dtype=float)
        mag_hist[k] = np.asarray(sensors["magnetometer"], dtype=float)
        gps_hist[k] = np.asarray(sensors["gps"], dtype=float)
        baro_static_hist[k] = float(sensors["barometer"]["staticAbsolute"])
        baro_dynamic_hist[k] = float(sensors["barometer"]["dynamic"])
        gps_updated_hist[k] = bool(sensors.get("gps_updated", False))

        vel_body = np.asarray(out["y"][7:10], dtype=float)
        wind_body = np.asarray(world.wind[:3], dtype=float)
        vel_rel = vel_body - wind_body
        u_r, v_r, w_r = vel_rel
        v_air = max(float(np.linalg.norm(vel_rel)), 1.0e-5)
        alpha_deg_hist[k] = float(np.rad2deg(np.arctan2(w_r, u_r)))
        beta_deg_hist[k] = float(np.rad2deg(np.arcsin(np.clip(v_r / v_air, -1.0, 1.0))))

        if float(out["y"][2]) < 0.0:
            was_airborne = True

        if stop_on_ground_contact and was_airborne and float(out["y"][2]) >= 0.0:
            ground_latched = True

    sensor_hist = {
        "accelerometer": accel_hist,
        "gyroscope": gyro_hist,
        "magnetometer": mag_hist,
        "gps": gps_hist,
        "baro_static": baro_static_hist,
        "baro_dynamic": baro_dynamic_hist,
        "gps_updated": gps_updated_hist,
        "alpha_deg": alpha_deg_hist,
        "beta_deg": beta_deg_hist,
    }

    return t_hist, y_hist, left_rail_hist, sensor_hist


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    t_hist, y_hist, left_rail_hist, sensor_hist = run_fwlm_drop_sim(total_time_s=20.0, dt_s=0.01)
    plot_sensor_suite_overview(t_hist, sensor_hist, title_prefix="FWLM Drop From 300m")
    animate_vehicle(t_hist, y_hist, title_prefix="FWLM Drop From 300m", left_rail_hist=left_rail_hist)
