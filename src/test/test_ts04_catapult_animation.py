import logging

import numpy as np

from vehicle_animation_common import animate_vehicle, plot_sensor_suite_overview, run_vehicle_sim


def run_ts04_catapult_sim(total_time_s: float = 20.0, dt_s: float = 0.01):
    controls = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
    return run_vehicle_sim("ts04", controls=controls, total_time_s=total_time_s, dt_s=dt_s)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    t_hist, y_hist, left_rail_hist, sensor_hist = run_ts04_catapult_sim(total_time_s=20.0, dt_s=0.01)
    plot_sensor_suite_overview(t_hist, sensor_hist, title_prefix="TS04 Catapult Launch")
    animate_vehicle(t_hist, y_hist, title_prefix="TS04 Catapult Launch", left_rail_hist=left_rail_hist)
