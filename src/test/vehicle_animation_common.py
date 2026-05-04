import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from dynamics.quaternion import Quaternion
from dynamics.world import World


CATAPULT_DIR_NED = np.array([np.cos(np.deg2rad(45.0)), 0.0, -np.sin(np.deg2rad(45.0))], dtype=float)
CATAPULT_START_NED = np.array([0.0, 0.0, 0.0], dtype=float)
CATAPULT_LENGTH_M = 2.0


def configure_catapult(world: World) -> None:
    p = world.P
    p.rail_launch_enabled = True
    p.rail_dir_ned = CATAPULT_DIR_NED.copy()
    p.rail_start_ned = CATAPULT_START_NED.copy()
    p.rail_length = CATAPULT_LENGTH_M
    p.rail_pull_max = 50.0
    p.left_rail = False

    world.rail_launch_enabled = True
    world._configure_rail_launch()


def run_vehicle_sim(
    vehicle_model: str,
    controls: np.ndarray,
    total_time_s: float = 20.0,
    dt_s: float = 0.01,
    stop_on_ground_contact: bool = True,
    debug_alpha_beta: bool = False,
    debug_alpha_beta_stride: int = 10,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    world = World(vehicle_model=vehicle_model, u0=np.zeros(4), wind0=np.zeros(6))
    if hasattr(world.P, "debug_alpha_beta"):
        world.P.debug_alpha_beta = bool(debug_alpha_beta)
    if hasattr(world.P, "debug_alpha_beta_stride"):
        world.P.debug_alpha_beta_stride = int(max(debug_alpha_beta_stride, 1))
    configure_catapult(world)

    controls = np.asarray(controls, dtype=float)
    if controls.shape[0] < 4:
        controls = np.pad(controls, (0, 4 - controls.shape[0]))

    steps = int(total_time_s / dt_s)
    dt_us = int(dt_s * 1e6)

    y_hist = np.zeros((steps, 13))
    t_hist = np.zeros(steps)
    left_rail_hist = np.zeros(steps, dtype=bool)
    accel_hist = np.zeros((steps, 3))
    gyro_hist = np.zeros((steps, 3))
    mag_hist = np.zeros((steps, 3))
    gps_hist = np.zeros((steps, 7))
    baro_static_hist = np.zeros(steps)
    baro_dynamic_hist = np.zeros(steps)
    gps_updated_hist = np.zeros(steps, dtype=bool)
    alpha_deg_hist = np.zeros(steps)
    beta_deg_hist = np.zeros(steps)

    ground_latched = False
    was_airborne = False

    for k in range(steps):
        t_us = (k + 1) * dt_us
        if ground_latched and k > 0:
            y_hist[k] = y_hist[k - 1]
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
            t_hist[k] = t_us / 1e6
            continue

        world.set_controls(controls)
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


def set_axes_equal(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    x_middle = 0.5 * (x_limits[0] + x_limits[1])
    y_middle = 0.5 * (y_limits[0] + y_limits[1])
    z_middle = 0.5 * (z_limits[0] + z_limits[1])

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


def animate_vehicle(
    t_hist: np.ndarray,
    y_hist: np.ndarray,
    title_prefix: str,
    left_rail_hist: np.ndarray | None = None,
) -> None:
    pos_ned = y_hist[:, 0:3]
    quat = y_hist[:, 3:7]
    speed = np.linalg.norm(y_hist[:, 7:10], axis=1)
    vel_body = np.asarray(y_hist[:, 7:10], dtype=float)
    va = np.maximum(np.linalg.norm(vel_body, axis=1), 1.0e-5)
    alpha_deg = np.rad2deg(np.arctan2(vel_body[:, 2], vel_body[:, 0]))
    beta_deg = np.rad2deg(np.arcsin(np.clip(vel_body[:, 1] / va, -1.0, 1.0)))

    north = pos_ned[:, 0]
    east = pos_ned[:, 1]
    up = -pos_ned[:, 2]

    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection="3d")

    body_lines = {
        "x": ax.plot([], [], [], "r-", lw=2)[0],
        "y": ax.plot([], [], [], "g-", lw=2)[0],
        "z": ax.plot([], [], [], "b-", lw=2)[0],
    }
    trajectory_line = ax.plot([], [], [], "k--", lw=1.5)[0]

    margin = 4.0
    ax.set_xlim(np.min(north) - margin, np.max(north) + margin)
    ax.set_ylim(np.min(east) - margin, np.max(east) + margin)
    ax.set_zlim(np.min(up) - margin, np.max(up) + margin)

    ax.set_xlabel("North [m]")
    ax.set_ylabel("East [m]")
    ax.set_zlabel("Up [m]")
    ax.view_init(elev=20, azim=-120)
    set_axes_equal(ax)

    catapult_end_ned = CATAPULT_START_NED + CATAPULT_DIR_NED * CATAPULT_LENGTH_M
    ax.plot(
        [CATAPULT_START_NED[0], catapult_end_ned[0]],
        [CATAPULT_START_NED[1], catapult_end_ned[1]],
        [-CATAPULT_START_NED[2], -catapult_end_ned[2]],
        color="gray",
        lw=6,
        alpha=0.85,
    )

    axis_scale = 3.0
    follow_half_range = 16.0

    def update(frame_idx):
        p_ned = pos_ned[frame_idx]
        q = quat[frame_idx]
        q_norm = np.linalg.norm(q)
        if q_norm > 0.0:
            q = q / q_norm

        mfg = Quaternion.Mfg(q)
        mgf = mfg.T

        origin = np.array([p_ned[0], p_ned[1], -p_ned[2]])
        axes_body = np.eye(3) * axis_scale
        axes_ned = mgf @ axes_body

        for i, axis_name in enumerate(["x", "y", "z"]):
            end_ned = p_ned + axes_ned[:, i]
            body_lines[axis_name].set_data([origin[0], end_ned[0]], [origin[1], end_ned[1]])
            body_lines[axis_name].set_3d_properties([origin[2], -end_ned[2]])

        trajectory_line.set_data(north[: frame_idx + 1], east[: frame_idx + 1])
        trajectory_line.set_3d_properties(up[: frame_idx + 1])

        ax.set_xlim(origin[0] - follow_half_range, origin[0] + follow_half_range)
        ax.set_ylim(origin[1] - follow_half_range, origin[1] + follow_half_range)
        ax.set_zlim(origin[2] - follow_half_range, origin[2] + follow_half_range)
        left_rail = False if left_rail_hist is None else bool(left_rail_hist[frame_idx])
        ax.set_title(
            f"{title_prefix} | t = {t_hist[frame_idx]:.1f} s | "
            f"V = {speed[frame_idx]:.2f} m/s | "
            f"alpha = {alpha_deg[frame_idx]:.1f} deg | beta = {beta_deg[frame_idx]:.1f} deg | "
            f"left_rail={left_rail}"
        )
        return list(body_lines.values()) + [trajectory_line]

    ani = FuncAnimation(fig, update, frames=len(t_hist), interval=20, blit=False)
    _ = ani
    plt.tight_layout()
    plt.show()


def plot_sensor_suite_overview(t_hist: np.ndarray, sensor_hist: dict[str, np.ndarray], title_prefix: str) -> None:
    acc = sensor_hist["accelerometer"]
    gyro = sensor_hist["gyroscope"]
    baro_dynamic = sensor_hist["baro_dynamic"]
    gps_updated = sensor_hist["gps_updated"].astype(float)
    alpha_deg = sensor_hist.get("alpha_deg", np.zeros_like(t_hist))
    beta_deg = sensor_hist.get("beta_deg", np.zeros_like(t_hist))

    fig, axs = plt.subplots(5, 1, figsize=(9, 10), sharex=True)

    axs[0].plot(t_hist, acc[:, 0], label="ax")
    axs[0].plot(t_hist, acc[:, 1], label="ay")
    axs[0].plot(t_hist, acc[:, 2], label="az")
    axs[0].set_ylabel("Accel [m/s^2]")
    axs[0].legend(loc="upper right")
    axs[0].grid(True)

    axs[1].plot(t_hist, gyro[:, 0], label="p")
    axs[1].plot(t_hist, gyro[:, 1], label="q")
    axs[1].plot(t_hist, gyro[:, 2], label="r")
    axs[1].set_ylabel("Gyro [rad/s]")
    axs[1].legend(loc="upper right")
    axs[1].grid(True)

    axs[2].plot(t_hist, baro_dynamic, label="q_dyn")
    axs[2].set_ylabel("DynPress [Pa]")
    axs[2].legend(loc="upper right")
    axs[2].grid(True)

    axs[3].plot(t_hist, gps_updated, label="gps_updated")
    axs[3].set_ylabel("GPS upd")
    axs[3].set_ylim(-0.1, 1.1)
    axs[3].legend(loc="upper right")
    axs[3].grid(True)

    axs[4].plot(t_hist, alpha_deg, label="alpha")
    axs[4].plot(t_hist, beta_deg, label="beta")
    axs[4].set_ylabel("Angles [deg]")
    axs[4].set_xlabel("Time [s]")
    axs[4].legend(loc="upper right")
    axs[4].grid(True)

    fig.suptitle(f"{title_prefix} Sensor Suite")
    plt.tight_layout()
    plt.show()
