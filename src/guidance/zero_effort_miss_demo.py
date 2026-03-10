#!/usr/bin/env python3
"""Zero-Effort-Miss guidance demo with 2D animation.

Target is stationary. Interceptor uses a simple ZEM-based lateral acceleration command.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def simulate_zero_effort_miss(
    dt_s: float = 0.02,
    total_time_s: float = 25.0,
    zem_gain: float = 4.0,
    accel_max_mps2: float = 25.0,
    min_time_to_go_s: float = 0.25,
    intercept_radius_m: float = 1.0,
):
    # 2D North-East plane
    interceptor_pos = np.array([-800.0, -500.0], dtype=float)
    interceptor_vel = np.array([90.0, 28.0], dtype=float)

    # Stationary target
    target_pos = np.array([0.0, 0.0], dtype=float)
    target_vel = np.array([0.0, 0.0], dtype=float)

    steps = int(total_time_s / dt_s)

    t_hist = np.zeros(steps, dtype=float)
    interceptor_pos_hist = np.zeros((steps, 2), dtype=float)
    interceptor_vel_hist = np.zeros((steps, 2), dtype=float)
    target_pos_hist = np.zeros((steps, 2), dtype=float)
    range_hist = np.zeros(steps, dtype=float)
    zem_hist = np.zeros((steps, 2), dtype=float)

    hit_index = steps - 1

    for k in range(steps):
        t_s = k * dt_s

        rel_pos = target_pos - interceptor_pos
        rel_vel = target_vel - interceptor_vel

        distance = float(np.linalg.norm(rel_pos))
        speed_rel = float(np.linalg.norm(rel_vel))
        t_go = max(distance / max(speed_rel, 1e-6), min_time_to_go_s)

        # Zero-Effort-Miss estimate at time-to-go.
        zem = rel_pos + rel_vel * t_go

        # Use only lateral ZEM component (perpendicular to LOS) for guidance.
        los_hat = rel_pos / max(distance, 1e-9)
        zem_lateral = zem - np.dot(zem, los_hat) * los_hat

        accel_cmd = zem_gain * zem_lateral / max(t_go * t_go, 1e-9)
        accel_norm = float(np.linalg.norm(accel_cmd))
        if accel_norm > accel_max_mps2:
            accel_cmd *= accel_max_mps2 / accel_norm

        interceptor_vel = interceptor_vel + accel_cmd * dt_s
        interceptor_pos = interceptor_pos + interceptor_vel * dt_s

        t_hist[k] = t_s
        interceptor_pos_hist[k] = interceptor_pos
        interceptor_vel_hist[k] = interceptor_vel
        target_pos_hist[k] = target_pos
        range_hist[k] = distance
        zem_hist[k] = zem

        if distance <= intercept_radius_m:
            hit_index = k
            break

    last = hit_index + 1
    return {
        "t_s": t_hist[:last],
        "interceptor_pos": interceptor_pos_hist[:last],
        "interceptor_vel": interceptor_vel_hist[:last],
        "target_pos": target_pos_hist[:last],
        "range_m": range_hist[:last],
        "zem": zem_hist[:last],
    }


def animate_result(result: dict[str, np.ndarray]) -> None:
    t_s = result["t_s"]
    interceptor_pos = result["interceptor_pos"]
    target_pos = result["target_pos"]
    range_m = result["range_m"]
    zem = result["zem"]

    interceptor_n = interceptor_pos[:, 0]
    interceptor_e = interceptor_pos[:, 1]
    target_n = target_pos[:, 0]
    target_e = target_pos[:, 1]

    n_all = np.concatenate([interceptor_n, target_n])
    e_all = np.concatenate([interceptor_e, target_e])
    margin = 80.0

    fig, ax = plt.subplots(figsize=(9, 7))

    trajectory_line = ax.plot([], [], "b-", lw=2.0, label="Interceptor trajectory")[0]
    interceptor_marker = ax.plot([], [], "bo", ms=8, label="Interceptor")[0]
    target_marker = ax.plot(target_n[0], target_e[0], "r*", ms=14, label="Stationary target")[0]
    los_line = ax.plot([], [], "k--", lw=1.2, alpha=0.7, label="Line of sight")[0]

    info_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "0.8"},
    )

    ax.set_xlim(float(np.min(n_all) - margin), float(np.max(n_all) + margin))
    ax.set_ylim(float(np.min(e_all) - margin), float(np.max(e_all) + margin))
    ax.set_xlabel("North [m]")
    ax.set_ylabel("East [m]")
    ax.set_title("Zero-Effort-Miss Guidance Demo (Stationary Target)")
    ax.grid(True, alpha=0.35)
    ax.set_aspect("equal", adjustable="box")
    ax.legend(loc="lower right")

    def update(frame_idx: int):
        n = interceptor_n[: frame_idx + 1]
        e = interceptor_e[: frame_idx + 1]

        trajectory_line.set_data(n, e)
        interceptor_marker.set_data([interceptor_n[frame_idx]], [interceptor_e[frame_idx]])
        los_line.set_data(
            [interceptor_n[frame_idx], target_n[frame_idx]],
            [interceptor_e[frame_idx], target_e[frame_idx]],
        )

        zem_norm = float(np.linalg.norm(zem[frame_idx]))
        info_text.set_text(
            f"t = {t_s[frame_idx]:.2f} s\n"
            f"range = {range_m[frame_idx]:.2f} m\n"
            f"|ZEM| = {zem_norm:.2f} m"
        )
        return trajectory_line, interceptor_marker, target_marker, los_line, info_text

    ani = FuncAnimation(fig, update, frames=len(t_s), interval=20, blit=False)
    _ = ani
    plt.tight_layout()
    plt.show()


def main() -> None:
    result = simulate_zero_effort_miss()
    miss_distance = float(result["range_m"][-1])
    print(f"Final miss distance: {miss_distance:.3f} m")
    animate_result(result)


if __name__ == "__main__":
    main()
