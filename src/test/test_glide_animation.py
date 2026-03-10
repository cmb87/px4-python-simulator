import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from vehicle.parameters import Parameters
from vehicle.quaternion import Quaternion
from vehicle.world import World
from vehicle.vehicles.x8.forces import WingX8ForceModel


def build_initial_state():
    y0 = np.zeros(13)
    y0[0:3] = np.array([0.0, 0.0, -80.0])
    y0[3:7] = Quaternion.euler2quat(np.deg2rad(np.array([0.0, -3.0, 0.0])))
    y0[7:10] = np.array([0.0, 0.0, 0.0])
    y0[10:13] = np.array([0.0, 0.0, 0.0])
    return y0


def run_glide_sim(total_time_s=30.0, dt_s=0.01):
    p = Parameters()
    p.left_rail = True

    y0 = build_initial_state()
    u0 = np.zeros(4)
    wind0 = np.zeros(6)

    world = World(p, y0=y0, u0=u0, wind0=wind0, force_models=[WingX8ForceModel()])

    steps = int(total_time_s / dt_s)
    dt_us = int(dt_s * 1e6)

    y_hist = np.zeros((steps, 13))
    t_hist = np.zeros(steps)

    for k in range(steps):
        t_us = (k + 1) * dt_us
        out = world.update(t_us, paused=False)
        y_hist[k] = out["y"]
        t_hist[k] = t_us / 1e6

    return t_hist, y_hist


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


def cuboid_faces(center_body, size_body):
    cx, cy, cz = center_body
    lx, ly, lz = size_body
    hx, hy, hz = 0.5 * lx, 0.5 * ly, 0.5 * lz

    v = np.array([
        [cx - hx, cy - hy, cz - hz],
        [cx + hx, cy - hy, cz - hz],
        [cx + hx, cy + hy, cz - hz],
        [cx - hx, cy + hy, cz - hz],
        [cx - hx, cy - hy, cz + hz],
        [cx + hx, cy - hy, cz + hz],
        [cx + hx, cy + hy, cz + hz],
        [cx - hx, cy + hy, cz + hz],
    ])

    return np.array([
        [v[0], v[1], v[2], v[3]],
        [v[4], v[5], v[6], v[7]],
        [v[0], v[1], v[5], v[4]],
        [v[2], v[3], v[7], v[6]],
        [v[1], v[2], v[6], v[5]],
        [v[0], v[3], v[7], v[4]],
    ])


def build_simple_airframe_faces(scale=1.0):
    fuselage = cuboid_faces(center_body=np.array([0.0, 0.0, 0.0]), size_body=np.array([2.2, 0.25, 0.25]))
    wing = cuboid_faces(center_body=np.array([0.1, 0.0, 0.0]), size_body=np.array([0.9, 3.4, 0.06]))
    h_tail = cuboid_faces(center_body=np.array([-0.9, 0.0, 0.0]), size_body=np.array([0.5, 1.1, 0.05]))
    v_tail = cuboid_faces(center_body=np.array([-0.9, 0.0, 0.22]), size_body=np.array([0.35, 0.05, 0.45]))
    return np.concatenate([fuselage, wing, h_tail, v_tail], axis=0) * scale


def animate_glide(t_hist, y_hist):
    pos_ned = y_hist[:, 0:3]
    quat = y_hist[:, 3:7]

    north = pos_ned[:, 0]
    east = pos_ned[:, 1]
    up = -pos_ned[:, 2]

    mesh_faces_body = build_simple_airframe_faces(scale=3.0)

    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection="3d")

    body_lines = {
        "x": ax.plot([], [], [], "r-", lw=2)[0],
        "y": ax.plot([], [], [], "g-", lw=2)[0],
        "z": ax.plot([], [], [], "b-", lw=2)[0],
    }
    trajectory_line = ax.plot([], [], [], "k--", lw=1.5)[0]
    mesh_poly = Poly3DCollection([], facecolor="lightgray", edgecolor="k", linewidth=0.2, alpha=0.8)
    ax.add_collection3d(mesh_poly)

    margin = 5.0
    ax.set_xlim(np.min(north) - margin, np.max(north) + margin)
    ax.set_ylim(np.min(east) - margin, np.max(east) + margin)
    ax.set_zlim(max(0.0, np.min(up) - margin), np.max(up) + margin)

    ax.set_xlabel("North [m]")
    ax.set_ylabel("East [m]")
    ax.set_zlabel("Up [m]")
    ax.set_title("Unpowered Glide (Wing Aerodynamics Only)")
    ax.view_init(elev=20, azim=-120)
    set_axes_equal(ax)

    axis_scale = 4.0
    follow_half_range = 18.0

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
            x_vals = [origin[0], end_ned[0]]
            y_vals = [origin[1], end_ned[1]]
            z_vals = [origin[2], -end_ned[2]]
            body_lines[axis_name].set_data(x_vals, y_vals)
            body_lines[axis_name].set_3d_properties(z_vals)

        mesh_flat = mesh_faces_body.reshape(-1, 3).T
        mesh_rot_ned = (mgf @ mesh_flat).T.reshape(-1, 4, 3)
        mesh_trans_ned = mesh_rot_ned + p_ned.reshape(1, 1, 3)
        mesh_plot = mesh_trans_ned.copy()
        mesh_plot[:, :, 2] = -mesh_plot[:, :, 2]
        mesh_poly.set_verts(mesh_plot)

        trajectory_line.set_data(north[: frame_idx + 1], east[: frame_idx + 1])
        trajectory_line.set_3d_properties(up[: frame_idx + 1])

        ax.set_xlim(origin[0] - follow_half_range, origin[0] + follow_half_range)
        ax.set_ylim(origin[1] - follow_half_range, origin[1] + follow_half_range)
        ax.set_zlim(origin[2] - follow_half_range, origin[2] + follow_half_range)

        ax.set_title(f"Unpowered Glide | t = {t_hist[frame_idx]:.1f} s")
        return list(body_lines.values()) + [trajectory_line, mesh_poly]

    ani = FuncAnimation(fig, update, frames=len(t_hist), interval=20, blit=False)
    _ = ani
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    t_history, y_history = run_glide_sim(total_time_s=30.0, dt_s=0.01)
    animate_glide(t_history, y_history)
