import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from pid import PID
# -------------------------------------------------------------
# System parameters
# -------------------------------------------------------------
m = 1.0
J = 0.01
g = 9.81
lever_arm = 0.05
T = 0.1


pid_y =     PID(8, 2, 4, 20)
pid_theta = PID(20, 5, 1, 1.0) # starting to oscilate
pid_theta = PID(20, 5, 4, 1.0)

pid_theta = PID(20, 1, 0, 1.0)

# -------------------------------------------------------------
# Simulation state
# -------------------------------------------------------------
x = 0.0
y = -0.5
theta = 0.2

dx = dy = dtheta = 0.0

Tmag = 0.0
Tangle = 0.0

dt = 0.002
t_end = 6.0
steps = int(t_end / dt)

state = []

# -------------------------------------------------------------
# Simulation loop
# -------------------------------------------------------------
for i in range(steps):
    # --- control ---
    y_err = 0 - y
    th_err = 0 - theta

    Tmag_cmd = m*g + pid_y.update(y_err, dt)
    Tangle_cmd = pid_theta.update(th_err, dt)

    # --- actuator dynamics ---
    Tmag += (Tmag_cmd - Tmag) * dt / T
    Tangle += (Tangle_cmd - Tangle) * dt / T

    # --- forces ---
    Tx = Tmag * np.sin(Tangle)
    Ty = Tmag * np.cos(Tangle)

    ddx = Tx / m
    ddy = Ty / m - g
    torque = lever_arm * Tx
    ddtheta = torque / J

    # integrate
    dx += ddx * dt
    dy += ddy * dt
    dtheta += ddtheta * dt

    x += dx * dt
    y += dy * dt
    theta += dtheta * dt

    state.append([x, y, theta, Tmag, Tangle])

state = np.array(state)

# -------------------------------------------------------------
# Animation setup
# -------------------------------------------------------------
fig, ax = plt.subplots(figsize=(6,6))
ax.set_xlim(-2.4, 2.4)
ax.set_ylim(-2.0, 2.4)
ax.set_aspect('equal')
ax.grid(True)

# CG point
cg_point, = ax.plot([], [], 'ko', markersize=6)

# Line from CG to thrust point
cg_to_thrust, = ax.plot([], [], 'b-', lw=2)

# Thrust vector line
thrust_line, = ax.plot([], [], 'r-', lw=2)

def update(frame):
    x, y, th, Tmag, Tangle = state[frame]
    t = frame * dt

    # CG point
    cg_point.set_data([x], [y])

    # Thrust point location (in world frame)
    tx = x
    ty = y - lever_arm

    # Draw line from CG to thrust location
    cg_to_thrust.set_data([x, tx], [y, ty])

    # Thrust vector direction
    scale = -0.05
    tx2 = tx + scale * Tmag * np.sin(Tangle)
    ty2 = ty + scale * Tmag * np.cos(Tangle)

    thrust_line.set_data([tx, tx2], [ty, ty2])

    # Add **time in title**
    ax.set_title(f"2D Hover Simulation   t = {t:4.2f} s")

    return cg_point, cg_to_thrust, thrust_line

ani = FuncAnimation(fig, update, frames=len(state), interval=10, blit=True)

plt.show()