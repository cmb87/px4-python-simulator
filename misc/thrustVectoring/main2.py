import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ----------------------------
# Parameters
# ----------------------------

dt = 0.001
t_end = 10.0
steps = int(t_end / dt)

# Physical parameters
g = 9.81
m = 1.0
l = 0.2
I = 0.01
T = 15.0

# Servo parameters
servo_tau = 0.6            # first-order time constant (s)
servo_speed = 8.05           # rad/s (0.13s / 60deg)
servo_limit = np.deg2rad(15)

# Controller gains
ATT_P = .01
RATE_P = 0.15
RATE_I = 0.0
RATE_D = 0.01


# Controller gains
RATE_K = 10
ATT_P = 4.0
RATE_P = RATE_K*0.6
RATE_I = RATE_K*0.0
RATE_D = RATE_K*0.1

# ----------------------------
# State variables
# ----------------------------

theta = np.deg2rad(45.0)
omega = 0.0
delta = 0.0                  # actual servo output

rate_i = 0.0
prev_rate_err = 0.0

theta_log = np.zeros(steps)
delta_log = np.zeros(steps)

# ----------------------------
# Simulation
# ----------------------------

for i in range(steps):

    # Attitude controller (P only)
    rate_sp = -ATT_P * theta

    # Rate controller (PID)
    rate_err = rate_sp - omega
    rate_i += rate_err * dt
    rate_d = (rate_err - prev_rate_err) / dt
    prev_rate_err = rate_err

    delta_cmd = (
        RATE_P * rate_err
        + RATE_I * rate_i
        + RATE_D * rate_d
    )

    #delta_cmd = np.clip(delta_cmd, -servo_limit, servo_limit)

    # -------- Servo dynamics --------
    # First-order response
    delta_dot = (delta_cmd - delta) / servo_tau

    # Rate limit
    delta_dot = np.clip(delta_dot, -servo_speed, servo_speed)

    delta += delta_dot * dt

    # -------- Vehicle dynamics --------
    gravity_torque = m * g * l * np.sin(theta)
    control_torque = T * l * np.sin(delta)

    theta_ddot = (control_torque - gravity_torque) / I

    omega += theta_ddot * dt
    theta += omega * dt

    theta_log[i] = theta
    delta_log[i] = delta

# ----------------------------
# Animation
# ----------------------------

fig, ax = plt.subplots(figsize=(6,6))
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.5, 0.5)
ax.set_aspect('equal')
ax.grid()

# World vertical
ax.plot([0, 0], [-0.4, 0.4], 'k--')

body_line, = ax.plot([], [], 'b-', lw=4)
thrust_line, = ax.plot([], [], 'r-', lw=2)

body_len = 0.3
thrust_len = -0.25

def update(frame):
    theta = theta_log[frame]
    delta = delta_log[frame]

    # Body
    x_body = [0, body_len * np.sin(theta)]
    y_body = [0, body_len * np.cos(theta)]
    body_line.set_data(x_body, y_body)

    # Thrust vector
    total_angle = theta + delta
    x_thrust = [0, thrust_len * np.sin(total_angle)]
    y_thrust = [0, thrust_len * np.cos(total_angle)]
    thrust_line.set_data(x_thrust, y_thrust)

    return body_line, thrust_line

ani = animation.FuncAnimation(
    fig,
    update,
    frames=steps,
    interval=dt * 1000,
    blit=True
)

plt.show()
