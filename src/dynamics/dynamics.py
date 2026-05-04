import numpy as np
import sys
import logging

from vehicles.base_component import SimComponentBase
from .quaternion import Quaternion


logger = logging.getLogger(__name__)

def MOmega(OMEGAkf):
    p, q, r = OMEGAkf[0], OMEGAkf[1], OMEGAkf[2]
    M = [[0, -p, -q, -r], [p, 0, r, -q], [q, -r, 0, p], [r, q, -p, 0]]
    return np.asarray(M)


class Dynamics6DOF(SimComponentBase):
    def __init__(self, z_ground=100.0):
        super().__init__()
        self.z_ground = z_ground
        self._ground_contact_latched = False

    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        P = self._inputs.get("P")
        tau = self._inputs.get("tau")

        if y is None or P is None or tau is None:
            raise ValueError("Dynamics6DOF requires inputs: y, P, tau")

        t_s = float(t_us) / 1e6
        self.last_output = dynamics(t_s, y, P, tau, z_ground=self.z_ground)

        z_ned = float(y[2])
        on_ground = z_ned >= 0.0
        if on_ground and not self._ground_contact_latched:
            logger.info("Ground contact at t=%.3fs, z_ned=%.3f m", t_s, z_ned)
            self._ground_contact_latched = True
        elif (not on_ground) and self._ground_contact_latched:
            self._ground_contact_latched = False

        self._last_t_us = int(t_us)
        return self.last_output



def dynamics(t, y, P, tau, z_ground = 100.0):
    """
    Computes the time derivative of the state vector y.
    
    Parameters:
    - t: time (not used directly but required for ODE solvers)
    - y: 13-element state vector (pos, quaternions, vel, Omega)
    - P: parameters object with mass, I_cg, r_cg, M_rb
    - tau: external force and torque vector (6x1)
    
    Returns:
    - xdot: time derivative of state vector
    """
    pos = y[0:3]            # ned
    quaternions = y[3:7]
    vel = y[7:10]           # in Body frame
    Omega = y[10:13]        # Rates

    quaternions =  quaternions / np.linalg.norm(quaternions)  # Normalize quaternions


    # Switch to full 6DOF mode if not already done
    
    Mfg = Quaternion.Mfg(quaternions)
    Mgf = Mfg.T
    Momega = MOmega(Omega)


    # === Transform velocity to NED frame
    vel_ned = Mgf @ vel

    # === Compute gravity in body frame
    gravity = np.matmul(Mfg, np.array([0, 0, P.mass * P.gravity]))
  #  gravity = np.dot(Mfg, np.array([0, 0, 0]))


    # === Acceleration in body frame
    body_force = tau[0:3] + gravity
    accelBf = body_force / P.mass - np.cross(Omega, vel)

    # === Convert body acceleration to NED frame
    accel_ned = Mgf @ accelBf

    # === Simple ground contact condition at z_ned = 0.0
    if pos[2] >= 0.0:
        # Do not allow downward acceleration/velocity while on ground.
        if accel_ned[2] > 0.0:
            accel_ned[2] = 0.0
            accelBf = Mfg @ accel_ned

        if vel_ned[2] > 0.0:
            vel_ned[2] = 0.0
            vel = Mfg @ vel_ned

        pos[2] = 0.0

    # ===  Angular Acceleration
    aux1 = np.matmul(P.I_cg, Omega)
    aux2 =  tau[3:] - np.cross(Omega, aux1)
    rotAccelBf = np.matmul(P.I_cg_inv, aux2)


    # ===  Drehung
    quaternionsDot = 0.5 * np.matmul(Momega, quaternions)

  #  print( tau[3:], Omega, quaternionsDot, rotAccelBf)

    return np.concatenate((vel_ned, quaternionsDot, accelBf, rotAccelBf))


def rail_dynamics(t, y, P, tau):
    _ = t

    quaternions = np.asarray(y[3:7], dtype=float)
    vel = np.asarray(y[7:10], dtype=float)

    q_norm = float(np.linalg.norm(quaternions))
    if q_norm <= 1e-9:
        quaternions = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    else:
        quaternions = quaternions / q_norm

    mfg = Quaternion.Mfg(quaternions)
    mgf = mfg.T

    rail_dir_ned = np.asarray(P.rail_dir_ned, dtype=float)
    rail_dir_norm = float(np.linalg.norm(rail_dir_ned))
    if rail_dir_norm <= 1e-9:
        raise ValueError("Rail direction vector must be non-zero")
    rail_dir_ned = rail_dir_ned / rail_dir_norm

    vel_ned = mgf @ vel
    vel_rail_ned = np.dot(vel_ned, rail_dir_ned) * rail_dir_ned

    gravity_body = mfg @ np.array([0.0, 0.0, float(P.mass) * float(P.gravity)], dtype=float)
    body_force = np.asarray(tau[0:3], dtype=float) + gravity_body
    accel_body = body_force / float(P.mass)

    rail_dir_body = mfg @ rail_dir_ned
    accel_body = np.dot(accel_body, rail_dir_body) * rail_dir_body

    quaternions_dot = np.zeros(4)
    rot_accel_body = np.zeros(3)

    pos = np.asarray(y[0:3], dtype=float)
    rail_start_ned = np.asarray(P.rail_start_ned, dtype=float)
    rail_dist = float(np.dot(pos - rail_start_ned, rail_dir_ned))
    if rail_dist >= float(P.rail_length):
        P.left_rail = True

    return np.concatenate((vel_rail_ned, quaternions_dot, accel_body, rot_accel_body))






if __name__ == "__main__":

    import matplotlib.pyplot as plt

    from vehicles.common_forces.catapult_rail import railForces
    from vehicles.x8.forces import forces
    from vehicles.parameters import Parameters
    from sensors.sensors import sensors

    P = Parameters()  # Load parameters, assuming this is defined in parameters.py

    railPitch = np.deg2rad(0.0)
    railYaw = np.deg2rad(0.0)

    quatRail = Quaternion.euler2quat(np.asarray([0.0,railPitch,railYaw]))
    P.rail_dir_ned = Quaternion.Mfg(quatRail).T @ np.asarray([1,0,0])


    # Example usage
    t = 0.0
    y = np.zeros(13)  # Initial state


    y[7] = 0.0 # Start speed in body dir
    y[0:3] = P.rail_start_ned
    y[3:7] = quatRail

    u = np.zeros(4)  # Example control inputs
    u[3] = 0.78
    u[1] = 0.00


    wind = np.zeros(6)  # Example wind vector


    dt= 10000.0 / 1e6


    yall = []
    zall = []
    tall = []


    for n in range(8000):


        tau = forces(t, y, u, wind, P)

        if P.left_rail:
            
            ydot = dynamics(t, y, P, tau)
            
        else:
            tauRail = railForces(t, y, u, wind, P)

            ydot = rail_dynamics(t, y, P, tau + tauRail)
        

        z = sensors(t, y, ydot, u, wind, P, dt)

     #   print(z["accelerometer"])
        zall.append(z["euler"])  # Store the sensor data for later analysis
        yall.append(y.copy())  # Store the state for later analysis
        tall.append(t)

        y += ydot * dt  # Update state using Euler method
        t += dt
        #print(f"Time: {t:.2f}, State: {np.around(y,2)}")

        



    yall = np.asarray(yall)
    zall = np.asarray(zall)
    tall = np.asarray(tall)

    plt.plot(tall, zall[:,0], label='Accelerometer X')
    plt.plot(tall, zall[:,1], label='Accelerometer Y')
    plt.plot(tall, zall[:,2], label='Accelerometer Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Accelerometer Reading (m/s²)')
    plt.grid(True)
    plt.title('Accelerometer Readings Over Time')
    plt.legend()
    plt.show()




    plt.plot(yall[:, 0], -yall[:, 2], label='Position (NED)')
    plt.xlabel('North (m)')
    plt.ylabel('Altitude (m)')
    plt.axis("equal")
    plt.grid(True)
    plt.title('Quadcopter Position in NED Frame')
    plt.legend()
    plt.show()

    plt.plot(tall, np.linalg.norm( yall[:,7:10],axis=1 ), label='Position (NED)')
    plt.xlabel('Time')
    plt.ylabel('Speed')
    plt.grid(True)
    plt.title('Quadcopter Position in NED Frame')
    plt.legend()
    plt.show()


    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from mpl_toolkits.mplot3d import Axes3D
    from scipy.spatial.transform import Rotation as R

    def animate_6dof(time, y):
        pos = y[:, 0:3]
        quats = y[:, 3:7]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        

        # Initial 3D object: simple body frame (x-red, y-green, z-blue)
        scale = 0.5
        body_lines = {
            'x': ax.plot([], [], [], 'r-', lw=2)[0],
            'y': ax.plot([], [], [], 'g-', lw=2)[0],
            'z': ax.plot([], [], [], 'b-', lw=2)[0],
        }
        trajectory_line, = ax.plot([], [], [], 'k--', lw=1)

        ax.set_xlim([-15, 15])
        ax.set_ylim([-15, 15])
        ax.set_zlim([-30, 0])
        ax.set_xlabel('North')
        ax.set_ylabel('East')
        ax.set_zlabel('Down')
        ax.view_init(elev=-16, azim=-115, roll=-177)

        def update(frame):
            p = pos[frame]
            q = quats[frame] / np.linalg.norm(quats[frame])
            rot = R.from_quat([q[1], q[2], q[3], q[0]])  # scipy uses [x, y, z, w]

            # Body axes in local frame
            axes = np.eye(3) * scale
            rotated_axes = rot.apply(axes)

            origin = p.reshape((3, 1))
            ends = origin + rotated_axes.T

            for i, axis in enumerate(['x', 'y', 'z']):
                x_vals = [origin[0, 0], ends[0, i]]
                y_vals = [origin[1, 0], ends[1, i]]
                z_vals = [origin[2, 0], ends[2, i]]

                body_lines[axis].set_data(x_vals, y_vals)
                body_lines[axis].set_3d_properties(z_vals)

            trajectory_line.set_data(pos[:frame, 0], pos[:frame, 1])
            trajectory_line.set_3d_properties(pos[:frame, 2])

            return list(body_lines.values()) + [trajectory_line]

        ani = FuncAnimation(fig, update, frames=len(time), interval=40, blit=False)
        plt.show()



    animate_6dof(tall, yall)
