import numpy as np
from quaternion import Quaternion


def MOmega(OMEGAkf):
    p, q, r = OMEGAkf[0], OMEGAkf[1], OMEGAkf[2]
    M = [[0, -p, -q, -r], [p, 0, r, -q], [q, -r, 0, p], [r, q, -p, 0]]
    return np.asarray(M)


def dynamics(t, y, P, tau):
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

    quaternions = quaternions/ np.linalg.norm(quaternions)  # Normalize quaternions


    Mfg = Quaternion.Mfg(quaternions)

    Mgf = Mfg.T
    Momega = MOmega(Omega)


    # === Transform velocity to NED frame
    vel_ned = Mgf @ vel

    # === Compute gravity in body frame
    gravityForceBf = np.dot(Mfg, np.array([0, 0, P.mass * P.gravity]))

    # === Acceleration in body frame
    body_force = tau[0:3] + gravityForceBf
    accelBf = body_force / P.mass - np.cross(Omega, vel)

    # === Convert body acceleration to NED frame
    accel_ned = Mgf @ accelBf

    # === Simple ground contact condition
    z_ground = 0.0
    if pos[2] >= z_ground:
        print("Ground contact detected, setting vertical velocity and acceleration to zero.")
        if accel_ned[2] > 0:  # Trying to fall
            accel_ned[2] = 0.0
            accelBf = Mfg @ accel_ned  # Reproject back to body frame

        if vel_ned[2] > 0:  # Currently falling
            vel_ned[2] = 0.0
            vel = Mfg @ vel_ned  # Reproject back to body frame

        pos[2] = z_ground

    # ===  Angular Acceleration
    aux1 = np.dot(P.I_cg, Omega)
    aux2 =  tau[3:] - np.cross(Omega, aux1)
    rotAccelBf = np.dot(P.I_cg_inv, aux2)


    # ===  Position
    vel_ned = np.matmul(Mgf, vel) # Erdfest ned

    # ===  Drehung
    quaternionsDot = 0.5 * np.dot(Momega, quaternions)

    return np.concatenate((vel_ned, quaternionsDot, accelBf, rotAccelBf))



if __name__ == "__main__":

    import matplotlib.pyplot as plt

    from parameters import Parameters
    from forces import forces
    from sensors import sensors

    P = Parameters()  # Load parameters, assuming this is defined in parameters.py

    # Example usage
    t = 0.0
    y = np.zeros(13)  # Initial state
    y[3] = 1.0
    y[2] = -100.0
    y[7] = 20.0
    u = np.zeros(4)  # Example control inputs
    wind = np.zeros(6)  # Example wind vector
    dt = 0.01  # Time step


    yall = []
    zall = []
    tall = []

    for n in range(6000):

        tau = forces(t, y, u, wind, P)
        z = sensors(t, y, u, wind, P)
        xdot = dynamics(t, y, P, tau)
        

        print(z["gps"])
        zall.append(z["gps"])  # Store the sensor data for later analysis
        yall.append(y.copy())  # Store the state for later analysis
        tall.append(t)

        y += xdot * dt  # Update state using Euler method
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
    plt.title('Accelerometer Readings Over Time')
    plt.legend()
    plt.show()

    print(yall.shape    )
    plt.plot(yall[:, 0], yall[:, 2], label='Position (NED)')
    plt.xlabel('North (m)')
    plt.ylabel('East (m)')
    plt.title('Quadcopter Position in NED Frame')
    plt.legend()
    plt.show()

    plt.plot(yall[:, 0], yall[:, 7], label='Position (NED)')
    plt.xlabel('North (m)')
    plt.ylabel('East (m)')
    plt.title('Quadcopter Position in NED Frame')
    plt.legend()
    plt.show()