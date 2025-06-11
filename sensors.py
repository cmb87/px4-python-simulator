import numpy as np
from quaternion import Quaternion

def sensors(t, y, u, wind, P):

    pos = y[0:3]
    quat = y[3:7] / np.linalg.norm(y[3:7])
    vel = y[7:10]     # body frame
    omega = y[10:13]  # body frame

    # === Rotation matrix: NED to body
    Mfg = Quaternion.Mfg(quat)  # Maps NED -> Body
    Mgf = Mfg.T                 # Body -> NED

    # === Gravity in body frame
    gravity_ned = np.array([0, 0, P.mass * P.gravity])
    gravity_body = Mfg @ gravity_ned

    # === Accelerometer: measures specific force
    a_spec = (1 / P.mass) * gravity_body - np.cross(omega, vel)

    # === Gyroscope: measures angular velocity + bias + noise
    gyro_meas = omega + P.gyro_bias + np.random.normal(0, P.gyro_noise_std, 3)

    # === Magnetometer: get magnetic field in body frame
    magnetic_ned = P.magnetic_ned
    magnetic_body = Mfg @ magnetic_ned
    mag_meas = magnetic_body + P.mag_bias + np.random.normal(0, P.mag_noise_std, 3)

    # === Accelerometer measurement (specific force)
    acc_meas = a_spec + P.accel_bias + np.random.normal(0, P.accel_noise_std, 3)

    # === Barometer: altitude from NED z + noise
    baro_meas = pos[2] + P.baro_bias + np.random.normal(0, P.baro_noise_std)

    return {
        'accelerometer': acc_meas,
        'gyroscope': gyro_meas,
        'magnetometer': mag_meas,
        'barometer': baro_meas,
    }