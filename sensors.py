import numpy as np
from quaternion import Quaternion

R_E = 6378137.0  # Earth radius [m]

def sensors(t, y, ydot, u, wind, P):

    pos = y[0:3]
    quat = y[3:7] / np.linalg.norm(y[3:7])
    vel = y[7:10]     # body frame
    omega = y[10:13]  # body frame
    accel = ydot[7:10] # body frame


    # === Rotation matrix: NED to body
    Mfg = Quaternion.Mfg(quat)  # Maps NED -> Body
    Mgf = Mfg.T                 # Body -> NED

    # === Velocity in NED
    vel_ned = Mgf @ vel

    # === Gravity in body frame
    gravity_ned = np.array([0, 0, P.gravity])
  

    # === Accelerometer: measures specific force

  
   # a_spec = + accel -  Mfg @ gravity_ned
    a_spec = np.asarray([accel[0], -accel[1], -accel[2]]) -  Mfg @ gravity_ned

    # === Accelerometer measurement (specific force)
    acc_meas = a_spec + P.accel_bias + np.random.normal(0, P.accel_noise_std, 3)

    # === Gyroscope: measures angular velocity + bias + noise
   # gyro_meas = omega + P.gyro_bias + np.random.normal(0, P.gyro_noise_std, 3)
    gyro_meas = np.asarray([omega[0], -omega[1], -omega[2]]) + P.gyro_bias + np.random.normal(0, P.gyro_noise_std, 3)



    # === Magnetometer: get magnetic field in body frame
    magnetic_ned = P.magnetic_ned
    magnetic_body = Mfg @ magnetic_ned
    mag_meas = np.asarray([magnetic_body[0], magnetic_body[1], magnetic_body[2]]) + P.mag_bias + np.random.normal(0, P.mag_noise_std, 3)
    #mag_meas = magnetic_body + P.mag_bias + np.random.normal(0, P.mag_noise_std, 3)

    # === Barometer: altitude from NED z + noise
    baro_meas = P.rho*P.gravity*pos[2] + P.baro_bias + np.random.normal(0, P.baro_noise_std)

    # ===  GPS: convert NED → LLA using flat Earth approximation
    
    lat0 = np.deg2rad(P.gps_origin['lat'])
    lon0 = np.deg2rad(P.gps_origin['lon'])

    d_north, d_east, d_down = pos + np.random.normal(0, P.gps_pos_noise_std, 3)

    vel_ned_gps = vel_ned + np.random.normal(0, P.gps_vel_noise_std, 3) 

    delta_lat = d_north / R_E
    delta_lon = d_east / (R_E * np.cos(lat0))

    lat = np.rad2deg(lat0 + delta_lat)
    lon = np.rad2deg(lon0 + delta_lon)
    alt = P.gps_origin['alt'] - d_down

    cog_rad = np.arctan2(vel_ned[1], vel_ned[0])  # Note: atan2(East, North)
    cog_deg = np.degrees(cog_rad) % 360  # Wrap to [0, 360)


    gps_meas = np.array([lat, lon, alt,vel_ned_gps[0], vel_ned_gps[1],vel_ned_gps[2], cog_deg]) 


    return {
        'accelerometer': acc_meas,
        'gyroscope': gyro_meas,
        'magnetometer': mag_meas,
        'barometer': baro_meas,
        'gps': gps_meas,  # [lat, lon, alt]
    }