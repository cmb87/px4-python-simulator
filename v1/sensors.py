import numpy as np
from quaternion import Quaternion
from random import random
from magnetometer import MagnetometerSim
from imu import ADIS16448IMU
from gps import GpsSensor
from barometer import BarometerSensor

MAG = MagnetometerSim()
IMU = ADIS16448IMU()

GPS = GpsSensor()
GPS.set_home(47.397742, 8.545594, 488.0)
GPS.set_noise(True)
GPS.set_update_rate(10.0)

BARO = BarometerSensor()
BARO.set_update_rate(20.0)  # 20 Hz
BARO.set_drift_rate(0.05)   # 0.05 Pa/s drift
BARO.set_noise(True)


# ==================================================
def crandom():
    return random()-0.5


def ned2lla(ned, ref_pos):
    """
    Converts NED (North-East-Down) coordinates to LLA (Latitude, Longitude, Altitude)
    given a reference LLA position.

    Parameters:
    ned (numpy.ndarray): 3x1 array of NED coordinates [N; E; D] in meters
    ref_pos (numpy.ndarray): 3x1 array of reference LLA [latitude, longitude, altitude]
                             (latitude and longitude in degrees, altitude in meters)

    Returns:
    numpy.ndarray: 3x1 array of LLA [latitude, longitude, altitude] in degrees and meters
    """
    # WGS84 ellipsoid constants
    a = 6378137.0  # semi-major axis in meters
    e = 0.081819190842622  # first eccentricity

    # Convert reference position to radians and extract altitude
    ref_lat = np.deg2rad(ref_pos[0])
    ref_lon = np.deg2rad(ref_pos[1])
    ref_alt = ref_pos[2]

    # Radius of curvature in the prime vertical
    N = a / np.sqrt(1 - e**2 * np.sin(ref_lat)**2)

    # Reference ECEF coordinates
    ref_ecef = np.zeros(3)
    ref_ecef[0] = (N + ref_alt) * np.cos(ref_lat) * np.cos(ref_lon)
    ref_ecef[1] = (N + ref_alt) * np.cos(ref_lat) * np.sin(ref_lon)
    ref_ecef[2] = (N * (1 - e**2) + ref_alt) * np.sin(ref_lat)

    # Rotation matrix from NED to ECEF
    Rned2ecef = np.array([
        [-np.sin(ref_lat) * np.cos(ref_lon), -np.sin(ref_lon), -np.cos(ref_lat) * np.cos(ref_lon)],
        [-np.sin(ref_lat) * np.sin(ref_lon),  np.cos(ref_lon), -np.cos(ref_lat) * np.sin(ref_lon)],
        [ np.cos(ref_lat),                   0,               -np.sin(ref_lat)]
    ])

    # Convert NED to ECEF
    ecef_offset = Rned2ecef @ ned
    new_ecef = ref_ecef + ecef_offset

    # Extract ECEF coordinates
    x, y, z = new_ecef
    lon = np.arctan2(y, x)

    # Iterative latitude and altitude calculation
    p = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, p * (1 - e**2))
    for _ in range(5):
        N = a / np.sqrt(1 - e**2 * np.sin(lat)**2)
        alt = p / np.cos(lat) - N
        lat = np.arctan2(z + e**2 * N * np.sin(lat), p)

    # Convert to degrees
    lat_deg = np.rad2deg(lat)
    lon_deg = np.rad2deg(lon)

    # Normalize longitude to [-180, 180]
    if lon_deg > 180:
        lon_deg -= 360
    elif lon_deg < -180:
        lon_deg += 360

    return np.array([lat_deg, lon_deg, alt])




def sensors(t, y, ydot, u, wind, P, dt):

    pos = y[0:3]
    quat = y[3:7] / np.linalg.norm(y[3:7])
    vel = y[7:10]     # body frame
    omega = y[10:13]  # body frame

    accel = ydot[7:10] # body frame


    # === Rotation matrix: NED to body
    Mfg = Quaternion.Mfg(quat)  # Maps NED -> Body
    Mgf = Mfg.T                 # Body -> NED


    euler = np.rad2deg(Quaternion.quat2Euler(quat))

    # === Velocity in NED
    vel_ned = Mgf @ vel

    acc_meas, gyro_meas = IMU.update(accel, omega, quat)
    
    #print(acc_meas)

#     # === Gravity in body frame
#     gravity_ned = np.array([0, 0, P.gravity])
  

#     # === Accelerometer: measures specific force
#     # Dynamic ACCEL must be negative!
#     a_spec = np.asarray([accel[0], accel[1], accel[2]]) -  Mfg @ gravity_ned

#     # === Accelerometer measurement (specific force)
#     acc_meas = a_spec + P.accel_bias + np.random.normal(0, P.accel_noise_std, 3)

#     # === Gyroscope: measures angular velocity + bias + noise
#    # gyro_meas = omega + P.gyro_bias + np.random.normal(0, P.gyro_noise_std, 3)
#     gyro_meas = np.asarray([omega[0], omega[1], omega[2]]) + P.gyro_bias + np.random.normal(0, P.gyro_noise_std, 3)



    # === Magnetometer: get magnetic field in body frame

    result = MAG.simulate_step(quat)
    #print("--------")
    #print(result)


    mag_meas = result["mag_field_body_gauss"]

#    magnetic_ned = P.magnetic_ned
#    magnetic_body = Mfg @ magnetic_ned
#    mag_meas = np.asarray([magnetic_body[0], -magnetic_body[1], magnetic_body[2]]) + P.mag_bias + np.random.normal(0, 19*P.mag_noise_std, 3)


#     print()

#     lat, lon, alt = 47.3769, 8.5417, 0  # Zurich
#     mag = wmm(lat, lon, alt, yeardec = 2025.5 )


#     mag_ned = [mag['north'].item()/100000.0, mag['east'].item()/100000.0, mag['down'].item()/100000.0]  # ✅

#    # print(mag_ned)
#     r = R.from_quat(np.asarray([quat[1],quat[2],quat[3],quat[0] ]))  # Note the order: x, y, z, w for scipy

#     # Rotate NED magnetic vector to body frame
#     mag_ned_vec = np.array(mag_ned)
#     mag_meas = r.inv().apply(mag_ned_vec)  + P.mag_bias + np.random.normal(0, P.mag_noise_std, 3)


#     print(mag_meas, mag_meas0)

    #mag_meas = magnetic_body + P.mag_bias + np.random.normal(0, P.mag_noise_std, 3)

    # === Barometer: altitude from NED z + noise
    staticPressure = P.rho*P.gravity*-pos[2]
    dynamicPressure = 0.5 * P.rho * np.linalg.norm(vel_ned)**2

    baro_meas = {
        "static": staticPressure + P.baro_bias + np.random.normal(0, P.baro_noise_std),
        "dynamic": dynamicPressure + np.random.normal(0, P.baro_noise_std)
    }


    # reading = BARO.tick(-pos[2])
    # baro_meas = {
    #     "static": 100*reading["absolute_pressure_hpa"],
    #     "dynamic": dynamicPressure + np.random.normal(0, P.baro_noise_std)
    # }



    # ===  GPS: convert NED → LLA using flat Earth approximation

    # pos_noisy = pos + np.random.normal(0, P.gps_pos_noise_std, 3)

    # lat, lon, alt = ned2lla(pos_noisy, np.asarray([P.gps_origin['lat'], P.gps_origin['lon'], P.gps_origin['alt']]))
    # cog_rad = np.arctan2(vel_ned[1], vel_ned[0])  # Note: atan2(East, North)
    # cog_deg = np.degrees(cog_rad) 

    # vel_ned_gps = vel_ned + np.random.normal(0, P.gps_vel_noise_std, 3) 

    # gps_meas = np.array([lat, lon, alt, vel_ned_gps[0], vel_ned_gps[1],vel_ned_gps[2], cog_deg]) 


    data = GPS.tick(pos, vel_ned)

    gps_meas = np.array([
        data['latitude_deg'],
        data['longitude_deg'],
        data['altitude_m'],
        data['velocity_north'],
        data['velocity_east'],
        data['velocity_up'],  # GPS reports up as negative
        0.0
    ])

    #print( int(100*data['velocity_north']), int(100*data['velocity_east']), int(100*-data['velocity_up']) )

    return {
        'accelerometer': acc_meas,
        'gyroscope': gyro_meas,
        'magnetometer': mag_meas,
        'barometer': baro_meas,
        'gps': gps_meas,  # [lat, lon, alt]
        'euler': euler
    }



def Update(drone, t, y, ydot, z, u, wind, P, armed=False):
    
    pos = y[0:3]
    quat = y[3:7]
    vel = y[7:10]     # body frame
    omega = y[10:13]  # body frame
    accel = ydot[7:10]


    # HIL_GPS Data
    drone['i_lat__degE7']               = round((   z['gps'][0]     )*1e7)
    drone['i_lon__degE7']               = round((   z['gps'][1]     )*1e7)
    drone['i_alt__mm']                  = round((   z['gps'][2]     )*1000)
    drone['i_eph__cm']                  = round((   0.3         + random()*0.001    )*100)
    drone['i_epv__cm']                  = round((   0.4         + random()*0.001    )*100)
    drone['i_vel__cm/s']                = round((   np.linalg.norm(z['gps'][3:6])    )*100)
    drone['i_vn__cm/s']                 = round((   z['gps'][3]    )*100)
    drone['i_ve__cm/s']                 = round((   z['gps'][4]    )*100)
    drone['i_vd__cm/s']                 = round((   z['gps'][5]    )*100)
    drone['i_cog__cdeg']                = round((   z['gps'][6]    )*100)
    
    
    # HIL_SENSOR Data
    drone['f_xacc__m/s2']               = float((   z["accelerometer"][0]      )*1)
    drone['f_yacc__m/s2']               = float((   z["accelerometer"][1]      )*1)
    drone['f_zacc__m/s2']               = float((   z["accelerometer"][2]      )*1)
    drone['f_xgyro__rad/s']             = float((   z["gyroscope"][0]     )*1)
    drone['f_ygyro__rad/s']             = float((   z["gyroscope"][1]     )*1)
    drone['f_zgyro__rad/s']             = float((   z["gyroscope"][2]     )*1)
    drone['f_xmag__gauss']              = float((   z["magnetometer"][0]    )*1)
    drone['f_ymag__gauss']              = float((   z["magnetometer"][1]     )*1)
    drone['f_zmag__gauss']              = float((   z["magnetometer"][2]     )*1)
    drone['f_abs_pressure__hPa']        = float((   95598       + z["barometer"]["static"]        )*0.01)
    drone['f_diff_pressure__hPa']       = float((   z["barometer"]["dynamic"]         )*0.01)
    drone['f_pressure_alt__?']          = float((   z['gps'][2]         +crandom()*0.5      )*1)
    drone['f_temperature__degC']        = float((   0           +crandom()*0        )*1)
    
  

    # HIL_STATE_QUATERNION  Data
    drone['f_attitude_quaternion__1']   = [
                                          float((   quat[0]        )*1),
                                          float((   quat[1]        )*1),
                                          float((   quat[2]        )*1),
                                          float((   quat[3]        )*1),
                                        ]
    drone['f_rollspeed__rad/s']         = float((   omega[0]         )*1)
    drone['f_pitchspeed__rad/s']        = float((   omega[1]         )*1)
    drone['f_yawspeed__rad/s']          = float((   omega[2]         )*1)
    drone['i_vx__cm/s']                 = round((   vel[0]          +crandom()*0.001    )*100)
    drone['i_vy__cm/s']                 = round((   vel[1]            +crandom()*0.001    )*100)
    drone['i_vz__cm/s']                 = round((   vel[2]            +crandom()*0.001    )*100)
    drone['i_ind_airspeed__cm/s']       = round((   0           + random()*0.001    )*100)
    drone['i_true_airspeed__cm/s']      = round((   0           + random()*0.3      )*100)
    drone['i_xacc__mG']                 = round((   accel[0]/P.gravity     )*100)
    drone['i_yacc__mG']                 = round((   accel[1]/P.gravity     )*100)
    drone['i_zacc__mG']                 = round((   accel[2]/P.gravity     )*100)
    
  

   # if armed:    
    #    euler = np.rad2deg(Quaternion.quat2Euler(quat))

        #print("POS",np.around(pos,0),"EULER", np.around(euler,0),"MAG",np.around(z["magnetometer"],2), "ACC",np.around(z["accelerometer"],2), "GPS", np.around(z['gps'][3:],2) )

    return drone