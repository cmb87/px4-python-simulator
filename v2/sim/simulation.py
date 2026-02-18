import threading
import numpy as np
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import time
from random import random

from quaternion import Quaternion
from parameters import Parameters
from forces import forces, railForces
from sensors import sensors, Update
from dynamics import railDynamics, dynamics



# ==================================================

drone = {}
mavlinkData = {}

state = {"ready": False}

# ==================================================
def mavlink_listener(name, connection_string):
    print(f"[{name}] Connecting to {connection_string}...")
    master = mavutil.mavlink_connection(connection_string)

    # Wait for heartbeat
    master.wait_heartbeat()
    print(f"[{name}] Heartbeat received from system {master.target_system}, component {master.target_component}")



    while True:

      #  msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
      #  if msg:
      
     #       mavlinkData['gps'] = msg.to_dict()

        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            system_status = msg.system_status
            base_mode = msg.base_mode

            # Check if system is ready to arm
            if system_status == mavlink2.MAV_STATE_STANDBY:
                print("✅ System is READY TO ARM.")
                
            elif system_status == mavlink2.MAV_STATE_ACTIVE:
                print("✅ System is ARMED and ACTIVE.")
                state["ready"] = True

            elif system_status in [mavlink2.MAV_STATE_CRITICAL, mavlink2.MAV_STATE_EMERGENCY, mavlink2.MAV_STATE_POWEROFF]:
                print("❌ System is in a failure state.")
            else:
                print(f"System status: {system_status} (not ready)")




# ==================================================
def mavlink_simulation(name, connection_string):

    print(f"[{name}] Connecting to {connection_string}...")
    master = mavutil.mavlink_connection(connection_string)

    # Wait for heartbeat
    master.wait_heartbeat()
    print(f"[{name}] Heartbeat received from system {master.target_system}, component {master.target_component}")

    t_abs__s    = time.time()
    t_abs__us   = round(t_abs__s * 1e6)
    t_boot__us  = round(t_abs__us - 30e6)

    global drone

    P = Parameters()  # Load parameters, assuming this is defined in parameters.py

    railPitch = np.deg2rad(30.0)
    railYaw = np.deg2rad(0.0)

    quatRail = Quaternion.euler2quat(np.asarray([0.0,railPitch,railYaw]))
    P.rail_dir_ned = Quaternion.Mfg(quatRail).T @ np.asarray([1,0,0])

    # Example usage
    t = 0.0
    y = np.zeros(13)  # Initial state

    y[7] = 0.0 # Start speed in body dir
    y[0:3] = P.rail_start_ned
    y[3:7] = quatRail
    wind = np.zeros(6)  # Example wind vector

    u = np.zeros(4)  # Example control inputs
    u[3] = 0.78
    u[1] = 0.000

    mode = 0
    controls = np.zeros(16)

    delta_time__us = 50.0

    # Listen for MAVLink messages
    while True:


        t__us = t_abs__us - t_boot__us
        t__s = t__us / 1e6
        dt= delta_time__us / 1e6


        # =========== Controls to ==============
        elevator = np.deg2rad(20) * controls[0]
        left_aileron = controls[1]
        right_aileron = controls[2]
        aileron = np.deg2rad(20) * 0.5*(-right_aileron + left_aileron)

        rudder = np.deg2rad(20) * controls[3]

        throttle = controls[0]


        u = np.asarray([elevator,-aileron,rudder,throttle])
        #print(np.around(controls[:6],3), np.around(u) ) 


        # =========== Simulation step ===========
        if state["ready"]:

          #  print(u, controls[:6])
            tau = forces(t, y, u, wind, P)

            if P.left_rail:
                ydot = dynamics(t, y, P, tau)
                
            else:
                tauRail = railForces(t, y, u, wind, P)
                ydot = railDynamics(t, y, P, tau+tauRail)

        else:
            ydot = np.zeros(13)


        z = sensors(t, y, ydot, u, wind, P, dt)
        drone = Update(drone, t, y, ydot, z, u, wind, P, armed=state["ready"])


        # Update
        y += ydot * dt  # Update state using Euler method
        t += dt


        if (t__us % 100000 == 0):
            print("POS",np.around(y[:3],0),"EULER", np.around(z["euler"],0), int(1e7*(z["gps"][0]-P.gps_origin["lat"])),  int(1e+7 * (z["gps"][1]-P.gps_origin["lon"]) ))

        # =========== SYSTEM TIME ===========
        if t__us % 4000000 == 0:

            since_boot__us = t_abs__us - t_boot__us
            since_boot__ms = round(since_boot__us / 1000)
            
            time_unix_usec      = t_abs__us
            time_boot_ms        = since_boot__ms
            
            master.mav.system_time_send(
                time_unix_usec  = time_unix_usec        , # Timestamp (UNIX epoch time). [us] (type:uint64_t)
                time_boot_ms    = time_boot_ms          , # Timestamp (time since system boot). [ms] (type:uint32_t)
            )
            
        # =========== HEARTBEAT ===========
        if t__us % 1000000 == 0:
 
            the_type        = 0     # Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type. (type:uint8_t, values:MAV_TYPE)
            autopilot       = 0     # Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers. (type:uint8_t, values:MAV_AUTOPILOT)
            base_mode       = 0     # System mode bitmap. (type:uint8_t, values:MAV_MODE_FLAG)
            custom_mode     = 0     # A bitfield for use for autopilot-specific flags (type:uint32_t)
            system_status   = 0     # System status flag. (type:uint8_t, values:MAV_STATE)
            mavlink_version = 3     # MAVLink version, not writable by user, gets added by protocol because of magic data type          , # uint8_t_mavlink_version (type:uint8_t)
            
     
            # master.mav.heartbeat_send(
            #     type                = the_type          , 
            #     autopilot           = autopilot         , 
            #     base_mode           = base_mode         , 
            #     custom_mode         = custom_mode       , 
            #     system_status       = system_status     , 
            #     mavlink_version     = mavlink_version   , 
            # )

           # print(" HB Sent")
            
        # =========== HIL SENSOR ===========   
        if t__us % 4000 == 0:


            fields_updated = 0
            fields_updated |= (1 << 0)   # HIL_SENSOR_UPDATED_XACC
            fields_updated |= (1 << 1)   # HIL_SENSOR_UPDATED_YACC
            fields_updated |= (1 << 2)   # HIL_SENSOR_UPDATED_ZACC
            fields_updated |= (1 << 3)   # HIL_SENSOR_UPDATED_XGYRO
            fields_updated |= (1 << 4)   # HIL_SENSOR_UPDATED_YGYRO
            fields_updated |= (1 << 5)   # HIL_SENSOR_UPDATED_ZGYRO
            fields_updated |= (1 << 6)   # HIL_SENSOR_UPDATED_XMAG
            fields_updated |= (1 << 7)   # HIL_SENSOR_UPDATED_YMAG
            fields_updated |= (1 << 8)   # HIL_SENSOR_UPDATED_ZMAG
            fields_updated |= (1 << 9)   # HIL_SENSOR_UPDATED_ABS_PRESSURE
           # fields_updated |= (1 << 10)  # HIL_SENSOR_UPDATED_DIF_PRESSURE
            fields_updated |= (1 << 11)  # HIL_SENSOR_UPDATED_PRESSURE_ALT
            fields_updated |= (1 << 12)  # HIL_SENSOR_UPDATED_TEMPERATURE

         #   print("Fields updated:", fields_updated)

            time_usec           = t_abs__us                     # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
            xacc                = drone['f_xacc__m/s2']         # X acceleration [m/s/s] (type:float)
            yacc                = drone['f_yacc__m/s2']         # Y acceleration [m/s/s] (type:float)
            zacc                = drone['f_zacc__m/s2']         # Z acceleration [m/s/s] (type:float)
            xgyro               = drone['f_xgyro__rad/s']       # Angular speed around X axis in body frame [rad/s] (type:float)
            ygyro               = drone['f_ygyro__rad/s']       # Angular speed around Y axis in body frame [rad/s] (type:float)
            zgyro               = drone['f_zgyro__rad/s']       # Angular speed around Z axis in body frame [rad/s] (type:float)
            xmag                = drone['f_xmag__gauss']        # X Magnetic field [gauss] (type:float)
            ymag                = drone['f_ymag__gauss']        # Y Magnetic field [gauss] (type:float)
            zmag                = drone['f_zmag__gauss']        # Z Magnetic field [gauss] (type:float)
            abs_pressure        = drone['f_abs_pressure__hPa']  # Absolute pressure [hPa] (type:float)
            diff_pressure       = drone['f_diff_pressure__hPa'] # Differential pressure (airspeed) [hPa] (type:float)
            pressure_alt        = drone['f_pressure_alt__?']    # Altitude calculated from pressure (type:float)
            temperature         = drone['f_temperature__degC']  # Temperature [degC] (type:float)
            fields_updated      = fields_updated                          # Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim. (type:uint32_t)
            the_id              = 0                             # Sensor ID (zero indexed). Used for multiple sensor inputs (type:uint8_t)
 
            master.mav.hil_sensor_send(
                time_usec           = time_usec         ,
                xacc                = xacc              ,
                yacc                = yacc              ,
                zacc                = zacc              ,
                xgyro               = xgyro             ,
                ygyro               = ygyro             ,
                zgyro               = zgyro             ,
                xmag                = xmag              ,
                ymag                = ymag              ,
                zmag                = zmag              ,
                abs_pressure        = abs_pressure      ,
                diff_pressure       = diff_pressure     ,
                pressure_alt        = pressure_alt      ,
                temperature         = temperature       ,
                fields_updated      = fields_updated    ,
                id                  = the_id            ,
            )

            #print("SENSOR Sent")
        
        # =========== HIL GPS ===========
        if t__us % 8000 == 0:
      
            
           
            # try:
            #     print("---------")
            #     print(mavlinkData['gps']["lat"]-int(P.gps_origin["lat"]*1e7), mavlinkData['gps']["lon"]-int(P.gps_origin["lon"]*1e7), mavlinkData['gps']["alt"]-int(P.gps_origin["alt"]*1e3 ), mavlinkData['gps']["vx"], mavlinkData['gps']["vy"], mavlinkData['gps']["vz"])
            #     print(drone['i_lat__degE7']-int(P.gps_origin["lat"]*1e7), drone['i_lon__degE7']-int(P.gps_origin["lon"]*1e7), drone['i_alt__mm']-int(P.gps_origin["alt"]*1e3),drone['i_vn__cm/s'], drone['i_ve__cm/s'], drone['i_vd__cm/s'])
            # except:
            #     pass

            time_usec           = t_abs__us                 # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
            fix_type            = 3                         # 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. (type:uint8_t)
            lat                 = drone['i_lat__degE7']     # Latitude (WGS84) [degE7] (type:int32_t)
            lon                 = drone['i_lon__degE7']     # Longitude (WGS84) [degE7] (type:int32_t)
            alt                 = drone['i_alt__mm']        # Altitude (MSL). Positive for up. [mm] (type:int32_t)
            eph                 = drone['i_eph__cm']        # GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
            epv                 = drone['i_epv__cm']        # GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
            vel                 = drone['i_vel__cm/s']      # GPS ground speed. If unknown, set to: 65535 [cm/s] (type:uint16_t)
            vn                  = drone['i_vn__cm/s']       # GPS velocity in north direction in earth-fixed NED frame [cm/s] (type:int16_t)
            ve                  = drone['i_ve__cm/s']       # GPS velocity in east direction in earth-fixed NED frame [cm/s] (type:int16_t)
            vd                  = drone['i_vd__cm/s']       # GPS velocity in down direction in earth-fixed NED frame [cm/s] (type:int16_t)
            cog                 = 65535 #     # Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535 [cdeg] (type:uint16_t)
            satellites_visible  = 10                        # Number of satellites visible. If unknown, set to 255 (type:uint8_t)
            the_id              = 0                         # GPS ID (zero indexed). Used for multiple GPS inputs (type:uint8_t)
            yaw                 = 0                         # Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north [cdeg] (type:uint16_t)
            

            master.mav.hil_gps_send(
                time_usec           = time_usec             ,
                fix_type            = fix_type              ,
                lat                 = lat                   ,
                lon                 = lon                   ,
                alt                 = alt                   ,
                eph                 = eph                   ,
                epv                 = epv                   ,
                vel                 = vel                   ,
                vn                  = vn                    ,
                ve                  = ve                    ,
                vd                  = vd                    ,
                cog                 = cog                   ,
                satellites_visible  = satellites_visible    ,
                id                  = the_id                ,
                yaw                 = yaw                   ,
            )

            #print("GPS Sent")

        # =========== HIL STATE QUATERION ===========        
        if t__us % 8000 == 0:

            time_usec           = t_abs__us                         # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
            attitude_quaternion = drone['f_attitude_quaternion__1'] # Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation) (type:float)
            rollspeed           = drone['f_rollspeed__rad/s']       # Body frame roll / phi angular speed [rad/s] (type:float)
            pitchspeed          = drone['f_pitchspeed__rad/s']      # Body frame pitch / theta angular speed [rad/s] (type:float)
            yawspeed            = drone['f_yawspeed__rad/s']        # Body frame yaw / psi angular speed [rad/s] (type:float)
            lat                 = drone['i_lat__degE7']             # Latitude [degE7] (type:int32_t)
            lon                 = drone['i_lon__degE7']             # Longitude [degE7] (type:int32_t)
            alt                 = drone['i_alt__mm']                # Altitude [mm] (type:int32_t)
            vx                  = drone['i_vx__cm/s']               # Ground X Speed (Latitude) [cm/s] (type:int16_t)
            vy                  = drone['i_vy__cm/s']               # Ground Y Speed (Longitude) [cm/s] (type:int16_t)
            vz                  = drone['i_vz__cm/s']               # Ground Z Speed (Altitude) [cm/s] (type:int16_t)
            ind_airspeed        = drone['i_ind_airspeed__cm/s']     # Indicated airspeed [cm/s] (type:uint16_t)
            true_airspeed       = drone['i_true_airspeed__cm/s']    # True airspeed [cm/s] (type:uint16_t)
            xacc                = drone['i_xacc__mG']               # X acceleration [mG] (type:int16_t)
            yacc                = drone['i_yacc__mG']               # Y acceleration [mG] (type:int16_t)
            zacc                = drone['i_zacc__mG']               # Z acceleration [mG] (type:int16_t)
            

            # master.mav.hil_state_quaternion_send(
            #     time_usec           = time_usec             ,
            #     attitude_quaternion = attitude_quaternion   ,
            #     rollspeed           = rollspeed             ,
            #     pitchspeed          = pitchspeed            ,
            #     yawspeed            = yawspeed              ,
            #     lat                 = lat                   ,
            #     lon                 = lon                   ,
            #     alt                 = alt                   ,
            #     vx                  = vx                    ,
            #     vy                  = vy                    ,
            #     vz                  = vz                    ,
            #     ind_airspeed        = ind_airspeed          ,
            #     true_airspeed       = true_airspeed         ,
            #     xacc                = xacc                  ,
            #     yacc                = yacc                  ,
            #     zacc                = zacc                  ,
            # )

          #  print("QUAT Sent")
                

        # =========== RECEIVE MESSAGES ===========

        msg = master.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=False)

        if msg:
            mode = msg.mode
            controls = np.asarray(msg.controls)


        # ========== INCREMENT =============
        t_abs__us += int(delta_time__us)





# Define two threads with different MAVLink addresses (update these with your real ports)
thread_1 = threading.Thread(target=mavlink_listener, args=("GCS", "udp:127.0.0.1:14540"),daemon=True)
thread_2 = threading.Thread(target=mavlink_simulation, args=("Simulator", "tcpin:localhost:4560"),daemon=True)

# Start threads
thread_1.start()
thread_2.start()

# Keep main thread alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down...")