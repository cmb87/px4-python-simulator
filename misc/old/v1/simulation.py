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
    t_abs__us   = int(round(t_abs__s * 1e6))
    t_boot__us  = int(round(t_abs__s * 1e6))
    niter = 0

    global drone

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
    wind = np.zeros(6)  # Example wind vector

    u = np.zeros(4)  # Example control inputs
    u[3] = 0.78
    u[1] = 0.000

    mode = 0
    controls = np.zeros(16)

    delta_time__us = 1.0/250.0 * 1e6
    dt = delta_time__us / 1e6
  

    advanceTime = False
    lastTimeRan = 0
    receivedActuatorControls = False

    # Listen for MAVLink messages
    while True:

        # =========== RECEIVE MESSAGES ===========

        msg = master.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=False)

        if msg:
            mode = msg.mode
            controls = np.asarray(msg.controls)


            # =========== Controls to ==============
            

            left_aileron = controls[1]
            right_aileron = controls[2]

            aileron = np.deg2rad(20) * 0.5*(-right_aileron + left_aileron)
            elevator = np.deg2rad(20) *  0.5*(+right_aileron + left_aileron)
            rudder = np.deg2rad(20) * controls[3]
            throttle = controls[0]

            u = np.asarray([-elevator,-aileron,rudder,throttle])

          #  print(u)

            advanceTime = True
            receivedActuatorControls = True

           # print("HIL_ACTUATOR_CONTROLS: ",  t_abs__us)
        else:
            receivedActuatorControls = False

        # ====================

        ioRunOnly = (niter % 2 != 0)
        needsToPause = ((lastTimeRan == t_abs__us) | ioRunOnly) 

        if ( not receivedActuatorControls and not ioRunOnly):
            advanceTime = True

        # =========== Simulation step ===========
        ydot = np.zeros(13)


        if advanceTime:

            if state["ready"]:
                tau = forces(t, y, u, wind, P)
  
                if P.left_rail:
                    ydot = dynamics(t, y, P, tau)
                else:
                    tauRail = railForces(t, y, u, wind, P)
                    ydot = railDynamics(t, y, P, tau+tauRail)

            else:
                ydot = np.zeros(13)
            

            # Update
            lastTimeRan = t_abs__us
            y += ydot * dt  # Update state using Euler method
            t += dt
            t_abs__us += int(delta_time__us)
            advanceTime = False
           # print("Advance Time: ",  t_abs__us)
            #time.sleep(dt)


            
        # Update Sensor model:
        z = sensors(t, y, ydot, u, wind, P, dt)


        # =========== SYSTEM TIME ===========
        if niter % 1000 == 0:

            unix_ms = int(time.time() * 1000)          # milliseconds since 1970 UTC
            unix_us = unix_ms * 1000                   # microseconds


            print('time_unix_usec ', unix_ms)
            print('time_boot_ms   ', int((t_abs__us)/1000))


            # JMavSim
            # <= time_unix_usec 1768291847414000
            # <= time_boot_ms 1768291841575
                           #   1768291966223
                           #   4294967295
            



            master.mav.system_time_send(
                time_unix_usec  = unix_us,        # Timestamp (UNIX epoch time). [us] (type:uint64_t)
                time_boot_ms    = int((t_abs__us-t_boot__us)/1000)  # Timestamp (time since system boot). [ms] (type:uint32_t)
            )
            

            
        # =========== HIL SENSOR ===========   
        if niter % 1 == 0 and not needsToPause:

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
            fields_updated      = fields_updated                          # Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim. (type:uint32_t)

            master.mav.hil_sensor_send(
                time_usec           = t_abs__us,
                xacc                = z['accelerometer'][0],
                yacc                = z['accelerometer'][1],              
                zacc                = z['accelerometer'][2],              
                xgyro               = z['gyroscope'][0],
                ygyro               = z['gyroscope'][1],
                zgyro               = z['gyroscope'][2],
                xmag                = z['magnetometer'][0],
                ymag                = z['magnetometer'][1],
                zmag                = z['magnetometer'][2],
                abs_pressure        = 0.01*(95597 + z['barometer']["static"]), # Pa to millibar
                diff_pressure       = 0.0,
                pressure_alt        = z['barometer']["pressure_alt"],
                temperature         = 300.15,
                fields_updated      = fields_updated    ,
                id                  = 0            ,
            )

            #print("HIL_SENSOR: ",  t_abs__us)
        
        # =========== HIL GPS ===========
        if niter % 50 == 0 and z:

            gps = z['gps']

            vel = (gps[3]**2+gps[4]**2+gps[5]**2)**0.5
            cog = np.rad2deg(np.atan2(gps[3],gps[4]))

            print(cog)

            master.mav.hil_gps_send(
                time_usec=t_abs__us,
                lat=int(gps[0] * 1e7),
                lon=int(gps[1] * 1e7),
                alt=int(gps[2] * 1e3),   # mm
                eph=int(13.4),
                epv=int(13.5),
                vel=int(vel * 100),
                vn=int(gps[3] * 100),
                ve=int(gps[4] * 100),
                vd=int(gps[5] * 100),
                cog=int(cog * 100),
                fix_type=3,
                satellites_visible=10           # fake value
            )

            #print("GPS Sent")

        # =========== HIL STATE QUATERION ===========        
        if niter % 400 == 0:
            pass

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
                

        
        niter += 1







# Define two threads with different MAVLink addresses (update these with your real ports)
thread_1 = threading.Thread(target=mavlink_listener, args=("GCS", "udp:127.0.0.1:14540"),daemon=True)
thread_2 = threading.Thread(target=mavlink_simulation, args=("Simulator", "tcpin:127.0.0.1:4560"),daemon=True)

# Start threads
thread_1.start()
thread_2.start()

# Keep main thread alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Shutting down...")