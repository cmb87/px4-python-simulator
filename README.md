# PX4 Python SITL (Fixed-Wing)

Python lockstep SITL bridge that runs the local 6DOF fixed-wing model and publishes MAVLink HIL sensor streams to PX4.

## Architecture

```text
                      +-----------------------+
                      |       PX4 SITL        |
                      |  EKF / controllers    |
                      +-----------+-----------+
                                  |
                     HIL_ACTUATOR_CONTROLS
                                  |
                                  v
                    +-----------------------------+
                    |    world_mavlink_loop.py    |
                    |  lockstep + MAVLink bridge  |
                    +--------------+--------------+
                                   |
                          controls_to_u()
                                   |
                                   v
                    +-----------------------------+
                    |        vehicle.World         |
                    |-----------------------------|
                    |  WingX8ForceModel (forces)  |
                    |  Dynamics6DOF (state integ) |
                    |  SensorSuite (IMU/Mag/Baro/ |
                    |              GPS synthesis)  |
                    +---------------+-------------+
                                    |
               +--------------------+--------------------+
               |                    |                    |
               v                    v                    v
       HIL_SENSOR (250 Hz)   HIL_GPS (5 Hz)    HIL_STATE_QUATERNION
               |                    |               (on PX4 request)
               +--------------------+--------------------+
                                    |
                                    v
                                 PX4 EKF


Ground-truth side channel:

vehicle.World -> world_mavlink_loop.py -> GroundTruth WS -> ground_truth_ws_visualizer.py
```

## Coordinate Frames

- Navigation frame is NED (`x=north`, `y=east`, `z=down`).
- Body frame is FRD (`x=forward`, `y=right`, `z=down`).
- The ground-truth visualizer displays NED directly (`Down` axis).

## Important Implementation Note

The IMU path in `vehicle/sensors/sensors.py` reconstructs accelerometer input from body-velocity derivative plus transport term:

`accel_for_imu = ydot_body_vel + omega x vel_body`

This aligns the simulated accelerometer behavior with the Java implementation and keeps EKF turn behavior consistent.

## Run

- Start this bridge: `python world_mavlink_loop.py`
- Optional ground-truth view: `python ground_truth_ws_visualizer.py --host 127.0.0.1 --port 8765`
