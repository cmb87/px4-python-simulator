# PX4 Python SITL

Python lockstep SITL bridge that runs the local 6DOF vehicle model and publishes MAVLink HIL sensor streams to PX4.

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
                    |           main.py           |
                    |  lockstep + MAVLink bridge  |
                    +--------------+--------------+
                                   |
                          controls_to_u()
                                   |
                                   v
                    +-----------------------------+
                    |        vehicle.World        |
                    |-----------------------------|
                    |  selected vehicle model     |
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

vehicle.World -> main.py -> GroundTruth WS -> ground_truth_ws_visualizer.py
```

## Vehicle Models

Vehicle-specific setup lives in `src/vehicle/model/`.

- `x8`: fixed-wing model using `forces/wing_x8.py`
- `iris`: quadcopter model using:
  - `forces/iris_quad.py` (4x simple motor model)
  - `forces/passive_sphere_aero.py` (passive drag)

`vehicle/world.py` only selects the active model and runs dynamics/sensors.

### How Models Integrate Into Architecture

At runtime, model integration happens in this order:

1. `src/main.py` reads `SIM_VEHICLE_MODEL` and creates `World(vehicle_model=...)`.
2. `vehicle/world.py` resolves model-specific pieces:
   - parameters via `_build_parameters(...)`
   - force model list via `_build_force_models(...)`
3. For each simulation step, `World.update(...)`:
   - passes `y`, `u`, `wind`, `P` into each force model
   - sums all returned 6D wrenches into `tau`
   - runs `Dynamics6DOF` with `tau` and `P`
   - runs `SensorSuite` from updated state and state derivative
4. `src/main.py` publishes sensor outputs to PX4 via MAVLink HIL messages.

This keeps vehicle specifics inside `vehicle/model/*` and `vehicle/forces/*`, while `vehicle/world.py` stays vehicle-agnostic.

### Select Model

Use `SIM_VEHICLE_MODEL`:

- `SIM_VEHICLE_MODEL=x8` (default)
- `SIM_VEHICLE_MODEL=iris`
- `SIM_VEHICLE_MODEL=ts04`
- `SIM_GPS_ORIGIN_LAT` / `SIM_GPS_ORIGIN_LON` / `SIM_GPS_ORIGIN_ALT` set GPS origin for all models.
  Defaults: `47.397742`, `8.545594`, `470.0`.

TS04 optional startup attitude toggle:

- `SIM_TS04_PITCH90_START=1` starts the model at +90 deg pitch attitude.
- Default is off (`SIM_TS04_PITCH90_START=0`).
- `SIM_TS04_MOTOR_MAP=a,b,c,d` remaps TS04 sim motor indices `[0,1,2,3]` from incoming
  `HIL_ACTUATOR_CONTROLS.controls[a,b,c,d]`.
  Example: `SIM_TS04_MOTOR_MAP=1,3,0,2`.
- MAVLink (`HIL_STATE_QUATERNION`) and ground-truth websocket always publish raw simulation attitude.
- Ground-truth websocket payload includes `aero.alpha_deg` and `aero.beta_deg` (set to `null` when airspeed is too low).

## Coordinate Frames

- Navigation frame is NED (`x=north`, `y=east`, `z=down`).
- Body frame is FRD (`x=forward`, `y=right`, `z=down`).
- The ground-truth visualizer displays NED directly (`Down` axis).

## Important Implementation Note

The IMU path in `vehicle/sensors/sensors.py` reconstructs accelerometer input from body-velocity derivative plus transport term:

`accel_for_imu = ydot_body_vel + omega x vel_body`

This aligns the simulated accelerometer behavior with the Java implementation and keeps EKF turn behavior consistent.

## Run

- Start this bridge: `python src/main.py`
- Optional ground-truth view: `python ground_truth_ws_visualizer.py --host 127.0.0.1 --port 8765`

Example:

- `SIM_VEHICLE_MODEL=iris python src/main.py`
