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

- Optional ground-truth view: `python ground_truth_ws_visualizer.py --host 127.0.0.1 --port 8765`

### Start by role

Standalone (single simulator instance, no transfer alignment):

- `SIM_VEHICLE_MODEL=iris SIM_ROLE=standalone SIM_MAVLINK_BIND_PORT=4560 python src/main.py`

Master (runs local dynamics and streams ground truth to slave over UDP):

- `SIM_VEHICLE_MODEL=iris SIM_ROLE=master SIM_MAVLINK_BIND_PORT=4560 SIM_TRANSFER_UDP_TARGET_HOST=127.0.0.1 SIM_TRANSFER_UDP_TARGET_PORT=18000 python src/main.py`

Slave (uses master dynamics with rigid transform, then optional cutover):

- `SIM_VEHICLE_MODEL=iris SIM_ROLE=slave SIM_MAVLINK_BIND_PORT=4561 SIM_TRANSFER_UDP_BIND_HOST=0.0.0.0 SIM_TRANSFER_UDP_BIND_PORT=18000 SIM_TRANSFER_ARM_M=0.5,0.0,0.0 SIM_TRANSFER_REL_EULER_DEG=0.0,0.0,15.0 SIM_TRANSFER_CUTOVER_MODE=mavlink_cmd python src/main.py`

Master + slave with websocket enabled on both (same host):

- Master: `SIM_VEHICLE_MODEL=iris SIM_ROLE=master SIM_MAVLINK_BIND_PORT=4560 SIM_TRANSFER_UDP_TARGET_HOST=127.0.0.1 SIM_TRANSFER_UDP_TARGET_PORT=18000 SIM_GT_WS_ENABLED=true SIM_GT_WS_HOST=0.0.0.0 SIM_GT_WS_PORT=8765 python src/main.py`
- Slave: `SIM_VEHICLE_MODEL=iris SIM_ROLE=slave SIM_MAVLINK_BIND_PORT=4561 SIM_TRANSFER_UDP_BIND_HOST=0.0.0.0 SIM_TRANSFER_UDP_BIND_PORT=18000 SIM_TRANSFER_ARM_M=0.5,0.0,0.0 SIM_TRANSFER_REL_EULER_DEG=0.0,0.0,15.0 SIM_TRANSFER_CUTOVER_MODE=mavlink_cmd SIM_GT_WS_ENABLED=true SIM_GT_WS_HOST=0.0.0.0 SIM_GT_WS_PORT=8766 python src/main.py`

Ground-truth websocket options (useful for multi-instance setups):

- `SIM_GT_WS_ENABLED=auto|true|false` (default `auto`; enabled for `standalone`/`master`, disabled for `slave`)
- `SIM_GT_WS_HOST` (default `0.0.0.0`)
- `SIM_GT_WS_PORT` (default `8765`)

If websocket is enabled on multiple instances, use unique `SIM_GT_WS_PORT` values per instance.

Model selection works in all roles, e.g.:

- `SIM_VEHICLE_MODEL=iris SIM_ROLE=standalone python src/main.py`

## Transfer Alignment (Master/Slave)

`src/main.py` supports three runtime roles:

- `SIM_ROLE=standalone` (default): local dynamics only.
- `SIM_ROLE=master`: local dynamics + UDP ground-truth stream (`time_us`, `y`, `ydot`) for a slave instance.
- `SIM_ROLE=slave`: receives master stream, applies rigid transform (arm + relative orientation), and publishes transformed sensors to its own PX4 instance. Cutover can switch from coupled master dynamics to local slave dynamics.

### Role and MAVLink endpoint

- `SIM_MAVLINK_BIND_HOST` (default `0.0.0.0`)
- `SIM_MAVLINK_BIND_PORT` (default `4560`)

Use different MAVLink bind ports for master/slave when running both simultaneously.

### Master transfer stream

- `SIM_TRANSFER_UDP_TARGET_HOST` (default `127.0.0.1`)
- `SIM_TRANSFER_UDP_TARGET_PORT` (default `18000`)

### Slave transfer input and transform

- `SIM_TRANSFER_UDP_BIND_HOST` (default `0.0.0.0`)
- `SIM_TRANSFER_UDP_BIND_PORT` (default `18000`)
- `SIM_TRANSFER_TIMEOUT_S` (default `1.0`)
- `SIM_TRANSFER_ARM_M=dx,dy,dz` arm offset vector [m]
- `SIM_TRANSFER_ARM_FRAME=master_body` (only supported option)
  - `master_body`: arm is fixed in master body frame (rigid-body lever-arm model)
- `SIM_TRANSFER_REL_EULER_DEG=roll,pitch,yaw` fixed slave-from-master body rotation [deg]

Connection notifications:

- Slave logs when transfer connection to master is established.
- Master logs when it receives a transfer connection hello from a slave endpoint.

### Slave cutover modes

- `SIM_TRANSFER_CUTOVER_MODE=never|time|mavlink_cmd` (default `mavlink_cmd`)
- `SIM_TRANSFER_CUTOVER_TIME_S` (used when mode is `time`)

For `mavlink_cmd`, send `COMMAND_LONG` with `MAV_CMD_USER_1` to trigger cutover.

### Example two-instance setup (same host)

Master:

- `SIM_VEHICLE_MODEL=iris SIM_ROLE=master SIM_MAVLINK_BIND_PORT=4560 SIM_TRANSFER_UDP_TARGET_HOST=127.0.0.1 SIM_TRANSFER_UDP_TARGET_PORT=18000 SIM_GT_WS_ENABLED=true SIM_GT_WS_PORT=8765 python src/main.py`

Slave:

- `SIM_VEHICLE_MODEL=iris SIM_ROLE=slave SIM_MAVLINK_BIND_PORT=4561 SIM_TRANSFER_UDP_BIND_PORT=18000 SIM_TRANSFER_ARM_M=0.5,0.0,0.0 SIM_TRANSFER_ARM_FRAME=master_body SIM_TRANSFER_REL_EULER_DEG=0.0,0.0,15.0 SIM_GT_WS_ENABLED=true SIM_GT_WS_PORT=8766 python src/main.py`

## Transfer Alignment Flow and Lever Compensation

```mermaid
flowchart TD
  MState[Master state from local dynamics\ny_m, ydot_m] --> UDP[UDP transfer packet\nseq, time_us, y, ydot]
  UDP --> SRecv[Slave receives latest packet]
  SRecv --> Arm[Arm frame fixed to master body\nr = R_mw * arm_m]
  Arm --> Kine
  Kine[Lever compensation\nv_s,w = v_m,w + omega_w x r\na_s,w = a_m,w + alpha_w x r + omega_w x (omega_w x r)] --> Rot
  Rot[Apply relative attitude\nq_s = q_sm ⊗ q_m\nomega_s,b = R_sm * omega_m,b\nv_s,b = R_sw * v_s,w] --> Out
  Out[Publish transformed slave sensors\nto slave PX4 instance]
```

Symbols below match `src/transfer_alignment.py` (`transform_master_to_slave_state(...)`):

- `q_m`: master attitude quaternion (wxyz), body-to-world matrix `R_mw = Mfg(q_m)^T`
- `q_sm`: fixed slave-from-master body rotation from `SIM_TRANSFER_REL_EULER_DEG`
- `q_s = q_sm ⊗ q_m`: slave attitude quaternion
- `arm_m`: `SIM_TRANSFER_ARM_M` lever arm
- `r`: lever arm in world frame (`r = R_mw arm_m`)

Equations used for rigid-body lever-arm compensation:

- Position: `p_s = p_m + r`
- Angular rate (slave body): `omega_s,b = R_sm omega_m,b`
- World velocity: `v_s,w = v_m,w + omega_w x r`
- World acceleration: `a_s,w = a_m,w + alpha_w x r + omega_w x (omega_w x r)`
- Slave body velocity derivative: `dv_s,b = R_sw a_s,w - omega_s,b x v_s,b`
- Quaternion derivative: `qdot_s = 0.5 * Omega(omega_s,b) * q_s`
