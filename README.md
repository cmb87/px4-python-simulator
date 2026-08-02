# PX4 Python SITL

Python lockstep SITL bridge that runs the local 6DOF vehicle model and publishes MAVLink HIL sensor streams to PX4.

## Python Package

This repository is installable as a Python package.

- Editable install: `pip install -e .`
- Standard install: `pip install .`
- Console entry point after install: `px4-python-sitl`

The package metadata is defined in `pyproject.toml`.

If editable install fails in a ROS2 container with a `build_editable` error, upgrade build tooling first and then install:

- `python3 -m pip install --upgrade pip setuptools wheel build`
- `python3 -m pip install -e . || python3 -m pip install .`

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

vehicle.World -> main.py -> GroundTruth WS -> src/test/ground_truth_ws_visualizer.py
```

## Vehicle Models

Vehicle-specific setup lives in `src/vehicles/<name>/`, and model selection lives in `src/vehicles/vehicle_catalog.py`.

Default models:

- `x8`
- `iris`

`src/dynamics/world.py` is model-agnostic: it resolves parameters, force models, and default initial state via the vehicle catalog.

### Add a New Vehicle

1. Add a vehicle folder under `src/vehicles/` (for example `src/vehicles/my_uav/`) with:
   - `parameters.py`
   - `forces.py`
   - `initial_state.py`
   - `definition.py` exposing `make_parameters()`, `make_force_models(parameters)`, and `make_initial_state(config)`
   - optionally reuse shared force blocks from `src/vehicles/common_forces/`
2. Register it explicitly in `src/vehicles/vehicle_catalog.py` by adding one entry to `VEHICLES`.
3. Run tests (`pytest src/test/test_vehicle_catalog.py`) to verify catalog wiring and one-step world execution.

### Optional Rail Launch (All Vehicles)

Rail launch can be enabled for any vehicle model. While on rail, the simulator constrains motion to rail translation and keeps attitude aligned to the rail. After rail distance reaches rail length, dynamics automatically switch to free 6DOF.

Per-vehicle rail parameters live in each vehicle's `parameters.py`:

- `rail_launch_enabled` (default `False`)
- `rail_dir_ned` (default `[0.7071, 0.0, -0.7071]`)
- `rail_start_ned` (default `[0.0, 0.0, 0.0]`)
- `rail_length` (default `2.0`)
- `rail_pull_max` (default `1.0`)

Runtime launch delay after arm is controlled by env var:

- `SIM_CATAPULT_LAUNCH_COUNTDOWN_S` (default `3.0`)
- Applies only when `rail_launch_enabled=True`.

### How Models Integrate Into Architecture

At runtime, model integration happens in this order:

1. `src/main.py` reads `SIM_VEHICLE_MODEL` and creates `World(vehicle_model=...)`.
2. `src/dynamics/world.py` resolves model-specific pieces from the catalog:
   - parameters via the model spec
   - force model list via the model spec
   - default initial state via the model spec
3. For each simulation step, `World.update(...)`:
   - evaluates and sums all force-model outputs into `tau`
   - runs `Dynamics6DOF` with `tau` and `P`
   - runs `SensorSuite` from updated state and state derivative
4. `src/main.py` publishes sensor outputs to PX4 via MAVLink HIL messages.

This keeps vehicle specifics inside `src/vehicles/<name>/*`, while `src/dynamics/world.py` stays vehicle-agnostic.

### Select Model

Use `SIM_VEHICLE_MODEL`:

- `SIM_VEHICLE_MODEL=x8` (default)
- `SIM_VEHICLE_MODEL=iris`
- `SIM_VEHICLE_MODEL=ts06`
- `SIM_GPS_LAT` / `SIM_GPS_LON` / `SIM_GPS_ALT` set GPS origin for all models.
  Legacy names `SIM_GPS_ORIGIN_LAT` / `SIM_GPS_ORIGIN_LON` / `SIM_GPS_ORIGIN_ALT` are still accepted.
  Defaults: `47.397742`, `8.545594`, `470.0`.
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

## Sensor Noise Defaults

To better match jMAVSim-like behavior, IMU, magnetometer, GPS, and barometer noise paths are enabled by default in `SensorSuite`.

## Gymnasium / RL Usage

The simulator can also run without PX4 through a Gymnasium-compatible environment.

- Install with RL dependencies: `pip install -e .[rl]`
- Import: `from rl import Px4SimEnv`
- The environment wraps `dynamics.world.World` directly, so no MAVLink connection or PX4 process is required.
- Observations are the 13-element ground-truth state by default: position NED, quaternion `wxyz`, body velocity, and body rates.
- Actions are 4 normalized values in `[-1, 1]`; for `iris` they are mapped to motor commands in `[0, 1]`, for fixed-wing models action 0 is mapped to throttle and the remaining channels are passed through as surface commands.

Example airborne reset:

```python
from rl import Px4SimEnv

env = Px4SimEnv(vehicle_model="iris", initial_state="airborne")
obs, info = env.reset(options={"altitude_m": 30.0, "speed_mps": 0.0})
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
env.close()
```

Launch handling is explicit through `launch_mode`:

- `default`: use the vehicle's configured launch behavior.
- `catapult`: force a rail/catapult start; the model begins on the rail and switches to free 6DOF after rail exit.
- `airborne`: preserve the airborne initial state and skip rail dynamics by setting `P.left_rail=True`.
- `free`: disable launch mechanics entirely and start directly in free 6DOF.

Optional websocket ground-truth output is available for debugging and visualization:

```python
env = Px4SimEnv(
    vehicle_model="iris",
    initial_state="airborne",
    enable_websocket=True,
    websocket_port=8765,
    websocket_system_id=1,
)
```

For parallel RL training, keep websocket disabled or assign a unique port to each debug instance.

X8 websocket debug example:

- Run: `PYTHONPATH=src python support/examples/x8_rl_websocket.py`
- Connect the visualizer to `ws://127.0.0.1:8765`
- The example starts the X8 already airborne with `launch_mode="airborne"`, so the catapult rail switch is skipped.

Two independent X8 vehicles with two websocket outputs:

- Run: `PYTHONPATH=src python support/examples/two_x8_two_websockets.py`
- Connect one visualizer to `ws://127.0.0.1:8765` for `system_id=1`
- Connect another visualizer to `ws://127.0.0.1:8766` for `system_id=2`
- This uses two independent `Px4SimEnv` instances, not coupled multi-vehicle physics.
- The example also prints a simple ground-truth camera measurement in both directions: center pixel, depth, and visibility.

Simple camera projection utility:

```python
from perception import PinholeCamera

camera = PinholeCamera(width=640, height=480, fx=400.0, fy=400.0)
measurement = camera.project_world_point(
    point_ned=neighbor_state[0:3],
    ego_position_ned=ego_state[0:3],
    ego_quaternion_wxyz=ego_state[3:7],
)
print(measurement.visible, measurement.pixel, measurement.depth_m)
```

## Technical Documentation (LaTeX)

A full architecture document is available at `support/docs/src/architecture.tex`.

- Build PDF: `make -C support/docs/src`
- Output: `support/docs/build/architecture.pdf`

The document covers:

- 6DOF dynamics kernel
- vehicle models (`iris`, `x8`, `ts06`)
- sensor and force models
- MAVLink HIL and external interfaces (WebSocket/FlightGear ground truth)

## Run

- Optional ground-truth view: `python src/test/ground_truth_ws_visualizer.py --host 127.0.0.1 --port 8765`

### Start

Single-instance simulator run:

- `SIM_VEHICLE_MODEL=iris SIM_MAVLINK_BIND_PORT=4560 python src/main.py`

Ground-truth output options:

- `SIM_GT_OUTPUT_MODE=websocket|flightgear_udp|off` (default `websocket`)
- `SIM_GT_OUTPUT_RATE_HZ` (default `30.0`)
- `SIM_GT_WS_HOST` (default `0.0.0.0`)
- `SIM_GT_WS_PORT` (default `8765`)
- `SIM_FG_UDP_HOST` (default `127.0.0.1`)
- `SIM_FG_UDP_PORT` (default `5503`)

If websocket output is enabled on multiple instances, use unique `SIM_GT_WS_PORT` values per instance.

FlightGear UDP mode publishes smooth ground-truth pose (not noisy GPS sensor output) as FGNetFDM v24 packets.

Example (FlightGear UDP output):

- `SIM_VEHICLE_MODEL=iris SIM_GT_OUTPUT_MODE=flightgear_udp SIM_GT_OUTPUT_RATE_HZ=30 SIM_FG_UDP_HOST=127.0.0.1 SIM_FG_UDP_PORT=5503 python src/main.py`

Model selection example:

- `SIM_VEHICLE_MODEL=iris python src/main.py`

### Docker

Build the image from the repo root:

- `./docker/build.sh`

Run the default container setup:

- `docker compose -f docker/docker-compose.yaml up`

The compose file runs the simulator in host-network mode so it can talk to a local PX4 SITL instance without extra port mapping.

Common settings live in `docker/docker-compose.yaml`:

- `SIM_VEHICLE_MODEL` selects the simulated vehicle (default there is `ts06`)
- `SIM_MAVLINK_BIND_HOST` / `SIM_MAVLINK_BIND_PORT` configure the MAVLink TCP listener
- `SIM_GT_OUTPUT_MODE` selects `websocket`, `flightgear_udp`, or `off`
- `SIM_GT_WS_PORT` configures the ground-truth websocket port

Stop the container with:

- `docker compose -f docker/docker-compose.yaml down`

## Support Layout

- `support/examples/` contains runnable example scripts.
- `support/tools/` contains helper scripts and the multi-UAV viewer.
- `support/docs/src/` contains authored documentation sources.
- `support/docs/build/` contains generated LaTeX build outputs.
- `support/docs/references/` contains reference PDFs kept in the repository.


# Interactive Websocket visualization

Go to https://cmb87.github.io/ and connect the simulator via websocket
