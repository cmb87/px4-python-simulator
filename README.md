# PX4 Python SITL

Python lockstep SITL bridge for PX4 with a local 6DOF vehicle model, synthesized sensors, optional ground-truth output, and a Gymnasium-compatible RL interface.

## What This Repo Does

- Runs a local vehicle model in Python through `src/dynamics/world.py`
- Publishes MAVLink HIL sensor data to PX4 through `src/main.py`
- Supports multiple vehicle definitions: `iris`, `x8`, `ts06`
- Can run without PX4 through `rl.Px4SimEnv`
- Includes a simple pinhole camera projection utility in `src/perception/`
- Includes support scripts, examples, and technical docs under `support/`

## Repository Layout

- `src/` core simulator, vehicle models, RL environment, networking, and perception utilities
- `px4/` PX4 airframe integration notes and custom airframe files
- `docker/` container build and compose setup
- `support/examples/` runnable examples
- `support/tools/` helper scripts and the multi-UAV viewer
- `support/docs/src/` authored technical documentation
- `support/docs/build/` generated LaTeX outputs
- `support/docs/references/` reference PDFs kept in the repo

## Install

Standard install:

- `pip install .`

If you want the RL environment too:

- `pip install .[rl]`

If editable install fails in a container with a `build_editable` error, upgrade build tools first and retry:

- `python3 -m pip install --upgrade pip setuptools wheel build`
- `python3 -m pip install -e . || python3 -m pip install .`

For development without package installation, most examples and tests also work with `PYTHONPATH=src`.

## Core Concepts

- World frame: NED (`x=north`, `y=east`, `z=down`)
- Body frame: FRD (`x=forward`, `y=right`, `z=down`)
- Main simulator loop: `src/main.py`
- Vehicle model selection: `src/vehicles/vehicle_catalog.py`
- Model-agnostic plant wrapper: `src/dynamics/world.py`
- Ground-truth websocket publisher: `src/networking/websocket_publisher.py`

The simulator runs the plant locally, synthesizes IMU, magnetometer, barometer, and GPS outputs, and streams them to PX4 as MAVLink HIL messages.

## Use With PX4

### 1. Prepare PX4

For stock `iris`, PX4 SITL can already run without custom airframes.

For custom models such as `x8` and `ts06`, this repo ships PX4 airframe files under `px4/airframes/`.

At the moment those files are:

- `px4/airframes/10021_none_x8`
- `px4/airframes/10022_none_ts06`
- `px4/airframes/ts06_params.params`

To make PX4 aware of these vehicles:

1. copy the airframe files from `px4/airframes/` into your PX4 checkout
2. register them in PX4 CMake files
3. build PX4 SITL targets such as `none_x8` or `none_ts06`

In practice:

1. Copy the airframe files:

```bash
cp px4/airframes/* <PX4 repo>/ROMFS/px4fmu_common/init.d-posix/airframes/
```

2. Register the airframes in:

- `<PX4 repo>/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`

Add entries such as:

```text
10021_none_x8
10022_none_ts06
```

3. Register SITL launch targets in:

- `<PX4 repo>/src/modules/simulation/simulator_mavlink/CMakeLists.txt`

This repo's `px4/README.md` contains ready-to-copy `add_custom_target(...)` blocks for `none_x8` and `none_ts06`.

4. Rebuild PX4 SITL.

If you are adding a completely new vehicle beyond `x8` and `ts06`, repeat the same pattern:

1. create a new PX4 airframe file under `px4/airframes/`
2. assign it a new autostart ID
3. add that airframe to PX4's airframe CMake list
4. add a matching `none_<name>` SITL target in PX4's simulator MAVLink CMake file
5. add the matching simulator-side vehicle definition under `src/vehicles/<name>/`
6. register the simulator vehicle in `src/vehicles/vehicle_catalog.py`

For the full PX4-side instructions, see `px4/README.md`.

### 2. Start the Python simulator

From this repo root:

```bash
SIM_VEHICLE_MODEL=iris SIM_MAVLINK_BIND_PORT=4560 python src/main.py
```

Useful environment variables:

- `SIM_VEHICLE_MODEL=x8|iris|ts06`
- `SIM_MAVLINK_BIND_HOST` default `0.0.0.0`
- `SIM_MAVLINK_BIND_PORT` default `4560`
- `SIM_GPS_LAT` / `SIM_GPS_LON` / `SIM_GPS_ALT` for GPS origin
- Legacy GPS names `SIM_GPS_ORIGIN_LAT` / `SIM_GPS_ORIGIN_LON` / `SIM_GPS_ORIGIN_ALT` are still accepted

### 3. Start PX4 SITL

Example for the custom tailsitter airframe:

```bash
cd ~/PX4-Autopilot
make px4_sitl none_ts06
```

PX4 connects to the simulator through MAVLink HIL and uses the simulated sensor stream for EKF and control.

### 4. Optional ground-truth output

Ground-truth output can be exposed independently of the noisy GPS sensor stream.

Modes:

- `SIM_GT_OUTPUT_MODE=websocket` default
- `SIM_GT_OUTPUT_MODE=flightgear_udp`
- `SIM_GT_OUTPUT_MODE=off`

Websocket options:

- `SIM_GT_OUTPUT_RATE_HZ` default `30.0`
- `SIM_GT_WS_HOST` default `0.0.0.0`
- `SIM_GT_WS_PORT` default `8765`

FlightGear UDP options:

- `SIM_FG_UDP_HOST` default `127.0.0.1`
- `SIM_FG_UDP_PORT` default `5503`

Example:

```bash
SIM_VEHICLE_MODEL=ts06 SIM_GT_OUTPUT_MODE=websocket python src/main.py
```

### 5. Visualize ground truth

Options:

1. Open `https://cmb87.github.io/` and connect to the websocket.
2. Use the local debug helper:

```bash
python src/test/ground_truth_ws_visualizer.py --host 127.0.0.1 --port 8765
```

## RL Usage

The simulator can run without PX4 through `rl.Px4SimEnv`.

Install RL dependencies:

- `pip install .[rl]`

Key behavior:

- No PX4 process is required
- The environment wraps `World` directly
- Observation is the 13-element simulator state by default
- Action space is 4D in `[-1, 1]`
- For `iris`, actions are remapped to motor commands in `[0, 1]`
- For fixed-wing models, action `0` is throttle and the remaining channels pass through as surface commands

Example:

```python
from rl import Px4SimEnv

env = Px4SimEnv(vehicle_model="iris", initial_state="airborne")
obs, info = env.reset(options={"altitude_m": 30.0, "speed_mps": 0.0})
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
env.close()
```

Launch behavior is controlled through `launch_mode`:

- `default` use the vehicle's configured launch behavior
- `catapult` force a rail launch
- `airborne` start in the air and skip rail dynamics
- `free` disable launch mechanics entirely

You can also enable websocket ground-truth output from RL runs for debugging:

```python
from rl import Px4SimEnv

env = Px4SimEnv(
    vehicle_model="x8",
    initial_state="airborne",
    enable_websocket=True,
    websocket_port=8765,
    websocket_system_id=1,
)
```

Examples:

- `PYTHONPATH=src python support/examples/x8_rl_websocket.py`
- `PYTHONPATH=src python support/examples/two_x8_two_websockets.py`

## Camera And Perception

This repo includes a geometry utility for camera projection in `src/perception/camera.py`.

What it gives you:

- a pinhole camera model through `perception.PinholeCamera`
- projection from NED world points into an ego-mounted camera
- visibility, pixel coordinates, and depth for each target point
- direct two-vehicle geometry by treating one vehicle as the camera carrier and the other as the target

What it does not do yet:

- no image renderer
- no rasterized scene generation
- no built-in camera sensor in the PX4 HIL loop

Use it when you already have world points or another vehicle state and want synthetic camera measurements.

Example:

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

### Render one vehicle from another vehicle's camera

For two vehicles, the camera model uses:

1. the target vehicle position as `point_ned`
2. the ego vehicle position as `ego_position_ned`
3. the ego vehicle attitude as `ego_quaternion_wxyz`

That means you can compute what vehicle A sees of vehicle B, and also what vehicle B sees of vehicle A, from their two simulator states.

Example:

```python
from perception import PinholeCamera

camera = PinholeCamera(width=640, height=480, fx=400.0, fy=400.0)

cam0_sees_1 = camera.project_world_point(
    point_ned=state1[0:3],
    ego_position_ned=state0[0:3],
    ego_quaternion_wxyz=state0[3:7],
)
cam1_sees_0 = camera.project_world_point(
    point_ned=state0[0:3],
    ego_position_ned=state1[0:3],
    ego_quaternion_wxyz=state1[3:7],
)
```

This returns, for each camera view:

- `visible` whether the other vehicle is inside the image and in front of the camera
- `pixel` the projected image coordinates
- `depth_m` the forward distance in the camera frame

There is already a minimal two-vehicle example in:

- `PYTHONPATH=src python support/examples/two_x8_two_websockets.py`

That example:

- runs two independent `Px4SimEnv` instances
- steps both vehicles forward
- projects each vehicle into the other vehicle's camera
- opens simple OpenCV windows that show the projected target location

There is also a unit test covering the same geometry in `src/test/test_camera_projection.py`.

### How to add a camera to your workflow

The usual pattern is:

1. run a `World` or `Px4SimEnv` step
2. read the ego state from `y[0:3]` and `y[3:7]`
3. define a target world point or another vehicle position
4. call `camera.project_world_point(...)`
5. feed the resulting synthetic measurement into your controller, estimator, or RL observation pipeline

If you want a true camera sensor path inside the simulator, the clean place to add it is alongside the other synthetic sensors in `src/sensors/`, then expose it either through the RL info dict or a side-channel publisher.

## Vehicle Models

Registered models live in `src/vehicles/vehicle_catalog.py`.

Current models:

- `iris`
- `x8`
- `ts06`

Vehicle-specific code lives under `src/vehicles/<name>/`.

`src/dynamics/world.py` stays model-agnostic and resolves:

- parameter definitions
- force model lists
- default initial state

### Add a new vehicle

1. Create `src/vehicles/my_uav/`
2. Add:
   - `parameters.py`
   - `forces.py`
   - `initial_state.py`
   - `definition.py`
3. Expose:
   - `make_parameters()`
   - `make_force_models(parameters)`
   - `make_initial_state(config)`
4. Register the vehicle in `src/vehicles/vehicle_catalog.py`
5. Run:

```bash
PYTHONPATH=src pytest src/test/test_vehicle_catalog.py
```

### Rail launch

Rail launch can be enabled per vehicle. Relevant parameters live in each vehicle's `parameters.py`:

- `rail_launch_enabled`
- `rail_dir_ned`
- `rail_start_ned`
- `rail_length`
- `rail_pull_max`

Runtime launch delay after arm is controlled by:

- `SIM_CATAPULT_LAUNCH_COUNTDOWN_S` default `3.0`

## Docker

Build the image:

```bash
./docker/build.sh
```

Run the default compose setup:

```bash
docker compose -f docker/docker-compose.yaml up
```

The compose file uses host networking so a local PX4 SITL instance can connect without additional port mapping.

Main settings in `docker/docker-compose.yaml`:

- `SIM_VEHICLE_MODEL`
- `SIM_MAVLINK_BIND_HOST`
- `SIM_MAVLINK_BIND_PORT`
- `SIM_GT_OUTPUT_MODE`
- `SIM_GT_WS_PORT`

Stop it with:

```bash
docker compose -f docker/docker-compose.yaml down
```

## Multi-UAV Support

The repository also includes a multi-UAV bridge and viewer.

See:

- `support/docs/src/multi-uav.md`
- `support/tools/run_multi.sh`
- `support/tools/viz/drone_viewer.html`

## Tests

Run the non-interactive test suite locally with:

```bash
PYTHONPATH=src pytest src/test
```

Interactive animation scripts live in `src/test/` too, but they are excluded from normal pytest collection through `pytest.ini`.

## Technical Documentation

Full technical documentation source:

- `support/docs/src/architecture.tex`

Build the PDF:

```bash
make -C support/docs/src
```

Output:

- `support/docs/build/architecture.pdf`

## Notes

- Sensor noise is enabled by default to better match jMAVSim-like behavior.
- The IMU path reconstructs accelerometer input from body-velocity derivative plus transport term to keep EKF behavior consistent.
