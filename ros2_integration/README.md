# ROS2 Integration Example

This folder provides a ROS2 example node that keeps the existing MAVLink lockstep behavior (TCP + HIL messages) and publishes simulator outputs via ROS2 topics.

## What it does

- Connects MAVLink over TCP (`tcpin:<host>:<port>`) using `pymavlink`
- Runs the simulator in lockstep at fixed rate (default 250 Hz)
- Sends MAVLink messages equivalent to current simulator behavior:
  - `HIL_SENSOR`
  - `HIL_GPS`
  - optional `HIL_STATE_QUATERNION`
  - `SYSTEM_TIME`
  - heartbeat
- Publishes ROS2 outputs (ENU conventions):
  - `/sim/tf` (`tf2_msgs/TFMessage`, `map` -> `base_link`)
  - `/sim/gps/fix` (`sensor_msgs/NavSatFix`)
  - `/sim/gps/vel` (`geometry_msgs/TwistStamped`, ENU world-frame velocity)

The workspace also includes a C++ websocket bridge package that subscribes to these simulator topics and serves JSON over websocket.

No WebSocket ground-truth stream and no UDP transfer-alignment stream are used in this example.

## Package location

ROS2 package root:

`ros2_integration/ros2_ws/src/px4_python_sitl_ros2`

Websocket bridge package:

`ros2_integration/ros2_ws/src/px4_sitl_ws_bridge`

## Build

From repository root:

```bash
source /opt/ros/<distro>/setup.bash
python -m pip install --upgrade pip setuptools wheel build
python -m pip install -e . || python -m pip install .
colcon build --base-paths ros2_integration/ros2_ws/src
```

Then source the overlay:

```bash
source install/setup.bash
```

## Docker dev setup

From repository root:

```bash
docker compose -f ros2_integration/docker-compose.yaml up -d
docker exec -it ros2_<distro> bash
```

Inside the container:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
bash ros2_integration/ros2_ws/setup_ros2_dev.sh --build
source ros2_integration/ros2_ws/install/setup.bash
```

`setup_ros2_dev.sh` installs apt prerequisites (`python3-pip`, `python3-rosdep`, `python3-colcon-common-extensions`, `python3-vcstool`, `libwebsocketpp-dev`, `libboost-system-dev`), upgrades Python build tooling, installs `px4-python-sitl`, installs Python runtime deps (`numpy`, `pymavlink`), and installs ROS2 package dependencies via `rosdep`.

If `pip install -e .` fails with a missing `build_editable` hook, run the upgrade line above first or rely on the fallback `python -m pip install .`.

## Run

```bash
ros2 launch px4_python_sitl_ros2 px4_lockstep_ros2.launch.py
```

Launch simulator + websocket bridge together:

```bash
ros2 launch px4_python_sitl_ros2 px4_lockstep_with_ws_bridge.launch.py
```

Or run directly:

```bash
ros2 run px4_python_sitl_ros2 px4_lockstep_ros2_node
```

Run websocket bridge:

```bash
ros2 run px4_sitl_ws_bridge px4_ros2_ws_bridge_node
```

## Important parameters

- `mavlink_bind_host` (default `0.0.0.0`)
- `mavlink_bind_port` (default `4560`)
- `vehicle_model` (`iris`, `x8`, `ts04`)
- `rate_hz` (default `250`)
- `frame_id` (default `map`)
- `child_frame_id` (default `base_link`)

## Notes

- Install `px4-python-sitl` in the same Python environment used by ROS2 so `px4_python_sitl.vehicle.world` can be imported.
- ENU output is derived from simulator NED/FRD state:
  - position: `[e, n, -d]`
  - body vectors FRD -> FLU: `[x, -y, -z]`
