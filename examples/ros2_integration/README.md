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
  - `/tf` (`map` -> `base_link`)
  - `/sim/odom` (`nav_msgs/Odometry`, twist in body frame)
  - `/sim/gps/fix` (`sensor_msgs/NavSatFix`)
  - `/sim/gps/vel` (`geometry_msgs/TwistStamped`, ENU world-frame velocity)

No WebSocket ground-truth stream and no UDP transfer-alignment stream are used in this example.

## Package location

ROS2 package root:

`examples/ros2_integration/px4_python_sitl_ros2_example`

## Build

From repository root:

```bash
source /opt/ros/<distro>/setup.bash
colcon build --packages-select px4_python_sitl_ros2_example --base-paths examples/ros2_integration
```

Then source the overlay:

```bash
source install/setup.bash
```

## Run

```bash
ros2 launch px4_python_sitl_ros2_example px4_lockstep_ros2.launch.py
```

Or run directly:

```bash
ros2 run px4_python_sitl_ros2_example px4_lockstep_ros2_node
```

## Important parameters

- `mavlink_bind_host` (default `0.0.0.0`)
- `mavlink_bind_port` (default `4560`)
- `vehicle_model` (`iris`, `x8`, `ts04`)
- `rate_hz` (default `250`)
- `frame_id` (default `map`)
- `child_frame_id` (default `base_link`)

## Notes

- The node imports simulator code from this repository (`src/` and `src/vehicle`) at runtime.
- ENU output is derived from simulator NED/FRD state:
  - position: `[e, n, -d]`
  - body vectors FRD -> FLU: `[x, -y, -z]`
