from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="px4_python_sitl_ros2_example",
                executable="px4_lockstep_ros2_node",
                name="px4_lockstep_ros2_node",
                output="screen",
                parameters=[
                    {
                        "mavlink_bind_host": "0.0.0.0",
                        "mavlink_bind_port": 4560,
                        "vehicle_model": "iris",
                        "rate_hz": 250,
                        "ts04_pitch90_start": True,
                        "ts04_motor_map": "0,1,2,3",
                        "gps_origin_lat": 47.397742,
                        "gps_origin_lon": 8.545594,
                        "gps_origin_alt": 470.0,
                        "frame_id": "map",
                        "child_frame_id": "base_link",
                    }
                ],
            )
        ]
    )
