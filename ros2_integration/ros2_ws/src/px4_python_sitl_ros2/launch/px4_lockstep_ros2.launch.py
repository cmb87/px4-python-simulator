from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_ns = LaunchConfiguration("sim_ns")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim_ns",
                default_value="/sim",
                description="Namespace prefix for simulator topics",
            ),
            Node(
                package="px4_python_sitl_ros2",
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
                        "gps_origin_lat": 48.35386539065191,
                        "gps_origin_lon": 11.78159133408772,
                        "gps_origin_alt": 447.0,
                        "frame_id": "map",
                        "child_frame_id": "base_link",
                        "tf_topic": [sim_ns, "/tf"],
                        "gps_fix_topic": [sim_ns, "/gps/fix"],
                        "gps_vel_topic": [sim_ns, "/gps/vel"],
                        "actuators_topic": [sim_ns, "/actuators"],
                        "aero_topic": [sim_ns, "/aero"],
                        "sysid_topic": [sim_ns, "/px4_sysid"],
                    }
                ],
            )
        ]
    )
