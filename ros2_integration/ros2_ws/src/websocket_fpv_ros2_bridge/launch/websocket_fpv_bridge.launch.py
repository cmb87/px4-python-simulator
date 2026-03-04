from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_ns = LaunchConfiguration("sim_ns")
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim_ns",
                default_value="/sim",
                description="Namespace prefix for simulator topics",
            ),
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="9001"),
            DeclareLaunchArgument("frame_id", default_value="camera"),
            Node(
                package="websocket_fpv_ros2_bridge",
                executable="websocket_fpv_bridge_node",
                name="websocket_fpv_bridge_node",
                output="screen",
                parameters=[
                    {
                        "host": host,
                        "port": port,
                        "frame_id": frame_id,
                        "image_topic": [sim_ns, "/image"],
                    }
                ],
            ),
        ]
    )
