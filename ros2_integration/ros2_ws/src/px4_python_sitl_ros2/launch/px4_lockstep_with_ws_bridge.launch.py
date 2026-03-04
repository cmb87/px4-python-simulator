from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    sim_ns = LaunchConfiguration("sim_ns")
    tf_topic = [sim_ns, "/tf"]
    gps_fix_topic = [sim_ns, "/gps/fix"]
    gps_vel_topic = [sim_ns, "/gps/vel"]
    actuators_topic = [sim_ns, "/actuators"]
    aero_topic = [sim_ns, "/aero"]
    sysid_topic = [sim_ns, "/px4_sysid"]

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
                        "gps_origin_lat": 47.397742,
                        "gps_origin_lon": 8.545594,
                        "gps_origin_alt": 470.0,
                        "frame_id": "map",
                        "child_frame_id": "base_link",
                        "tf_topic": tf_topic,
                        "gps_fix_topic": gps_fix_topic,
                        "gps_vel_topic": gps_vel_topic,
                        "actuators_topic": actuators_topic,
                        "aero_topic": aero_topic,
                        "sysid_topic": sysid_topic,
                    }
                ],
            ),
            ComposableNodeContainer(
                name="px4_ws_bridge_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                output="screen",
                composable_node_descriptions=[
                    ComposableNode(
                        package="px4_sitl_ws_bridge",
                        plugin="Px4Ros2WsBridgeNode",
                        name="px4_ros2_ws_bridge_node",
                        extra_arguments=[{"use_intra_process_comms": True}],
                        parameters=[
                            {
                                "ws_host": "0.0.0.0",
                                "ws_port": 8765,
                                "tf_topic": tf_topic,
                                "gps_fix_topic": gps_fix_topic,
                                "gps_vel_topic": gps_vel_topic,
                                "actuators_topic": actuators_topic,
                                "aero_topic": aero_topic,
                                "sysid_topic": sysid_topic,
                            }
                        ],
                    )
                ],
            ),
        ]
    )
