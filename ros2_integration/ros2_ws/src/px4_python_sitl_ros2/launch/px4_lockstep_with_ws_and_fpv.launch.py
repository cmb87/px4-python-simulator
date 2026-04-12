from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    sim_ns = LaunchConfiguration("sim_ns")

    mavlink_bind_host = LaunchConfiguration("mavlink_bind_host")
    mavlink_bind_port = LaunchConfiguration("mavlink_bind_port")
    vehicle_model = LaunchConfiguration("vehicle_model")
    rate_hz = LaunchConfiguration("rate_hz")
    ts04_pitch90_start = LaunchConfiguration("ts04_pitch90_start")
    gps_origin_lat = LaunchConfiguration("gps_origin_lat")
    gps_origin_lon = LaunchConfiguration("gps_origin_lon")
    gps_origin_alt = LaunchConfiguration("gps_origin_alt")
    frame_id = LaunchConfiguration("frame_id")
    child_frame_id = LaunchConfiguration("child_frame_id")

    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")

    fpv_ws_host = LaunchConfiguration("fpv_ws_host")
    fpv_ws_port = LaunchConfiguration("fpv_ws_port")
    fpv_ws_frame_id = LaunchConfiguration("fpv_ws_frame_id")

    tf_topic = [sim_ns, "/tf"]
    gps_fix_topic = [sim_ns, "/gps/fix"]
    gps_vel_topic = [sim_ns, "/gps/vel"]
    actuators_topic = [sim_ns, "/actuators"]
    aero_topic = [sim_ns, "/aero"]
    sysid_topic = [sim_ns, "/px4_sysid"]
    image_topic = [sim_ns, "/image"]

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim_ns", default_value="/sim"),
            DeclareLaunchArgument("mavlink_bind_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("mavlink_bind_port", default_value="4560"),
            DeclareLaunchArgument("vehicle_model", default_value="iris"),
            DeclareLaunchArgument("rate_hz", default_value="250"),
            DeclareLaunchArgument("ts04_pitch90_start", default_value="true"),
            DeclareLaunchArgument("gps_origin_lat", default_value="48.35386539065191"),
            DeclareLaunchArgument("gps_origin_lon", default_value="11.78159133408772"),
            DeclareLaunchArgument("gps_origin_alt", default_value="447.0"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("child_frame_id", default_value="base_link"),
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("ws_port", default_value="8765"),
            DeclareLaunchArgument("fpv_ws_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("fpv_ws_port", default_value="9001"),
            DeclareLaunchArgument("fpv_ws_frame_id", default_value="camera"),
            Node(
                package="px4_python_sitl_ros2",
                executable="px4_lockstep_ros2_node",
                name="px4_lockstep_ros2_node",
                output="screen",
                parameters=[
                    {
                        "mavlink_bind_host": mavlink_bind_host,
                        "mavlink_bind_port": mavlink_bind_port,
                        "vehicle_model": vehicle_model,
                        "rate_hz": rate_hz,
                        "ts04_pitch90_start": ts04_pitch90_start,
                        "gps_origin_lat": gps_origin_lat,
                        "gps_origin_lon": gps_origin_lon,
                        "gps_origin_alt": gps_origin_alt,
                        "frame_id": frame_id,
                        "child_frame_id": child_frame_id,
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
                                "ws_host": ws_host,
                                "ws_port": ws_port,
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
            Node(
                package="websocket_fpv_ros2_bridge",
                executable="websocket_fpv_bridge_node",
                name="websocket_fpv_bridge_node",
                output="screen",
                parameters=[
                    {
                        "host": fpv_ws_host,
                        "port": fpv_ws_port,
                        "frame_id": fpv_ws_frame_id,
                        "image_topic": image_topic,
                    }
                ],
            ),
        ]
    )
