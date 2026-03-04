cmake_minimum_required(VERSION 3.8)
project(px4_sitl_ws_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(px4_ros2_ws_bridge_component SHARED src/px4_ros2_ws_bridge_node.cpp)
target_compile_definitions(px4_ros2_ws_bridge_component PRIVATE PX4_SITL_WS_BRIDGE_COMPONENT_ONLY)
target_compile_features(px4_ros2_ws_bridge_component PUBLIC cxx_std_17)
ament_target_dependencies(
  px4_ros2_ws_bridge_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
)
target_link_libraries(px4_ros2_ws_bridge_component Boost::system)
rclcpp_components_register_nodes(px4_ros2_ws_bridge_component "Px4Ros2WsBridgeNode")

add_executable(px4_ros2_ws_bridge_node src/px4_ros2_ws_bridge_node.cpp)
target_compile_features(px4_ros2_ws_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(
  px4_ros2_ws_bridge_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
)
target_link_libraries(px4_ros2_ws_bridge_node Boost::system)

install(
  TARGETS px4_ros2_ws_bridge_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS px4_ros2_ws_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()
