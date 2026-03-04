cmake_minimum_required(VERSION 3.8)
project(websocket_fpv_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(JPEG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(websocket_fpv_bridge_node src/websocket_fpv_bridge_node.cpp)
target_compile_features(websocket_fpv_bridge_node PUBLIC cxx_std_17)
ament_target_dependencies(
  websocket_fpv_bridge_node
  rclcpp
  sensor_msgs
)
target_include_directories(websocket_fpv_bridge_node PRIVATE ${JPEG_INCLUDE_DIRS})
target_link_libraries(websocket_fpv_bridge_node Boost::system ${JPEG_LIBRARIES})

install(TARGETS websocket_fpv_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
