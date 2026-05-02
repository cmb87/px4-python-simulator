cmake_minimum_required(VERSION 3.14)
project(x8_simulation VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

include(FetchContent)

# Eigen for math
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(eigen)

# nlohmann/json for JSON
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.11.2
)
FetchContent_MakeAvailable(json)

# websocketpp for WebSockets
FetchContent_Declare(
  websocketpp
  GIT_REPOSITORY https://github.com/zaphoyd/websocketpp.git
  GIT_TAG 0.8.2
)
FetchContent_MakeAvailable(websocketpp)

# asio (dependency for websocketpp)
FetchContent_Declare(
  asio
  GIT_REPOSITORY https://github.com/chriskohlhoff/asio.git
  GIT_TAG asio-1-24-0
)
FetchContent_MakeAvailable(asio)

# MAVLink headers
FetchContent_Declare(
  mavlink
  GIT_REPOSITORY https://github.com/mavlink/c_library_v2.git
  GIT_TAG master
)
FetchContent_MakeAvailable(mavlink)

add_executable(x8_sim
    src/main.cpp
    src/x8/forces.cpp
    src/dynamics/dynamics.cpp
    src/sensors/sensors.cpp
    src/networking/websocket_publisher.cpp
    src/networking/mavlink_interface.cpp
)

target_include_directories(x8_sim PRIVATE 
    src
    ${eigen_SOURCE_DIR}
    ${json_SOURCE_DIR}/single_include
    ${websocketpp_SOURCE_DIR}
    ${asio_SOURCE_DIR}/asio/include
    ${mavlink_SOURCE_DIR}
)

target_link_libraries(x8_sim PRIVATE 
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    Threads::Threads
)

target_compile_definitions(x8_sim PRIVATE 
    ASIO_STANDALONE
    _WEBSOCKETPP_CPP11_STL_
)
