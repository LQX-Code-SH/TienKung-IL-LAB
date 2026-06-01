cmake_minimum_required(VERSION 3.8)
project(zmq_image_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_executable(zmq_image_bridge zmq_image_bridge.cpp)
target_include_directories(zmq_image_bridge PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(zmq_image_bridge
    ${ZMQ_LIBRARIES}
    pthread
)
ament_target_dependencies(zmq_image_bridge rclcpp sensor_msgs image_transport)

install(TARGETS zmq_image_bridge DESTINATION lib/${PROJECT_NAME})
