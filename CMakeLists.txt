cmake_minimum_required (VERSION 3.10.2)
project (rsbp_ros2_adapter LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

add_executable(rsbp_ros2_adapter
              demo.cpp
              )
target_include_directories(rsbp_ros2_adapter PUBLIC
${rclcpp_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
)


set(CMAKE_VERBOSE_MAKEFILE FALSE)

target_link_libraries(rsbp_ros2_adapter      
pthread
dl
${rclcpp_LIBRARIES}
${sensor_msgs_LIBRARIES}
m		      
)
