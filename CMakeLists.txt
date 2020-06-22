cmake_minimum_required (VERSION 3.10.2)
project (rsbp_ros2_adapter LANGUAGES CXX)

find_package(Boost COMPONENTS system REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(rsbp_ros2_adapter
              demo.cpp
              )
target_include_directories(rsbp_ros2_adapter PUBLIC
${rclcpp_INCLUDE_DIRS}
${sensor_msgs_INCLUDE_DIRS}
)


target_link_libraries(rsbp_ros2_adapter
${Boost_LIBRARIES}       
pcap
pthread
${rclcpp_LIBRARIES}
${sensor_msgs_LIBRARIES}		      
)
