cmake_minimum_required (VERSION 3.10.2)
project (rsbp_ros2_adapter LANGUAGES CXX)

find_package(Boost COMPONENTS system REQUIRED)

add_executable(rsbp_ros2_adapter
              demo.cpp
              )

target_link_libraries(rsbp_ros2_adapter
                    ${Boost_LIBRARIES}       
                      pcap
                      pthread
)
