# rs-bp-ros2-adapter
This is a small adapter for interfacing [Robosense Bpearl](https://www.robosense.ai/rslidar/rs-bpearl) with ROS2.
It is not intended to replace Robosense's own ROS2 drivers, but to showcase a simple ROS2 module/package. 

# What does it do
This program opens an UDP socket and blocks execution until a message from the Robosense Lidar is received. Then it extracts the point cloud information from the Robosenses custom protocol message and publishes the point cloud to ROS2.

