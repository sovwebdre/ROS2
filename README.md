# ROS2
ROS2 codes from Robotics Laboratory

## Setup
Firstly you need to set up the robot
- power it up
- turn buttons on
- connect Lidar
- connect esp32
- 
## Running

Launch Lidar node:

    ros2 launch sllidar_ros2 sllidar_c1_launch.py

Run micro-ROS agent:

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB1

after that reset the esp32
    
Launch it:

    ros2 launch slam_toolbox online_async_launch.py

