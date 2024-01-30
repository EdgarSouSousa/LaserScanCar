#!/bin/bash

# Terminal 1
mate-terminal -- bash -c '
source /opt/ros/iron/setup.bash
source install/setup.bash
sudo chmod 777 /dev/ttyUSB0
ros2 launch ldlidar_stl_ros2 ld06.launch.py
' &
 
# Terminal 2
mate-terminal -- bash -c '
source /opt/ros/iron/setup.bash
source install/setup.bash
ros2 run laser_consumer laser_consumer
' &

