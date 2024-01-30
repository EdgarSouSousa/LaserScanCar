#!/bin/bash

# Source ROS setup.bash
source /opt/ros/iron/setup.bash

# Source your specific setup.bash
source install/setup.bash

# Build with colcon
colcon build --symlink-install
