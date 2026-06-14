#!/bin/bash

# ROS2 workspace 경로 (자신의 워크스페이스 경로로 바꾸세요)
WORKSPACE=~/sim_ws

cd $WORKSPACE

echo "Building ROS2 workspace..."
colcon build --packages-select project_f1_tenth

echo "Sourcing setup.bash..."
source install/setup.bash

echo "Running ROS2 node..."
ros2 run project_f1_tenth project_f1_tenth
