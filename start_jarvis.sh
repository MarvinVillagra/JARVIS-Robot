#!/bin/bash
echo "=== JARVIS STARTUP ==="
date

source /opt/ros/humble/setup.bash
echo "ROS 2 sourced"

source ~/ros2_ws/install/setup.bash
echo "Workspace sourced"

echo "Launching motor controller..."
ros2 run robot_pwm_test motor_controller > /home/flackods3/ros2_ws/log_motor.txt 2>&1 &

sleep 1

echo "Launching ps5_publisher..."
ros2 run robot_pwm_test ps5_publisher > /home/flackods3/ros2_ws/log_ps5.txt 2>&1

