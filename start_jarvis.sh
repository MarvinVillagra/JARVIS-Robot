#!/bin/bash

# Define User and Workspace Paths explicitly
ORANGEPI_HOME="/home/orangepi"
ROS2_WS_PATH="${ORANGEPI_HOME}/ros2_ws"
LOG_DIR="${ROS2_WS_PATH}/logs"
JARVIS_CONTROL_PKG="jarvis_control"

echo "=== JARVIS AUTO-STARTUP ===" > "${LOG_DIR}/jarvis_startup.log" # Initial log
date >> "${LOG_DIR}/jarvis_startup.log"

# Ensure the log directory exists
mkdir -p "${LOG_DIR}"

echo "Sourcing ROS 2 environment..." >> "${LOG_DIR}/jarvis_startup.log"
source "/opt/ros/humble/setup.bash"
if [ $? -eq 0 ]; then
    echo "ROS 2 sourced successfully." >> "${LOG_DIR}/jarvis_startup.log"
else
    echo "ERROR: Failed to source ROS 2. Exiting." >> "${LOG_DIR}/jarvis_startup.log"
    exit 1
fi

echo "Sourcing workspace..." >> "${LOG_DIR}/jarvis_startup.log"
source "${ROS2_WS_PATH}/install/setup.bash"
if [ $? -eq 0 ]; then
    echo "Workspace sourced successfully." >> "${LOG_DIR}/jarvis_startup.log"
else
    echo "ERROR: Failed to source workspace. Make sure it has been built. Exiting." >> "${LOG_DIR}/jarvis_startup.log"
    exit 1
fi

# Wait a few seconds for system services (like I2C) to be fully ready
sleep 5

echo "Launching motor controller..." >> "${LOG_DIR}/jarvis_startup.log"
ros2 run "${JARVIS_CONTROL_PKG}" motor_controller > "${LOG_DIR}/motor_controller.log" 2>&1 &
MOTOR_CONTROLLER_PID=$!
echo "Motor controller launched with PID ${MOTOR_CONTROLLER_PID}." >> "${LOG_DIR}/jarvis_startup.log"

sleep 2 # Give the controller a moment to initialize

echo "Launching PS5 publisher..." >> "${LOG_DIR}/jarvis_startup.log"
ros2 run "${JARVIS_CONTROL_PKG}" ps5_publisher > "${LOG_DIR}/ps5_publisher.log" 2>&1 &
PS5_PUBLISHER_PID=$!
echo "PS5 publisher launched with PID ${PS5_PUBLISHER_PID}." >> "${LOG_DIR}/jarvis_startup.log"

echo "JARVIS core nodes launched. Monitoring PIDs." >> "${LOG_DIR}/jarvis_startup.log"
echo "Motor Controller PID: ${MOTOR_CONTROLLER_PID}" >> "${LOG_DIR}/jarvis_startup.log"
echo "PS5 Publisher PID: ${PS5_PUBLISHER_PID}" >> "${LOG_DIR}/jarvis_startup.log"

# This will keep the script alive as long as the child processes are running.
# If one crashes, the script might exit depending on shell behavior with `wait`.
# For true daemon behavior and auto-restart, systemd is better.
wait ${MOTOR_CONTROLLER_PID} ${PS5_PUBLISHER_PID}

EXIT_STATUS=$?
echo "Nodes have exited or script was terminated. Exit status: ${EXIT_STATUS}" >> "${LOG_DIR}/jarvis_startup.log"
date >> "${LOG_DIR}/jarvis_startup.log"
echo "=== JARVIS SHUTDOWN / END OF SCRIPT ===" >> "${LOG_DIR}/jarvis_startup.log"

exit ${EXIT_STATUS}
