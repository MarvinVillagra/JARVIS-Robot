# JARVIS Robot (ROS 2 + PS5 Controller)

This repository contains all code for the JARVIS mobile robot project powered by ROS 2 Humble on a Raspberry Pi 4. It supports full wireless control using a PS5 controller, safe startup logic, and a dual-battery power system.

## Features
- Wireless PS5 Controller Input (Bluetooth)
-Motor Control via PCA9685 & TB6612FNG
- Separate power for Pi and motors
- Smooth PWM startup to prevent current spikes
- uto-launch on boot using `systemd`

## Wiring Summary
- 7.4V 2S 18650 battery powers motors
- 5V USB powers Pi
- Common GND is required

## Running
Manual start:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run robot_pwm_test motor_controller
ros2 run robot_pwm_test ps5_publisher

~/ros2_ws/start_jarvis.sh (currently working on it/not fuctional)

---

Author Marvin Villagra University of Texas at Tyler / Houston Community College


