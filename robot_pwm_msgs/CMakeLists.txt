cmake_minimum_required(VERSION 3.8)
project(robot_pwm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED) # Added for message generation

# Add your .msg files here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorPwm.msg"
  # Add other .msg files if you have more, e.g., "msg/AnotherMessage.msg"
)

ament_package()
