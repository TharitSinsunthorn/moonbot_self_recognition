cmake_minimum_required(VERSION 3.5)
project(moonbot_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(moonbot_description REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moonbot_custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/IK_node.py
    scripts/cmd_manager_node.py
    scripts/maincmd.py
    scripts/real_action.py
    scripts/real_pub.py
    scripts/sim_action.py
    scripts/sim_pub.py
    scripts/modular.py
    scripts/singlelimb/RF.py
    scripts/singlelimb/LF.py
    scripts/singlelimb/LR.py
    scripts/singlelimb/RR.py
    scripts/launch_detection.py
    connection/RF_con.py
    connection/LF_con.py
    connection/LR_con.py
    connection/RR_con.py

  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
