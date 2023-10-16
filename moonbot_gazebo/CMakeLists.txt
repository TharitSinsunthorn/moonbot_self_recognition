cmake_minimum_required(VERSION 3.8)
project(moonbot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(moonbot_description REQUIRED)
find_package(moonbot_control REQUIRED)
find_package(gazebo_ros2_control)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  DIRECTORY
    launch
	  worlds
	  models

  DESTINATION
    share/${PROJECT_NAME}/
)


install(
  PROGRAMS
    scripts/joint_trajectory_test.py
    scripts/action_joint.py
    scripts/limb_joint.py
    scripts/emergency.py
    scripts/RF.py
    scripts/RR.py
    scripts/LR.py
    scripts/LF.py
    scripts/modular.py

  DESTINATION
    lib/${PROJECT_NAME}
)


ament_package()
