cmake_minimum_required(VERSION 3.5)
project(moonbot_teleop)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moonbot_custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(./include/)

add_executable(moonbot_teleop_joy_node src/moonbot_teleop_joy.cpp)
ament_target_dependencies(moonbot_teleop_joy_node 
  rclcpp 
  sensor_msgs 
  std_msgs 
  moonbot_custom_interfaces
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    config
    include
    src
  DESTINATION
    share/${PROJECT_NAME}/
)

install(TARGETS
  moonbot_teleop_joy_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
