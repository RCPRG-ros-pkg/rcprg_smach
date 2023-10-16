cmake_minimum_required(VERSION 3.8)
project(rcprg_smach)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(move_base_msgs REQUIRED) port to ROS2
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
    rclpy
    actionlib_msgs
    std_msgs
    geometry_msgs
    # move_base_msgs
    tf2
    tf2_ros
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msgs and services"
#   DEPENDENCIES ${dependencies}
# )

ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Uncomment if you have node executables
# install(TARGETS your_node_targets
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()