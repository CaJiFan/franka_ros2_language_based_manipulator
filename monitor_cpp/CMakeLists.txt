cmake_minimum_required(VERSION 3.8)
project(monitor_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(franka_msgs REQUIRED)
# FORCE CMake to look in the right place (Update path to match YOUR home dir)
# include_directories("/home/userlab/cjimenez/ros2_ws/install/franka_msgs/include") 
# include_directories("/home/userlab/cjimenez/ros2_ws/install/franka_msgs/include/franka_msgs") 

link_directories("/home/userlab/cjimenez/ros2_ws/install/franka_msgs/lib")

find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nlohmann_json REQUIRED)

# Create the executable
add_executable(pose_monitor src/pose_monitor.cpp)

# Link libraries
ament_target_dependencies(pose_monitor
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# Install it (so 'ros2 run' can find it)
install(TARGETS
  pose_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Add the new MoveIt node
add_executable(move_to_pose src/move_to_pose.cpp)
ament_target_dependencies(move_to_pose
  rclcpp
  rclcpp_action
  franka_msgs
  moveit_ros_planning_interface
  geometry_msgs
  std_msgs
)

target_link_libraries(move_to_pose_node nlohmann_json::nlohmann_json)

install(TARGETS
  move_to_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()