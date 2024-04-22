cmake_minimum_required(VERSION 3.22)
project(moveit_drake LANGUAGES CXX)

list(APPEND CMAKE_PREFIX_PATH /opt/drake/lib)
find_package(drake REQUIRED)

find_package(ament_cmake REQUIRED)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(class_loader REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(warehouse_ros REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    rclcpp
    rviz_visual_tools
    warehouse_ros)

include_directories(include)

add_library(moveit_diff_ik_plugin SHARED src/diff_ik_plugin.cpp)

ament_target_dependencies(moveit_diff_ik_plugin rclcpp pluginlib moveit_core
                          moveit_msgs EIGEN3)

target_link_libraries(moveit_diff_ik_plugin drake::drake)

add_subdirectory(demo)

ament_package()
