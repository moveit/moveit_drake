cmake_minimum_required(VERSION 3.22)
project(moveit_drake LANGUAGES CXX)

list(APPEND CMAKE_PREFIX_PATH /opt/drake/lib)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)

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

generate_parameter_library(drake_moveit_parameters res/drake_moveit.yaml)

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

# ktopt planning plugin
add_library(moveit_ktopt_planner_plugin SHARED src/ktopt_planner_manager.cpp
                                               src/ktopt_planning_context.cpp)
target_link_libraries(moveit_ktopt_planner_plugin drake::drake
                      drake_moveit_parameters)

pluginlib_export_plugin_description_file(moveit_core
                                         ktopt_interface_plugin_description.xml)

install(
  TARGETS moveit_ktopt_planner_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/ktopt_interface moveit_drake)

ament_target_dependencies(moveit_ktopt_planner_plugin rclcpp pluginlib
                          moveit_core moveit_msgs EIGEN3)
ament_export_dependencies(moveit_core drake generate_parameter_library)

add_subdirectory(demo)

ament_package()
