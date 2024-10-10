add_executable(pipeline_testbench_example src/pipeline_testbench_main.cpp)
ament_target_dependencies(pipeline_testbench_example
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(constrained_planning_demo src/constrained_planning_demo.cpp)
ament_target_dependencies(constrained_planning_demo
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS pipeline_testbench_example constrained_planning_demo
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
