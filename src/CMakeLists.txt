###############################################
# Declare the main library #
###############################################

add_library(dgm_bolt SHARED dgm_bolt.cpp)
target_link_libraries(dgm_bolt ${catkin_LIBRARIES})
if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(dgm_bolt rt pthread)
endif()

###############################
# Dclare the robot executable #
###############################

set(executable_name dg_main_bolt)

add_executable(${executable_name}
    robot_executable/bolt.cpp
)
target_link_libraries(${executable_name}
    dgm_bolt
    ${catkin_LIBRARIES}
)
set(executable_name dg_main_bolt)

set(robot_config_file "${robot_properties_bolt_SOURCE_PREFIX}/config/dgm_parameters_bolt.yaml")
if(NOT EXISTS ${robot_config_file})
    message(FATAL_ERROR "Error, cannot find the DGM config file: ${robot_config_file}")
endif()
target_compile_definitions(${executable_name} PUBLIC
                            YAML_PARAMS="${robot_config_file}")
