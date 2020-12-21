#
# Write here your robot name in order to build it..
#

set(all_src_targets)

set(robot_name bolt)

set(robot_properties_package_name
        robot_properties_bolt)

set(dynamic_graph_manager_yaml_file
        dynamic_graph_manager/dgm_parameters_bolt.yaml)


# create library
add_library(dgm_${robot_name} SHARED dgm_${robot_name}.cpp)
# Add the include dependencies.
target_include_directories(
        dgm_${robot_name}
        PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
# Link the dependencies.
ament_target_dependencies(dgm_bolt mim_msgs)
target_link_libraries(dgm_bolt ament_index_cpp::ament_index_cpp)
target_link_libraries(dgm_bolt blmc_drivers::blmc_drivers)
target_link_libraries(dgm_bolt blmc_robots::blmc_robots)
target_link_libraries(dgm_bolt real_time_tools::real_time_tools)
target_link_libraries(dgm_bolt bolt::bolt)
target_link_libraries(dgm_bolt dynamic_graph_manager::dynamic_graph_manager)
target_link_libraries(dgm_bolt yaml_utils::yaml_utils)
target_link_libraries(dgm_bolt ${catkin_LIBRARIES})

# Export the target.
list(APPEND all_src_targets dgm_${robot_name})



set(executable_name dg_main_${robot_name})
# Robot executables.
add_executable(${executable_name} robot_executable/${robot_name}.cpp)
# Add the include dependencies.
target_include_directories(
        ${executable_name}
        PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        ${dynamic_graph_manager_INCLUDE_DIR} $<INSTALL_INTERFACE:include>)
## Link the dependencies.
target_link_libraries(${executable_name} dgm_${robot_name})
# Some compile definitions
target_compile_definitions(
        ${executable_name} PUBLIC ROBOT_PROPERTIES_PACKAGE_NAME="${robot_properties_package_name}"
        ROBOT_PROPERTIES_YAML_PATH="${dynamic_graph_manager_yaml_file}")
list(APPEND all_src_targets ${executable_name})



#
# Install and Export the libraries.
#
install(
  TARGETS ${all_src_targets}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)