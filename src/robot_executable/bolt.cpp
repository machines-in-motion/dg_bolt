/**
 * \file bolt.cpp
 * \brief Execute the main program to control the bolt
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the TestBench8Motors class in a small demo.
 */

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dg_bolt/dgm_bolt.hpp"

int main(int, char* [])
{
    // Get the dynamic_graph_manager config file.
    std::string share_path = ament_index_cpp::get_package_share_directory(
        ROBOT_PROPERTIES_PACKAGE_NAME);
    std::string yaml_path = share_path + "/" + ROBOT_PROPERTIES_YAML_PATH;
    std::string yaml_path =
        "/home/edaneshmand/devel_ros2/workspace/src/robot_properties_bolt/src/"
        "robot_properties_bolt/robot_properties_bolt/dynamic_graph_manager/"
        "dgm_parameters_bolt.yaml";
    std::cout << "Loading paramters from " << yaml_path << std::endl;
    YAML::Node param = YAML::LoadFile(yaml_path);
    // Create the dgm.
    dg_bolt::DGMBolt dgm;

    // Initialize and run it.
    dgm.initialize(param);
    dgm.run();

    // Wait until ROS is shutdown.
    std::cout << "Wait for shutdown, press CTRL+C to close." << std::endl;
    dynamic_graph_manager::ros_spin();
    dynamic_graph_manager::ros_shutdown();
}
