/**
 * \file bolt.cpp
 * \brief Execute the main program to control the bolt
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the TestBench8Motors class in a small demo.
 */

#include "dg_bolt/dgm_bolt.hpp"

int main(int, char* [])
{
    std::cout << "Loading paramters from " << YAML_PARAMS << std::endl;
    YAML::Node param = YAML::LoadFile(YAML_PARAMS);
    dg_bolt::DGMBolt dgm;

    dgm.initialize(param);
    dgm.run();
    std::cout << "Wait for shutdown, press CTRL+C to close." << std::endl;
    ros::waitForShutdown();
}
