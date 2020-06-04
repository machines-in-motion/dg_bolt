/**
 * \file dgm_bolt.cpp
 * \brief The hardware wrapper of the bolt robot
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file defines the TestBench8Motors class.
 */

#include "dg_bolt/dgm_bolt.hpp"
#include <dynamic_graph_manager/ros_init.hh>

namespace dg_bolt
{
DGMBolt::DGMBolt()
{
    was_in_safety_mode_ = false;
}

DGMBolt::~DGMBolt()
{
}

void DGMBolt::initialize_hardware_communication_process()
{
    /**
     * Load the calibration parameters
     */
    bolt::Vector6d joint_index_to_zero;
    YAML::ReadParameter(params_["hardware_communication"]["calibration"],
                        "index_to_zero_angle",
                        zero_to_index_angle_from_file_);

    // get the hardware communication ros node handle
    ros::NodeHandle& ros_node_handle = dynamic_graph::ros_init(
        dynamic_graph::DynamicGraphManager::hw_com_ros_node_name_);

    /** initialize the user commands */
    ros_user_commands_.push_back(ros_node_handle.advertiseService(
        "calibrate_joint_position",
        &DGMBolt::calibrate_joint_position_callback,
        this));

    std::string network_id;
    YAML::ReadParameter(
        params_["hardware_communication"], "network_id", network_id);

    bolt_.initialize(network_id);
}

void DGMBolt::get_sensors_to_map(dynamic_graph::VectorDGMap& map)
{
    bolt_.acquire_sensors();

    /**
      * Joint data
      */
    map.at("joint_positions") = bolt_.get_joint_positions();
    map.at("joint_velocities") = bolt_.get_joint_velocities();
    map.at("joint_torques") = bolt_.get_joint_torques();
    map.at("joint_target_torques") = bolt_.get_joint_target_torques();
    map.at("joint_encoder_index") = bolt_.get_joint_encoder_index();

    /**
      * Additional data
      */
    map.at("slider_positions") = bolt_.get_slider_positions();

    /**
     * Robot status
     */
    dynamicgraph::Vector& map_motor_enabled = map.at("motor_enabled");
    dynamicgraph::Vector& map_motor_ready = map.at("motor_ready");
    dynamicgraph::Vector& map_motor_board_enabled =
        map.at("motor_board_enabled");
    dynamicgraph::Vector& map_motor_board_errors = map.at("motor_board_errors");
    const std::array<bool, BOLT_NB_MOTOR>& motor_enabled =
        bolt_.get_motor_enabled();
    const std::array<bool, BOLT_NB_MOTOR>& motor_ready =
        bolt_.get_motor_ready();
    const std::array<bool, BOLT_NB_MOTOR_BOARD>& motor_board_enabled =
        bolt_.get_motor_board_enabled();
    const std::array<int, BOLT_NB_MOTOR_BOARD>& motor_board_errors =
        bolt_.get_motor_board_errors();

    for (unsigned i = 0; i < BOLT_NB_MOTOR; ++i)
    {
        map_motor_enabled[i] = motor_enabled[i];
        map_motor_ready[i] = motor_ready[i];
    }
    for (unsigned i = 0; i < BOLT_NB_MOTOR_BOARD; ++i)
    {
        map_motor_board_enabled[i] = motor_board_enabled[i];
        map_motor_board_errors[i] = motor_board_errors[i];
    }
}

void DGMBolt::set_motor_controls_from_map(const dynamic_graph::VectorDGMap& map)
{
    try
    {
        // here we need to perform and internal copy. Otherwise the compilator
        // complains
        ctrl_joint_torques_ = map.at("ctrl_joint_torques");
        // Actually send the control to the robot
        bolt_.send_target_joint_torque(ctrl_joint_torques_);
    }
    catch (const std::exception& e)
    {
        rt_printf(
            "DGMBolt::set_motor_controls_from_map: "
            "Error sending controls, %s\n",
            e.what());
    }
}

bool DGMBolt::calibrate_joint_position_callback(
    dg_bolt::JointCalibration::Request& req,
    dg_bolt::JointCalibration::Response& res)
{
    // parse and register the command for further call.
    add_user_command(std::bind(&DGMBolt::calibrate_joint_position,
                               this,
                               zero_to_index_angle_from_file_));

    // return whatever the user want
    res.sanity_check = true;

    // the service has been executed properly
    return true;
}

void DGMBolt::calibrate_joint_position(
    const bolt::Vector6d& zero_to_index_angle)
{
    bolt_.calibrate(zero_to_index_angle);
}

}  // namespace dg_bolt
