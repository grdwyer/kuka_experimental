/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author: Lars Tingelstad <lars.tingelstad@ntnu.no>
 */

#include "kuka_rsi_hw_interface/kuka_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <stdexcept>


namespace kuka_rsi_hw_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


CallbackReturn KukaHardwareInterface::on_init(const hardware_interface::HardwareInfo &system_info)
{
    if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
    in_buffer_.resize(1024);
    out_buffer_.resize(1024);
    remote_host_.resize(1024);
    remote_port_.resize(1024);

    clock_ = rclcpp::Clock(RCL_STEADY_TIME);
    info_ = system_info;

    node_ = std::make_shared<rclcpp::Node>("rsi_node");
    rclcpp::QoS qos(10);
    qos.reliable().transient_local();
    auto pub = node_->create_publisher<std_msgs::msg::String>("rsi_xml_doc", qos);    
    
    rt_rsi_pub_ = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(pub);

    // initialize
    joint_position_.assign(system_info.joints.size(), 0.0);
    joint_velocity_.assign(system_info.joints.size(), 0.0);
    joint_effort_.assign(system_info.joints.size(), 0.0);
    joint_position_command_.assign(system_info.joints.size(), 0.0); 
    joint_velocity_command_.assign(system_info.joints.size(), 0.0);
    joint_effort_command_.assign(system_info.joints.size(), 0.0); 
    rsi_initial_joint_positions_.assign(system_info.joints.size(), 0.0); 
    rsi_joint_position_corrections_.assign(system_info.joints.size(), 0.0);
    
    ipoc_ = 0;

    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(rclcpp::get_logger("KukaHardwareInterface"),
                         "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(),
                         joint.command_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("KukaHardwareInterface"),
                         "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                         joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }

//        if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
//        {
//            RCLCPP_FATAL(rclcpp::get_logger("KukaHardwareInterface"),
//                         "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
//                         joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
//            return hardware_interface::return_type::ERROR;
//        }

        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(rclcpp::get_logger("KukaHardwareInterface"), "Joint '%s' has %ld state interface. 3 expected.",
                         joint.name.c_str(), joint.state_interfaces.size());
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(rclcpp::get_logger("KukaHardwareInterface"),
                         "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(rclcpp::get_logger("KukaHardwareInterface"),
                         "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
        {
            RCLCPP_FATAL(rclcpp::get_logger("KukaHardwareInterface"),
                         "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                         joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_POSITION);
            return CallbackReturn::ERROR;
        }
    }
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KukaHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaHardwareInterface"), "Setting up state interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaHardwareInterface"), "Setting joint: " << info_.joints[i].name << " at index " << i);
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_[i]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_[i]));
        
        state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_effort_[i]));

    }


    // for (auto& sensor : info_.sensors)
    // {
    //     for (uint j = 0; j < sensor.state_interfaces.size(); ++j)
    //     {
    //         state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name,
    //                                                                          &current_ext_torque_[j]));
    //     }
    // }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KukaHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaHardwareInterface"), "Setting up command interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaHardwareInterface"), "Setting joint: " << info_.joints[i].name << " at index " << i);
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[i]));

        // TODO: replace with an effort interface and possibly wrench in the future
//        command_interfaces.emplace_back(hardware_interface::CommandInterface(
//                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_velocity_commands_[i]));
    }

    return command_interfaces;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KukaHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("KukaHardwareInterface"), "Starting ...please wait...");

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // The robot's IP address.
    std::string listen_ip = info_.hardware_parameters["listen_ip"];
    int listen_port = std::stoi(info_.hardware_parameters["listen_port"]);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaHardwareInterface"), "Initializing driver using: \n\tIP: " << listen_ip << "\n\tPort: " << listen_port);
      // Wait for connection from robot
    server_.reset(new UDPServer(listen_ip, listen_port));

    RCLCPP_INFO(rclcpp::get_logger("KukaHardwareInterface"), "Waiting for robot!");

    int bytes = server_->recv(in_buffer_);

    // Drop empty <rob> frame with RSI <= 2.3
    if (bytes < 100)
    {
      bytes = server_->recv(in_buffer_);
    }

    rsi_state_ = RSIState(in_buffer_);
    for (std::size_t i = 0; i < joint_position_.size(); ++i)
    {
      joint_position_[i] = angles::from_degrees(rsi_state_.positions[i]);
      joint_position_command_[i] = joint_position_[i];
      rsi_initial_joint_positions_[i] = rsi_state_.initial_positions[i];
    }
    ipoc_ = rsi_state_.ipoc;
    out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
    server_->send(out_buffer_);
    // Set receive timeout to 1 second
    server_->set_timeout(1000);
    RCLCPP_INFO(rclcpp::get_logger("KukaHardwareInterface"), "Got connection from robot");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("KukaHardwareInterface"), "instantiated driver");

    return CallbackReturn::SUCCESS;
    // TODO: add a check if this is successfull otherwise return an error code
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn KukaHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(rclcpp::get_logger("KukaHardwareInterface"), "Stopping ...please wait...");

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Need to implement an actual stop for the kuka

    RCLCPP_INFO(rclcpp::get_logger("KukaHardwareInterface"), "System successfully stopped!");

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type KukaHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  in_buffer_.resize(1024);

  if (server_->recv(in_buffer_) == 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  if (rt_rsi_pub_->trylock()){
    rt_rsi_pub_->msg_.data = in_buffer_;
    rt_rsi_pub_->unlockAndPublish();
  }

  rsi_state_ = RSIState(in_buffer_);
  for (std::size_t i = 0; i < joint_position_.size(); ++i)
  {
    joint_position_[i] = angles::from_degrees(rsi_state_.positions[i]);
  }
  ipoc_ = rsi_state_.ipoc;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type KukaHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  out_buffer_.resize(1024);

  for (std::size_t i = 0; i < joint_position_command_.size(); ++i)
  {
    rsi_joint_position_corrections_[i] = angles::to_degrees(joint_position_command_[i]) - rsi_initial_joint_positions_[i];
  }

  out_buffer_ = RSICommand(rsi_joint_position_corrections_, ipoc_).xml_doc;
  server_->send(out_buffer_);

  return hardware_interface::return_type::OK;
}

} // namespace kuka_rsi_hw_interface

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(kuka_rsi_hw_interface::KukaHardwareInterface, hardware_interface::SystemInterface)