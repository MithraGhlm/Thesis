// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "diffdrive_canopen/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>


namespace diffdrive_canopen
{
hardware_interface::CallbackReturn DiffDriveCanOpenHardware::on_init(
  const hardware_interface::HardwareInfo & info) // HWI gets passed in here & parameters are set
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCanOpenHardware"), "on_init ...please wait...");
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCanOpenHardware"), "starting thread ...please wait...");
  comms_.initialize();
  // looking for parameters with these names in the ros2_control.xacro file
  comms_.wheel_l_->set_name(info_.hardware_parameters["left_wheel_name"]);
  comms_.wheel_r_->set_name(info_.hardware_parameters["right_wheel_name"]);
  //timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  std::this_thread::sleep_for(500ms);



  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCanOpenHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCanOpenHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCanOpenHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCanOpenHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveCanOpenHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveCanOpenHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCanOpenHardware"), "Activating ...please wait...");

// //  comms_.motor2_->set_transition(PD4Motor::TransitionCommand::Shutdown);

  comms_.wheel_l_->set_mode(PD4Motor::OperatingMode::Velocity);
  comms_.wheel_r_->set_mode(PD4Motor::OperatingMode::Velocity);

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCanOpenHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveCanOpenHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCanOpenHardware"), "Deactivating ...please wait...");

  comms_.stop_thread();

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveCanOpenHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


// When we update pos & vel values, it tells the rest of the ros2_control systme that those wheels pos & vel has changed.
// ---> the position and velocity value comming from the wheels (Read)
std::vector<hardware_interface::StateInterface> DiffDriveCanOpenHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    comms_.wheel_l_->name, hardware_interface::HW_IF_POSITION, &comms_.wheel_l_->pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    comms_.wheel_l_->name, hardware_interface::HW_IF_VELOCITY, &comms_.wheel_l_->vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    comms_.wheel_r_->name, hardware_interface::HW_IF_POSITION, &comms_.wheel_r_->pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    comms_.wheel_r_->name, hardware_interface::HW_IF_VELOCITY, &comms_.wheel_r_->vel));
  return state_interfaces;
}


// Anytime the ros2_control sets the velocity of these wheels, it changes the value of cmd
// ---> the master writing to motors (write)
std::vector<hardware_interface::CommandInterface> DiffDriveCanOpenHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    comms_.wheel_l_->name, hardware_interface::HW_IF_VELOCITY, &comms_.wheel_l_->cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    comms_.wheel_r_->name, hardware_interface::HW_IF_VELOCITY, &comms_.wheel_r_->cmd));
  return command_interfaces;
}


// TODO: read from wheels their pos and vel and put in the vector state_interfaces
hardware_interface::return_type DiffDriveCanOpenHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // convert tenths of degree to rad 

  comms_.wheel_l_->pos = comms_.wheel_l_->get_PositionActualValue()*(M_PI/1800);
  comms_.wheel_l_->vel = comms_.wheel_l_->get_VelocityActualValue()*(2*M_PI)/60;

  comms_.wheel_r_->pos = comms_.wheel_r_->get_PositionActualValue()*(M_PI/1800);
  comms_.wheel_r_->vel = comms_.wheel_r_->get_VelocityActualValue()*(2*M_PI)/60;

RCLCPP_INFO(
  rclcpp::get_logger("DiffDriveCanOpenHardware"),
  "pos_l %f, vel_l %f, pos_r %f, vel_r %f", comms_.wheel_l_->pos,  comms_.wheel_l_->vel,
  comms_.wheel_r_->pos, comms_.wheel_r_->vel);


  return hardware_interface::return_type::OK;
}

// write to wheels the pos and vel by putting in the vector command_interfaces
hardware_interface::return_type diffdrive_canopen ::DiffDriveCanOpenHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // convert rad/s to RPM
  double coeff = 60/(2*M_PI);

  RCLCPP_INFO(rclcpp::get_logger("The commanded velocity is:"), "%f , %f\n", comms_.wheel_l_->cmd*coeff, comms_.wheel_r_->cmd*coeff);
  
  comms_.wheel_l_->AsyncWrite<int16_t>(0x6042, 0, (comms_.wheel_l_->cmd*coeff));
  comms_.wheel_r_->set_TargetVelocity((comms_.wheel_r_->cmd*coeff));

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_canopen

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_canopen::DiffDriveCanOpenHardware, hardware_interface::SystemInterface)
