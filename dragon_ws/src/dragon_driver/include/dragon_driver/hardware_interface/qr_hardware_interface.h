/*
 * qr_hardware_interface.h
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#ifndef INCLUDE_HARDWARE_INTERFACE_QR_HARDWARE_INTERFACE_H_
#define INCLUDE_HARDWARE_INTERFACE_QR_HARDWARE_INTERFACE_H_

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <glog/logging.h>

#include <qr_driver.h>

namespace qr_driver {

class QrHardwareInterface: public hardware_interface::RobotHW {
public:
  virtual ~QrHardwareInterface() { }

  QrHardwareInterface(ros::NodeHandle& nh, QrDriver* robot)
    : nh_(nh), robot_(robot), num_joints_(0) {
    init();
    LOG(INFO) << "Loaded QrHardwareInterface";
  }

  /// \brief Initialize the hardware interface
  virtual void init() {
    if (robot_->joint_res_map_.empty()) {
      LOG(INFO) <<
          "No joints found on parameter server for controller, did you load the proper yaml file?";
      exit(-1);
    }
    LOG(INFO) << "Reading joint information from QrDriver";
    num_joints_ = robot_->joint_res_map_.size();
    joint_names_.reserve(num_joints_);
    for (auto& joint :  robot_->joint_res_map_) {
      joint_names_.push_back(joint.joint_name_);
    }
    // Resize vectors
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
    joint_effort_command_.resize(num_joints_);
    prev_joint_velocity_command_.resize(num_joints_);

    // Initialize controller
    for (std::size_t i = 0; i < num_joints_; ++i) {
      LOG(INFO) << "QrHardwareInterface Loading joint name: "
          << joint_names_[i];

      // Create joint state interface
      joint_state_interface_.registerHandle(
          hardware_interface::JointStateHandle(joint_names_[i],
              &joint_position_[i], &joint_velocity_[i],
              &joint_effort_[i]));

      // Create position joint interface
      position_joint_interface_.registerHandle(
          hardware_interface::JointHandle(
              joint_state_interface_.getHandle(joint_names_[i]),
              &joint_position_command_[i]));

      // Create velocity joint interface
      velocity_joint_interface_.registerHandle(
          hardware_interface::JointHandle(
              joint_state_interface_.getHandle(joint_names_[i]),
              &joint_velocity_command_[i]));
      effort_joint_interface_.registerHandle(
          hardware_interface::JointHandle(
              joint_state_interface_.getHandle(joint_names_[i]),
              &joint_effort_command_[i]));
      prev_joint_velocity_command_[i] = 0.;
    }

    registerInterface(&joint_state_interface_); // From RobotHW base class.
    registerInterface(&position_joint_interface_); // From RobotHW base class.
    registerInterface(&velocity_joint_interface_); // From RobotHW base class.
    registerInterface(&effort_joint_interface_); // From RobotHW base class.
    velocity_interface_running_ = false;
    position_interface_running_ = false;
    effort_interface_running_ = false;
  }

  /// \brief Read the state from the robot hardware.
  virtual void read() {
    // TODO
    ;
  }

  /// \brief write the command to the robot hardware.
  virtual void write() {
    // TODO
    ;
  }

  bool canSwitch(
        const std::list<hardware_interface::ControllerInfo> &start_list,
        const std::list<hardware_interface::ControllerInfo> &stop_list) const {
    for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
        start_list.begin(); controller_it != start_list.end();
        ++controller_it) {
      if (0 == controller_it->hardware_interface.compare(
          "hardware_interface::VelocityJointInterface")) {
        if (velocity_interface_running_) {
          LOG(ERROR) << "%s: An interface of that type (%s) is already running"
              << controller_it->name.c_str()
              << controller_it->hardware_interface.c_str();
          return false;
        }
        if (position_interface_running_) {
          bool error = true;
          for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
              stop_list.begin();
              stop_controller_it != stop_list.end();
              ++stop_controller_it) {
            if (0 == stop_controller_it->hardware_interface.compare(
                "hardware_interface::PositionJointInterface")) {
              error = false;
              break;
            }
          }
          if (error) {
            LOG(ERROR) << "%s (type %s) can not be run simultaneously with a PositionJointInterface"
                << controller_it->name.c_str()
                << controller_it->hardware_interface.c_str();
            return false;
          }
        }
      } else if (0 == controller_it->hardware_interface.compare(
          "hardware_interface::PositionJointInterface")) {
        if (position_interface_running_) {
          LOG(ERROR) << "%s: An interface of that type (%s) is already running"
              << controller_it->name.c_str()
              << controller_it->hardware_interface.c_str();
          return false;
        }
        if (velocity_interface_running_) {
          bool error = true;
          for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
              stop_list.begin();
              stop_controller_it != stop_list.end();
              ++stop_controller_it) {
            if (0 == stop_controller_it->hardware_interface.compare(
                "hardware_interface::VelocityJointInterface")) {
              error = false;
              break;
            }
          }
          if (error) {
            LOG(ERROR) << "%s (type %s) can not be run simultaneously with a VelocityJointInterface"
                << controller_it->name.c_str()
                << controller_it->hardware_interface.c_str();
            return false;
          }
        }
      }
    }

    // we can always stop a controller
    return true;
  }
  void doSwitch(const std::list<hardware_interface::ControllerInfo>&start_list,
      const std::list<hardware_interface::ControllerInfo>&stop_list) {
    for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
        stop_list.begin(); controller_it != stop_list.end();
        ++controller_it) {
      if (0 == controller_it->hardware_interface.compare(
          "hardware_interface::VelocityJointInterface")) {
        velocity_interface_running_ = false;
        LOG(INFO) << ("Stopping velocity interface");
      }
      if (0 == controller_it->hardware_interface.compare(
          "hardware_interface::PositionJointInterface")) {
        position_interface_running_ = false;
        // std::vector<double> tmp;
        // robot_->closeServo(tmp);
        LOG(INFO) << ("Stopping position interface");
      }
    }
    for (std::list<hardware_interface::ControllerInfo>::const_iterator controller_it =
        start_list.begin(); controller_it != start_list.end();
        ++controller_it) {
      if (0 == controller_it->hardware_interface.compare(
          "hardware_interface::VelocityJointInterface")) {
        velocity_interface_running_ = true;
        LOG(INFO) << ("Starting velocity interface");
      }
      if (0 == controller_it->hardware_interface.compare(
          "hardware_interface::PositionJointInterface")) {
        position_interface_running_ = true;
        // robot_->uploadProg();
        LOG(INFO) << ("Starting position interface");
      }
    }
  }

protected:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;
  // Robot API
  QrDriver* robot_;

  // Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  // Effort 接口实际并未实现
  hardware_interface::EffortJointInterface effort_joint_interface_;

  std::size_t num_joints_;
  bool velocity_interface_running_;
  bool position_interface_running_;
  bool effort_interface_running_;
  // Shared memory
  std::vector<std::string> joint_names_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  // 速度控制时， 使用到的辅助变量
  std::vector<double> prev_joint_velocity_command_;
};

} /* namespace qr_driver */

#endif /* INCLUDE_HARDWARE_INTERFACE_QR_HARDWARE_INTERFACE_H_ */
