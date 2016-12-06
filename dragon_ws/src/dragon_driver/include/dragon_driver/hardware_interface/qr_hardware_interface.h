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
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <glog/logging.h>

#include <qr_driver.h>
#include "hardware_interface/actuator.h"

namespace qr_driver {

class QrHardwareInterface: public hardware_interface::RobotHW {
public:
  virtual ~QrHardwareInterface() { }

  QrHardwareInterface(ros::NodeHandle& nh, QrDriver* robot)
    : nh_(nh), robot_(robot),
      joint_names_(robot->joint_res_map_),
      num_joints_(joint_names_.size()) {
    init();
    LOG(INFO) << "Loaded QrHardwareInterface";
  }

  /// \brief Initialize the hardware interface
  virtual void init() {
    if (joint_names_.empty()) {
      LOG(INFO) <<
          "No joints found on parameter server for controller, did you load the proper yaml file?";
      exit(-1);
    }
    // LOG(INFO) << "Reading joint information from QrDriver";
    // num_joints_ = robot_->joint_res_map_.size();
    // joint_names_.reserve(num_joints_);
    // for (auto& joint :  robot_->joint_res_map_) {
    //   joint_names_.push_back(joint.joint_name_);
    // }
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
      std::string joint_name = joint_names_[i].joint_name_;
      LOG(INFO) << "QrHardwareInterface Loading joint name: "
          << joint_name;

      // Create joint state interface
      joint_state_interface_.registerHandle(
          hardware_interface::JointStateHandle(joint_name,
              &joint_position_[i], &joint_velocity_[i],
              &joint_effort_[i]));

      // Create position joint interface
      position_joint_interface_.registerHandle(
          hardware_interface::JointHandle(
              joint_state_interface_.getHandle(joint_name),
              &joint_position_command_[i]));

      // Create velocity joint interface
      velocity_joint_interface_.registerHandle(
          hardware_interface::JointHandle(
              joint_state_interface_.getHandle(joint_name),
              &joint_velocity_command_[i]));

      // Create effort joint interface
      effort_joint_interface_.registerHandle(
          hardware_interface::JointHandle(
              joint_state_interface_.getHandle(joint_name),
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
    sensor_msgs::JointState msg;
    robot_->getJointStates(msg);
    for (std::size_t idx_msg = 0; idx_msg < msg.name.size(); ++idx_msg) {
      for (std::size_t idx_jnt = 0; idx_jnt < num_joints_; ++idx_jnt) {
        if (0 == msg.name[idx_msg].compare(joint_names_[idx_jnt].joint_name_)) {
          joint_position_[idx_jnt] = msg.position[idx_msg];
          joint_velocity_[idx_jnt] = msg.velocity[idx_msg];
          joint_effort_[idx_jnt] = msg.effort[idx_msg];
          break;
        }
      }
    }
  }

  /// \brief write the command to the robot hardware.
  virtual void write() {
    // TODO
    if (velocity_interface_running_) {
      // TODO do some rate limiting?
      std::vector<HWCmdSharedPtr> cmd_vec;
      cmd_vec.reserve(num_joints_);

      // TODO 需要实际公式计算， 当前实现版本仅仅是电机的速度控制
      // 并未转换到Joint速度指令
      for (std::size_t i = 0; i < num_joints_; ++i) {
        cmd_vec.push_back(
            HWCmdSharedPtr(new Actuator::CmdType(
                joint_names_[i].actuator_names_[0],
                joint_velocity_command_[i], Actuator::CmdType::MODE_VEL_
                )
            )
        );
      }
      robot_->addCommand(cmd_vec);
    } else if (position_interface_running_) {
      // robot_->servoj(joint_position_command_);
    } else if (effort_interface_running_) {
      ; // Nothing to do here
    } else {
      // LOG(WARNING) << "What a fucking command?";
    }
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
          LOG(ERROR) << controller_it->name.c_str()
              << ": An interface of that type ("
              << controller_it->hardware_interface.c_str()
              << ") is already running";
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
            LOG(ERROR) << controller_it->name.c_str()
                << " (type " << controller_it->hardware_interface.c_str()
                << ") can not be run simultaneously with a PositionJointInterface";
            return false;
          }
        }
        if (effort_interface_running_) {
          bool error = true;
          for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
              stop_list.begin();
              stop_controller_it != stop_list.end();
              ++stop_controller_it) {
            if (0 == stop_controller_it->hardware_interface.compare(
                "hardware_interface::EffortJointInterface")) {
              error = false;
              break;
            }
          }
          if (error) {
            LOG(ERROR) << controller_it->name.c_str()
                << " (type " << controller_it->hardware_interface.c_str()
                << ") can not be run simultaneously with a EffortJointInterface";
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
        if (effort_interface_running_) {
            bool error = true;
            for (std::list<hardware_interface::ControllerInfo>::const_iterator stop_controller_it =
                stop_list.begin();
                stop_controller_it != stop_list.end();
                ++stop_controller_it) {
              if (0 == stop_controller_it->hardware_interface.compare(
                  "hardware_interface::EffortJointInterface")) {
                error = false;
                break;
              }
            }
            if (error) {
              LOG(ERROR) << controller_it->name.c_str() << " (type "
                  << controller_it->hardware_interface.c_str()
                  << ") can not be run simultaneously with a VelocityJointInterface";
              return false;
            }
          }
      } else if (0 == controller_it->hardware_interface.compare(
          "hardware_interface::EffortJointInterface")) {
        if (effort_interface_running_) {
          LOG(ERROR) << controller_it->name.c_str()
              << ": An interface of that type ("
              << controller_it->hardware_interface.c_str()
              << ") is already running";
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
            LOG(ERROR) << controller_it->name.c_str()
                      << " (type " << controller_it->hardware_interface.c_str()
                      << ") can not be run simultaneously with a VelocityJointInterface";
            return false;
          }
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
              LOG(ERROR) << controller_it->name.c_str()
                  << " (type " << controller_it->hardware_interface.c_str()
                  << ") can not be run simultaneously with a PositionJointInterface";
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
      if (0 == controller_it->hardware_interface.compare(
          "hardware_interface::EffortJointInterface")) {
        effort_interface_running_ = false;
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
      if (0 == controller_it->hardware_interface.compare(
          "hardware_interface::EffortJointInterface")) {
        effort_interface_running_ = true;
        // robot_->uploadProg();
        LOG(INFO) << ("Starting effort interface");
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

  bool velocity_interface_running_;
  bool position_interface_running_;
  bool effort_interface_running_;

  const std::vector<QrDriver::JointResMap>& joint_names_;
  std::size_t num_joints_;

  // Shared memory
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
