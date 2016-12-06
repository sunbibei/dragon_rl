/*
 * qr_driver.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#include "hardware_interface/robot_state_base.h"
#include "hardware_interface/encoder.h"
#include "hardware_interface/actuator.h"
#include "propagate_interface/propagate_imp_base.h"
#include "robot_parser.h"

#include <thread>
#include <glog/logging.h>
#include <glog/log_severity.h>
#include <qr_driver.h>
#include <tinyxml.h>

namespace qr_driver {

QrDriver* QrDriver::instance_ = nullptr;
QrDriver* QrDriver::getInstance() {
  if (nullptr == instance_) {
    instance_ = new QrDriver;
    LOG(INFO) << "Create the singleton instance: QuadrupedDriver";
  }

  LOG(INFO) << "Return the singleton instance: QuadrupedDriver";
  return instance_;
}

QrDriver::QrDriver()
    : new_command_(false), keepalive_(true), connected_(false) {
}

QrDriver::~QrDriver() {
  this->halt();
  propagate_.reset((PropagateImpBase*)nullptr);
  robot_.reset((RobotStateBase*)nullptr);
  if (nullptr != instance_) {
    delete instance_;
    instance_ = nullptr;
  }
}

bool QrDriver::initFromFile(const std::string& xml) {

  if (((nullptr == robot_) || (nullptr == propagate_))
      && (!RobotParser::parserFromFile(xml, this))) {
    LOG(ERROR) << "The initialization FAIL in the QrDriver";
    return false;
  }

  LOG(INFO) << "The initialization has successful";
  connected_ = propagate_->init();
  return connected_;
}

bool QrDriver::initFromParam(const std::string& param) {

  if (((nullptr == robot_) || (nullptr == propagate_))
      && (!RobotParser::parserFromParam(param, this))) {
    LOG(ERROR) << "The initialization FAIL in the QrDriver";
    return false;
  }

  LOG(INFO) << "The initialization has successful";
  connected_ = propagate_->init();
  return connected_;
}

bool QrDriver::start() {
  propagate_thread_ = std::thread(&QrDriver::runPropagate, this);
  LOG(INFO) << "The propagate thread has started to run!";
  return true;
}

void QrDriver::runPropagate() {
  while (keepalive_) {
    while (connected_ && keepalive_) {
      // Everything is OK!
      connected_ = propagate_->read();
      if (new_command_) {
        connected_ = propagate_->write(cmd_vec_);
        new_command_ = false;
        cmd_vec_.clear();
      }
    }
    if (keepalive_) {
      //reconnect
      LOG(WARNING) << "Disconnected! In order to Keepalive, we try to reconnect... ...";
      int count = 0;
      while (keepalive_ && !connected_) {
        LOG(WARNING) << "Attempt to reconnect (" << count++ << " times)";
        connected_ = propagate_->init();
        if (!connected_) {
          // wait for 500ms
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      } // end while(keepalive_ && !connected_)
    } // end if (keepalive_)
  } // end while (keepalive_)
}

void QrDriver::halt() {
  keepalive_ = false;
  propagate_thread_.join();

  LOG(INFO) << "QrDrvier halt... ...";
}

/**
 * 增加单个执行器命令到命令队列
 */
void QrDriver::addCommand(const HWCommandBase& cmd) {
  cmd_vec_.push_back(cmd.name_);
  robot_->setCommand(cmd);
  new_command_ = true;
}

/**
 * 增加单个执行器命令到命令队列
 */
void QrDriver::addCommand(const HWCmdSharedPtr& cmd) {
  cmd_vec_.push_back(cmd->name_);
  robot_->setCommand(cmd);
  new_command_ = true;
}

/**
 * 增加多个执行器命令到命令队列
 */
void QrDriver::addCommand(const std::vector<HWCmdSharedPtr>& cmd_vec) {
  for (auto& cmd : cmd_vec) {
    cmd_vec_.push_back(cmd->name_);
  }
  robot_->setCommand(cmd_vec);
  new_command_ = true;
}

/**
 * 获取Joint的名称
 */
std::vector<std::string> QrDriver::getJointNames() {
  std::vector<std::string> names;
  if (!joint_res_map_.empty()) {
    names.reserve(joint_res_map_.size());
    for (auto& jnt : joint_res_map_) {
      names.push_back(jnt.joint_name_);
    }
  }

  return names;
}
/**
 * Actual joint positions
 */
std::vector<double> QrDriver::getJointPositions() {
  std::vector<double> positions;
  if (!joint_res_map_.empty()) {
    positions.reserve(joint_res_map_.size());
    for (auto& jnt : joint_res_map_) {
      Encoder::StateTypeSharedPtr state
        = boost::dynamic_pointer_cast<Encoder::StateType>(
            robot_->getState(jnt.encoder_names_[0]));
      if (nullptr != state)
        positions.push_back(state->pos_);
      else
        LOG(ERROR) << "No \"" << jnt.joint_name_ << "\" joint";
    }
  }

  return positions;
}
/**
 * Actual joint velocities
 */
std::vector<double> QrDriver::getJointVelocities() {
  std::vector<double> velocities;
  if (!joint_res_map_.empty()) {
    velocities.reserve(joint_res_map_.size());
    for (auto& jnt : joint_res_map_) {
      Encoder::StateTypeSharedPtr state
        = boost::dynamic_pointer_cast<Encoder::StateType>(
            robot_->getState(jnt.encoder_names_[0]));
      if (nullptr != state)
        velocities.push_back(state->vel_);
      else
        LOG(ERROR) << "No \"" << jnt.joint_name_ << "\" joint";
    }
  }

  return velocities;
}
/**
 * Actual joint torques TODO NO IMPLEMENTS
 */
std::vector<double> QrDriver::getJointTorques() {
  std::vector<double> torques;
  /*
  if (!joint_res_map_.empty()) {
    torques.reserve(joint_res_map_.size());
    for (auto& jnt : joint_res_map_) {
      Encoder::StateTypeSharedPtr state
        = boost::dynamic_pointer_cast<Encoder::StateType>(
            robot_->getState(jnt.encoder_names_[0]));
      if (nullptr != state)
        torques.push_back(state->tor_);
      else
        LOG(ERROR) << "No \"" << jnt.joint_name_ << "\" joint";
    }
  }
  */
  return torques;
}
/**
 * Actual JointState( Recommended )
 */
void QrDriver::getJointStates(sensor_msgs::JointState& msg) {
  if (joint_res_map_.empty()) return;

  msg.name.reserve(joint_res_map_.size());
  msg.position.reserve(joint_res_map_.size());
  msg.velocity.reserve(joint_res_map_.size());
  msg.effort.reserve(joint_res_map_.size());

  for (auto& jnt : joint_res_map_) {
    Encoder::StateTypeSharedPtr state
      = boost::dynamic_pointer_cast<Encoder::StateType>(
          robot_->getState(jnt.encoder_names_[0]));
    if (nullptr != state) {
      msg.name.push_back(jnt.joint_name_);
      msg.position.push_back(state->pos_);
      msg.velocity.push_back(state->vel_);
      msg.effort.push_back(0.0);
    } else
      LOG(ERROR) << "No \"" << jnt.joint_name_ << "\" joint";
  }
}

} /* namespace quadruped_robot_driver */
