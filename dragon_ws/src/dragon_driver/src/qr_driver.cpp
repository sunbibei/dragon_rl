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
    : servoj_time_(200), executing_traj_(false),
      new_command_(false), keepalive_(true),
      connected_(false) {
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

void QrDriver::stopTraj() {
  executing_traj_ = false;
}

bool QrDriver::doTraj(const std::vector<double>& inp_timestamps,
      const std::vector<std::vector<double>>& inp_positions,
      const std::vector<std::vector<double>>& inp_velocities) {
  std::chrono::high_resolution_clock::time_point t0, t;
  std::vector<double> positions;
  unsigned int j;

  if ((nullptr == propagate_.get())
      || (nullptr == robot_.get())){
    LOG(ERROR) << "Invalidate propagate or robot state";
    return false;
  }
  executing_traj_ = true;
  t0 = std::chrono::high_resolution_clock::now();
  t = t0;
  j = 0;
  while ((inp_timestamps[inp_timestamps.size() - 1]
      >= std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count())
      and executing_traj_) { // 确保仍在轨迹时间内
    while (inp_timestamps[j]
        <= std::chrono::duration_cast<std::chrono::duration<double>>(
            t - t0).count() && j < inp_timestamps.size() - 1) {
      j += 1; // 跳转到距离当前执行时间最近的一个轨迹点
    }
    // 计算得到当前时间所要达到的关节位置
    positions = interp_cubic(
        // 实际执行该轨迹点的时间与该轨迹点的前一个点要求时间的差值
        std::chrono::duration_cast<std::chrono::duration<double>>
            (t - t0).count() - inp_timestamps[j - 1],
        // 待执行轨迹点与前一个点理论时间间距
        inp_timestamps[j] - inp_timestamps[j - 1],
        // 待执行轨迹点与前一个点的要求位置
        inp_positions[j - 1], inp_positions[j],
        // 待执行轨迹点与前一个点的要求速度
        inp_velocities[j - 1], inp_velocities[j]);
    executeJointPositions(positions);

    // oversample with 4 * sample_time
    std::this_thread::sleep_for(
        std::chrono::milliseconds((int)(servoj_time_ / 4.)));
    t = std::chrono::high_resolution_clock::now();
  }
  executing_traj_ = false;
  return true;
}

void QrDriver::executeJointPositions(const std::vector<double>& positions) {
  ; // TODO
  std::vector<HWCmdSharedPtr> cmd_vec;
  cmd_vec.reserve(joint_res_map_.size());

  // TODO 需要实际公式计算， 当前实现版本仅仅是电机的位置控制
  // 并未转换到Joint速度指令
  for (std::size_t i = 0; i < joint_res_map_.size(); ++i) {
    cmd_vec.push_back(
        HWCmdSharedPtr(new Actuator::CmdType(
            joint_res_map_[i].actuator_names_[0],
            positions[i], Actuator::CmdType::MODE_POS_
            )
        )
    );
  }
  addCommand(cmd_vec);
}

void QrDriver::executeJointVelocities(const std::vector<double>& velocities) {
  // TODO do some rate limiting?
  std::vector<HWCmdSharedPtr> cmd_vec;
  cmd_vec.reserve(joint_res_map_.size());

  // TODO 需要实际公式计算， 当前实现版本仅仅是电机的速度控制
  // 并未转换到Joint速度指令
  for (std::size_t i = 0; i < joint_res_map_.size(); ++i) {
    cmd_vec.push_back(
        HWCmdSharedPtr(new Actuator::CmdType(
            joint_res_map_[i].actuator_names_[0],
            velocities[i], Actuator::CmdType::MODE_VEL_
            )
        )
    );
  }
  addCommand(cmd_vec);
}

/**
 * 计算得到t时刻的关节位置
 * @param t       实际执行时间
 * @param T       执行总时长
 * @param p0_pos  起始点位置
 * @param p1_pos  终止点位置
 * @param p0_vel  起始点速度
 * @param p1_vel  终止点速度
 */
std::vector<double> QrDriver::interp_cubic(double t, double T,
    const std::vector<double>& p0_pos, const std::vector<double>& p1_pos,
    const std::vector<double>& p0_vel, const std::vector<double>& p1_vel) {
  /* Returns positions of the joints at time 't' */
  std::vector<double> positions;
  for (unsigned int i = 0; i < p0_pos.size(); i++) {
    double a = p0_pos[i];
    double b = p0_vel[i];
    double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i]
        - T * p1_vel[i]) / pow(T, 2);
    double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i]
        + T * p1_vel[i]) / pow(T, 3);
    positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
  }
  return positions;
}

} /* namespace quadruped_robot_driver */
