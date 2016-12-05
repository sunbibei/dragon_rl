/*
 * actuator.h
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#ifndef INCLUDE_ACTUATOR_H_
#define INCLUDE_ACTUATOR_H_

#include "hardware_interface/robot_state_base.h"
#include <atomic>

namespace qr_driver {

/*
 * The actual motor state
 */
struct MotorState : public HWStateBase {
  std::atomic<double> pos_;
  std::atomic<double> vel_;
  std::atomic<double> tor_;

  MotorState(const std::string& name,
      double pos = 0, double vel = 0, double tor = 0)
    : HWStateBase(name),
      pos_(pos), vel_(vel), tor_(tor) { };

  virtual ~MotorState() {};

  virtual std::string toString() {
    std::stringstream ss;
    ss << HWStateBase::toString();
    ss << "position: " << pos_ << " "
        << "velocity: " << vel_ << " "
        << "torque: " << tor_ << " ";
    return ss.str();
  }
};

/*
 * The motor command
 */
struct MotorCmd : public HWCommandBase {
  typedef enum {MODE_POS_, MODE_VEL_, MODE_TOR_} MODE_;

  std::atomic<double> command_;
  std::atomic<MODE_>  mode_;

  MotorCmd(const std::string& name, double cmd = 0, MODE_ = MODE_TOR_)
    : HWCommandBase(name),
      command_(cmd), mode_(MODE_TOR_) { };
  virtual ~MotorCmd() {};

  virtual std::string toString() {
    std::stringstream ss;
    ss << HWCommandBase::toString();
    ss << "mode: " << mode_ << " "
        << "command: " << command_ << " ";
    return ss.str();
  }
};

/*
 * The actuator state and command handle
 */
class Actuator : public RobotStateBase {
public:
  typedef MotorCmd CmdType;
  typedef MotorState StateType;
  typedef boost::shared_ptr<MotorCmd> CmdTypeSharedPtr;
  typedef boost::shared_ptr<MotorState> StateTypeSharedPtr;

  Actuator(const std::string&  name, CmdType::MODE_ mode = CmdType::MODE_TOR_)
    : RobotStateBase(name),
      motor_state_(new StateType(name)),
      motor_cmd_(new CmdType(name, mode)) { }

  virtual ~Actuator() {
  }

  // 该函数子类选择性进行实现, 在函数内部, 需要完成数据的读写.
  virtual HWStateSharedPtr getState(const std::string& name) {
    if (0 != motor_state_->name_.compare(name)) {
      LOG(WARNING) << "Requset the ERROR name state (actual vs request): ("
          << motor_state_->name_ << " vs " << name << ")";
      return HWStateSharedPtr(nullptr);
    } else {
      return HWStateSharedPtr(new StateType(motor_state_->name_,
              motor_state_->pos_, motor_state_->vel_, motor_state_->tor_));
    }
  }

  virtual HWCmdSharedPtr getCommand(const std::string& name) {
    if (0 != motor_cmd_->name_.compare(name)) {
      LOG(WARNING) << "Requset the ERROR name command (actual vs request): ("
          << motor_cmd_->name_ << " vs " << name << ")";
      return HWCmdSharedPtr(nullptr);
    } else {
      return HWCmdSharedPtr(new CmdType(name,
              motor_cmd_->command_, motor_cmd_->mode_));
    }
  }

  virtual void setState(const HWStateBase& state) {
    if (0 != name_.compare(state.name_)) return;

    const StateType& motor_state = static_cast<const StateType&>(state);
    double val = motor_state.pos_;
    motor_state_->pos_ = val;
    val = motor_state.vel_;
    motor_state_->vel_ = val;
    val = motor_state.tor_;
    motor_state_->tor_ = val;
  };

  virtual void setCommand(const HWCommandBase& cmd) {
    if (0 != name_.compare(cmd.name_)) return;

    const CmdType& motor_cmd = static_cast<const CmdType&>(cmd);
    CmdType::MODE_ mode = motor_cmd.mode_;
    motor_cmd_->mode_ = mode;
    double val = motor_cmd.command_;
    motor_cmd_->command_ = val;
  };

  StateTypeSharedPtr motor_state_;
  CmdTypeSharedPtr   motor_cmd_;
};

} /* namespace qr_driver */

#endif /* INCLUDE_ACTUATOR_H_ */
