/*
 * encoder.h
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#ifndef INCLUDE_JOINT_ENCODER_H_
#define INCLUDE_JOINT_ENCODER_H_

#include <atomic>

#include <hardware_interface/robot_state_base.h>

namespace qr_driver {

struct EncoderState : public HWStateBase {
  std::atomic<double> pos_;

  EncoderState(const std::string& name, double pos = 0)
    : HWStateBase(name),
      pos_(pos)
  { };
  virtual ~EncoderState() {};
  virtual std::string toString() {
    std::stringstream ss;
    ss << HWStateBase::toString();
    ss << "pos: " << pos_ << " ";
    return ss.str();
  }
};

class Encoder: public RobotStateBase {
public:
  typedef EncoderState StateType;
  typedef boost::shared_ptr<EncoderState> StateTypeSharedPtr;

  Encoder(const std::string& name)
    : RobotStateBase(name),
      state_(new StateType(name))
  { };
  virtual ~Encoder() { };

  virtual HWStateSharedPtr getState(const std::string& name) {
    if (0 != state_->name_.compare(name)) {
      LOG(WARNING) << "Requset the ERROR name state (actual vs request): ("
          << state_->name_ << " vs " << name << ")";
      return HWStateSharedPtr(nullptr);
    } else {
      return HWStateSharedPtr(new StateType(name, state_->pos_));
    }
  };

  StateTypeSharedPtr state_;
};

} /* namespace qr_driver */

#endif /* INCLUDE_JOINT_ENCODER_H_ */
