/*
 * encoder.h
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#ifndef INCLUDE_JOINT_ENCODER_H_
#define INCLUDE_JOINT_ENCODER_H_

#include <atomic>
#include <chrono>

#include <hardware_interface/robot_state_base.h>

namespace qr_driver {

struct EncoderState : public HWStateBase {
  std::atomic<double> pos_;
  // 需要propagate实例中， 通过软件代码计算出速度填入数据
  std::atomic<double> vel_;
  // 计算速度的辅助变量, 保存前一次更新的时间
  std::chrono::high_resolution_clock::time_point previous_time_;

  EncoderState(const std::string& name, double pos = 0, double vel = 0)
    : HWStateBase(name),
      pos_(pos), vel_(vel),
      previous_time_(std::chrono::high_resolution_clock::now())
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
      return HWStateSharedPtr(new StateType(name, state_->pos_, state_->vel_));
    }
  };

  StateTypeSharedPtr state_;
};

} /* namespace qr_driver */

#endif /* INCLUDE_JOINT_ENCODER_H_ */
