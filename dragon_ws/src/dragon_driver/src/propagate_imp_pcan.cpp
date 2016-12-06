/*
 * propagate_imp_pcan.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#include <propagate_interface/propagate_imp_pcan.h>

namespace qr_driver {

PropagateImpPcan::PropagateImpPcan(const std::string& name)
  :PropagateImpBase(name)
{ }

PropagateImpPcan::~PropagateImpPcan() { }

// 完成PCAN的初始化
// 以及act_state_map_, act_cmd_map_, enc_state_map_三个MAP的从cmd_map_和state_map_中初始化
bool PropagateImpPcan::init() {
  return true;
}

// 完成数据的读写.
bool PropagateImpPcan::write(std::vector<std::string> names) {
  if (names.empty()) return true;

  LOG(INFO) << "PCAN write: ";
  for (auto name : names) {
    auto itr = cmd_map_.find(name);
    if (cmd_map_.end() != itr)
      LOG(INFO) << name << ": " << cmd_map_[name];
    else {
      LOG(WARNING) << "Could not found the " << name << " command handle( ";
    }
  }
  return true;
}

bool PropagateImpPcan::read() {
  // 从PCAN中获取到的数据对应到具体的状态name
  // 也可以从PCAN的数据中， 明确到底是什么类型的State
  // 转化为对应类型的State, 在进行赋值
  std::string name = "actuator1";
  auto itr = state_map_.find(name);
  if (state_map_.end() == itr) {
    LOG(WARNING) << "Could not found the " << name << " state handle: ";
  } else {
    Actuator::StateTypeSharedPtr act_state
      = boost::dynamic_pointer_cast<Actuator::StateType>(itr->second);

    act_state->pos_ = act_state->pos_ + 0.00001;
    act_state->vel_ = act_state->vel_ + 0.00001;
    act_state->tor_ = act_state->tor_ + 0.00001;
  }

  name = "encoder1";
  itr = state_map_.find(name);
  if (state_map_.end() == itr) {
    LOG(WARNING) << "Could not found the " << name << " state handle: ";
  } else {
    Encoder::StateTypeSharedPtr act_state
      = boost::dynamic_pointer_cast<Encoder::StateType>(itr->second);
    double current_pos = act_state->pos_ + 0.00001;
    auto current_time = std::chrono::high_resolution_clock::now();
    act_state->vel_ = (current_pos - act_state->pos_)
        / std::chrono::duration_cast<std::chrono::duration<double>>(
            current_time - act_state->previous_time_).count();
    act_state->pos_ = current_pos;
    act_state->previous_time_ = current_time;
  }

  name = "encoder2";
  itr = state_map_.find(name);
  if (state_map_.end() == itr) {
    LOG(WARNING) << "Could not found the " << name << " state handle: ";
  } else {
    Encoder::StateTypeSharedPtr act_state
      = boost::dynamic_pointer_cast<Encoder::StateType>(itr->second);
    double current_pos = act_state->pos_ + 0.0000001;
    auto current_time = std::chrono::high_resolution_clock::now();
    act_state->vel_ = (current_pos - act_state->pos_)
        / std::chrono::duration_cast<std::chrono::duration<double>>(
            current_time - act_state->previous_time_).count();
    act_state->pos_ = current_pos;
    act_state->previous_time_ = current_time;
  }

  return true;
}

} /* namespace qr_driver */
