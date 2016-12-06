/*
 * robot_state_base.h
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#ifndef ROBOT_STATE_BASE_H_
#define ROBOT_STATE_BASE_H_

#include <string>
#include <map>

#include <boost/shared_ptr.hpp>
#include <glog/logging.h>

namespace qr_driver {

struct HWStateBase   {
  const std::string name_;
  HWStateBase(const std::string& name) : name_(name) { };
  virtual ~HWStateBase() { };

  // For Debug
  virtual std::string toString() {
    return "name: " + name_ + " ";
  }
};

struct HWCommandBase {
  const std::string name_;
  HWCommandBase(const std::string& name) : name_(name) { };
  virtual ~HWCommandBase() { };

  // For Debug
  virtual std::string toString() {
    return "name: " + name_ + " ";
  }
};

class RobotStateBase;
typedef boost::shared_ptr<HWStateBase>    HWStateSharedPtr;
typedef boost::shared_ptr<HWCommandBase>  HWCmdSharedPtr;
typedef boost::shared_ptr<RobotStateBase> RobotStateSharedPtr;

class RobotStateBase {
public:
  // 该函数子类选择性进行实现, 在函数内部, 需要完成数据的读写.
  // 若RobotState子类具备State， 则必须重写该函数实现
  virtual HWStateSharedPtr getState(const std::string& name) {
    HWStateSharedPtr ret(nullptr);
    auto hw = hw_map_.find(name);
    if (hw_map_.end() == hw) {
      LOG(WARNING) << "Could not found the haredware interface: " << name;
      return ret;
    }
    ret = hw->second->getState(name);
    return ret;
  }

  // 若RobotState子类具备Command， 则必须重写该函数实现
  virtual HWCmdSharedPtr getCommand(const std::string& name) {
    HWCmdSharedPtr ret(nullptr);
    auto hw = hw_map_.find(name);
    if (hw_map_.end() == hw) {
      LOG(WARNING) << "Could not found the haredware interface: " << name;
      return ret;
    }
    ret = hw->second->getCommand(name);
    return ret;
  }
  // 若RobotState子类具备State， 则必须重写该函数实现
  virtual void setState(const HWStateBase& state) {
    auto hw = hw_map_.find(state.name_);
    if (hw_map_.end() == hw) {
      LOG(WARNING) << "Could not found the haredware interface: " << state.name_;
      return;
    }
    hw->second->setState(state);
  }
  // 若RobotState子类具备State， 则必须重写该函数实现
  virtual void setState(const HWStateSharedPtr& state) {
    auto hw = hw_map_.find(state->name_);
    if (hw_map_.end() == hw) {
      LOG(WARNING) << "Could not found the haredware interface: " << state->name_;
      return;
    }
    hw->second->setState(state);
  }
  // 若RobotState子类具备State， 则必须重写该函数实现
  virtual void setCommand(const HWCommandBase& cmd) {
    auto hw = hw_map_.find(cmd.name_);
    if (hw_map_.end() == hw) {
      LOG(WARNING) << "Could not found the haredware interface: " << cmd.name_;
      return;
    }
    hw->second->setCommand(cmd);
  }
  // 若RobotState子类具备State， 则必须重写该函数实现
  virtual void setCommand(const HWCmdSharedPtr& cmd) {
    auto hw = hw_map_.find(cmd->name_);
    if (hw_map_.end() == hw) {
      LOG(WARNING) << "Could not found the haredware interface: " << cmd->name_;
      return;
    }
    hw->second->setCommand(cmd);
  }
  // 若无特殊需求， 子类具备不需重写该函数， 保持默认实现即可
  virtual void setCommand(const std::vector<HWCmdSharedPtr>& cmd_vec) {
    for (auto& cmd : cmd_vec) {
      setCommand(cmd);
    }
  }

  RobotStateBase(const std::string& name)
      : name_(name) { };
  virtual ~RobotStateBase() { };

  const std::string& getName() { return name_; }

public:
  /**
   * 不同name_的HardwareInterface仅能存在一个.
   * 若新增同名的对象, 则直接替换.
   */
  virtual void addRobotState(RobotStateBase* res) {
    auto hw = hw_map_.find(res->getName());
    if (hw_map_.end() == hw) {
      LOG(INFO) << "Addition " << res->getName() << " RobotState Implement";
      RobotStateSharedPtr hw_ptr(res);
      hw_map_.insert(std::make_pair(res->getName(), hw_ptr));
    } else {
      LOG(WARNING) << "Replace " << res->getName() << " RobotState Implement Successful!";
      hw->second.reset(res);
    }
  }

  virtual void removeRobotState(RobotStateBase* res) {
    removeRobotState(res->name_);
  }

  virtual void removeRobotState(const std::string& name) {
    auto hw = hw_map_.find(name);
    if (hw_map_.end() != hw) {
      hw_map_.erase(hw);
    }
    LOG(INFO) << "Remove " << name << " RobotState Implement Successful!";
  }

protected:
  typedef std::map<std::string, boost::shared_ptr<RobotStateBase>> ResourceMap;
  std::string name_;
  ResourceMap hw_map_;
};

} /* namespace quadruped_robot_driver */

#endif /* ROBOT_STATE_BASE_H_ */
