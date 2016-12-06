/*
 * propagate_imp_base.h
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#ifndef PROPAGATE_IMP_BASE_H_
#define PROPAGATE_IMP_BASE_H_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include "hardware_interface/robot_state_base.h"

namespace qr_driver {

class PropagateImpBase {
public:
  PropagateImpBase(const std::string& name)
    : connected_(false), name_(name) { };
  virtual ~PropagateImpBase() { };

  /**
   * 每一个注册的imp都初始化成功， 才返回true
   */
  virtual bool init() {
    connected_ = true;
    for (auto& imp : imp_map_) {
      connected_ = (connected_ && imp.second->init());
    }
    return connected_;
  }
  /**
   * names可能包含多种imp的通信方式
   * 当且仅当names中存在CMD名称是属于本imp子类所管理， 并且写入失败时，
   * 返回false, 其他情况均返回true
   */
  virtual bool write(const std::vector<std::string>& names) {
    connected_ = false;
    for (auto& imp : imp_map_) {
      connected_ = (connected_ || imp.second->write(names));
    }
    return connected_;
  }

  /**
   * 所有通信通道均read正常， 方才返回true
   */
  virtual bool read() {
    for (auto& imp : imp_map_) {
      connected_ = connected_ && imp.second->read();
    }
    return connected_;
  }

  virtual void stop() {
    for (auto& imp : imp_map_) {
      imp.second->stop();
    }
  }

public:
  virtual void registerHandle(HWCmdSharedPtr& cmd) {
    auto itr = cmd_map_.find(cmd->name_);
    if (cmd_map_.end() != itr) {
      LOG(WARNING) << "Replace " << cmd->name_ << " Command Resource Handle"
          << " in '" << this->name_ << "' Propagate instance";
      itr->second = cmd;
    } else {
      LOG(INFO) << "Register " << cmd->name_ << " Command Resource Handle Successful"
          << " in '" << this->name_ << "' Propagate instance";
      cmd_map_.insert(std::make_pair(cmd->name_, cmd));
    }
  }

  virtual void registerHandle(HWStateSharedPtr& state) {
    auto itr = state_map_.find(state->name_);
    if (state_map_.end() != itr) {
      LOG(WARNING) << "Replace " << state->name_ << " State Resource Handle"
          << " in '" << this->name_ << "' Propagate instance";
      itr->second = state;
    } else {
      LOG(INFO) << "Register " << state->name_ << " State Resource Handle Successful"
          << " in '" << this->name_ << "' Propagate instance";
      state_map_.insert(std::make_pair(state->name_, state));
    }
  }

  /**
   * 注册命令句柄到指定name的通信接口
   */
  virtual void registerHandle(HWCmdSharedPtr& cmd, const std::string& name) {
    PropagateImpBase* which(this);
    if (0 != name.compare(this->name_)) {
      auto itr = imp_map_.find(name);
      if (imp_map_.end() == itr) {
        LOG(WARNING) << "Could not found " << name
            << " propagate instance, we will register " << cmd->name_
            << " handle in the " << this->name_ << "instance";
      } else {
        which = itr->second.get();
      }
    }

    which->registerHandle(cmd);
  }

  /**
   * 注册状态句柄到指定name的通信接口
   */
  virtual void registerHandle(HWStateSharedPtr& state, const std::string& name) {
    PropagateImpBase* which(this);
    if (0 != name.compare(this->name_)) {
      auto itr = imp_map_.find(name);
      if (imp_map_.end() == itr) {
        LOG(WARNING) << "Could not found " << name
            << " propagate instance, we will register " << state->name_
            << " handle in the " << this->name_ << "instance";
      } else {
        which = itr->second.get();
      }
    }

    which->registerHandle(state);
  }

public:
  std::string getName() { return name_; };
  /**
   * 不同name_的PropagateImp仅能存在一个.
   * 若新增同名的对象, 则直接替换.
   */
  virtual void addImp(PropagateImpBase* imp) {
    auto itr = imp_map_.find(imp->getName());
    if (imp_map_.end() == itr) {
      LOG(INFO) << "Addition " << imp->getName() << " Propagate Implement";
      PropaImpSharedPtr imp_ptr(imp);
      imp_map_.insert(std::make_pair(imp->getName(), imp_ptr));
    } else {
      LOG(WARNING) << "Replace " << imp->getName() << " Propagate Implement Successful!";
      itr->second.reset(imp);
    }
  }

  virtual void removeImp(PropagateImpBase* imp) {
    removeImp(imp->name_);
  }

  virtual void removeImp(const std::string& name) {
    auto imp = imp_map_.find(name);
    if (imp_map_.end() != imp) {
      imp_map_.erase(imp);
    }
    LOG(INFO) << "Remove " << name << " Propagate Implement Successful!";
  }

protected:
  typedef std::map<std::string, HWCmdSharedPtr> ResourceMapCmd;
  typedef std::map<std::string, HWStateSharedPtr> ResourceMapState;
  ResourceMapCmd   cmd_map_;
  ResourceMapState state_map_;

  bool connected_;
protected:
  typedef boost::shared_ptr<PropagateImpBase> PropaImpSharedPtr;
  typedef std::map<std::string, PropaImpSharedPtr> PropaImpMap;
  const std::string name_;
  PropaImpMap       imp_map_;
};

} /* namespace quadruped_robot_driver */

#endif /* PROPAGATE_IMP_BASE_H_ */
