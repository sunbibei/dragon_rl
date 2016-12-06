/*
 * robot_parser.cpp
 *
 *  Created on: Dec 3, 2016
 *      Author: silence
 */

#include <robot_parser.h>
#include <glog/logging.h>

#include "hardware_interface/robot_state_base.h"
#include "hardware_interface/encoder.h"
#include "hardware_interface/actuator.h"

#include "propagate_interface/propagate_imp_base.h"
#include "propagate_interface/propagate_imp_pcan.h"

#include "qr_driver.h"

namespace qr_driver {

RobotParser::ParserMap RobotParser::hw_parser_map_ = ParserMap();
TiXmlElement* RobotParser::xml_root_ = nullptr;

RobotParser::RobotParser() { }

RobotParser::~RobotParser() { }

bool RobotParser::parserFromFile(const std::string& filename, QrDriver* robot) {
  if (!RobotParser::init(filename)) {
    LOG(ERROR) << "RobotParser has initialized fail";
    return false;
  }

  return (parserRobotStateAndPropagate(robot)
            && parserJointStates(robot));
}

bool RobotParser::parserFromParam(const std::string& param, QrDriver* robot) {
  if (!RobotParser::initParam(param)) {
    LOG(ERROR) << "RobotParser has initialized fail";
    return false;
  }

  return (parserRobotStateAndPropagate(robot)
            && parserJointStates(robot));
}

/**
 * 完成解析函数MAP的初始化， 以及别的初始化工作
 */
bool RobotParser::init(const std::string& filename) {
  RobotParser::hw_parser_map_.insert(
      std::make_pair("Actuator", RobotParser::getActuator));
  RobotParser::hw_parser_map_.insert(
      std::make_pair("Encoder", RobotParser::getEncoder));

  // 初始化XML文档相关内容
  TiXmlDocument* xml_doc = new TiXmlDocument();
  if (!xml_doc->LoadFile(filename)) {
    LOG(ERROR) << "Could not found the "<< filename << ", did you forget define the file?";
    return false;
  }
  xml_root_ = xml_doc->RootElement();  // Robot
  return true;
}

bool RobotParser::initParam(const std::string& param) {
  RobotParser::hw_parser_map_.insert(
      std::make_pair("Actuator", RobotParser::getActuator));
  RobotParser::hw_parser_map_.insert(
      std::make_pair("Encoder", RobotParser::getEncoder));

  // 初始化XML文档相关内容
  TiXmlDocument* xml_doc = new TiXmlDocument();
  xml_doc->Parse(param.c_str());
  xml_root_ = xml_doc->RootElement();  // Robot
  return true;
}

bool RobotParser::parserJointStates(QrDriver* robot) {
  bool nothing = true;
  auto jnt_root = xml_root_->FirstChildElement("joint_states");
  for (auto jnt = jnt_root->FirstChildElement("joint");
      nullptr != jnt; jnt = jnt->NextSiblingElement("joint")) {
    // if false in the follow code, need to free this pointer
    if (nullptr == jnt->Attribute("name")) {
      std::stringstream ss;
      ss << "joint_" << robot->joint_res_map_.size();
      LOG(WARNING) << "The joint tag has no 'name' attribute, "
          << "we will use the default name: \"" << ss.str() << "\"";
      jnt->SetAttribute("name", ss.str());
    }
    QrDriver::JointResMap jnt_res;
    jnt_res.joint_name_ = jnt->Attribute("name");
    LOG(INFO) << "Assemble joint: \"" << jnt_res.joint_name_ << "\"";
    for (auto act = jnt->FirstChildElement("actuator");
        nullptr != act; act = act->NextSiblingElement("actuator")) {
      if (nullptr != act->Attribute("name")) {
        LOG(INFO) << "Push the " << jnt_res.actuator_names_.size()
                  << " actuator of the \"" << jnt_res.joint_name_ << "\" joint";
        jnt_res.actuator_names_.push_back(act->Attribute("name"));
      }
    }
    for (auto enc = jnt->FirstChildElement("encoder");
        nullptr != enc; enc = enc->NextSiblingElement("encoder")) {
      if (nullptr != enc->Attribute("name")) {
        LOG(INFO) << "Push the " << jnt_res.actuator_names_.size()
                  << " encoder of the \"" << jnt_res.joint_name_ << "\" joint";
        jnt_res.encoder_names_.push_back(enc->Attribute("name"));
      }
    }
    robot->joint_res_map_.push_back(jnt_res);
    nothing = false;
  }
  if (nothing) {
    LOG(WARNING) << "No joint states define";
  }
  return true;
}

bool RobotParser::parserRobotStateAndPropagate(QrDriver* robot) {
  // 寻找communication块， 并初始化propagate实例
  auto com_root = xml_root_->FirstChildElement("communication");
  if (nullptr == com_root) {
    LOG(ERROR) << "Could not found the 'communication' block.";
    return false;
  }
  if (nullptr == com_root->Attribute("name")) {
    LOG(WARNING) << "Could not found the 'name' attribute of the 'communication' block, using the default name 'Propagate'";
    com_root->SetAttribute("name", "Propagate");
  }
  PropagateImpBase* propagate = new PropagateImpBase(com_root->Attribute("name"));
  bool nothing = true;
  for (auto ch = com_root->FirstChildElement("channel");
      nullptr != ch; ch = ch->NextSiblingElement("communication")) {
    // if false in the follow code, need to free this pointer
    propagate->addImp(RobotParser::parserPropagate(ch));
    nothing = false;
  }
  if (nothing) {
    LOG(ERROR) << "Could not found the 'channel' block.";
    return false;
  }

  // 寻找hardware块， 并初始化RobotState实例
  auto hw_root = xml_root_->FirstChildElement("hardware");
  if (nullptr == hw_root) {
    LOG(ERROR) << "Could not found the 'hardware' block.";
    delete propagate;
    return false;
  }
  if (nullptr == hw_root->Attribute("name")) {
    LOG(WARNING) << "Could not found the 'name' attribute of the 'hardware' block, using the default name 'RobotState'";
    hw_root->SetAttribute("name", "RobotState");
  }
  RobotStateBase* robot_state = new RobotStateBase(hw_root->Attribute("name"));
  nothing = true;
  for (auto spec = hw_root->FirstChildElement("specification");
      spec != nullptr; spec = spec->NextSiblingElement("specification")) {
    auto state = parserRobotState(spec, propagate);
    if (nullptr != state) {
      robot_state->addRobotState(state);
      nothing = false;
    }
  }
  if (nothing) {
    LOG(ERROR) << "The RobotState is empty!";
    delete propagate;
    delete robot_state;
    return false;
  }
  // 增加每个状态part到robot_state
  hw_root = hw_root->FirstChildElement("part");
  int part_count(0);
  while (nullptr != hw_root) {
    if (nullptr == hw_root->Attribute("name")) {
      std::stringstream ss;
      ss << "part" << part_count;
      LOG(WARNING) << "Could not found the 'name' attribute of the 'part' block, using the default name '"
          << ss.str() << "'";
      hw_root->SetAttribute("name", ss.str());
    }
    ++part_count;
    RobotStateBase* part = new RobotStateBase(hw_root->Attribute("name"));
    nothing = true;
    for (auto spec = hw_root->FirstChildElement("specification");
        spec != nullptr; spec = spec->NextSiblingElement("specification")) {
      auto state = parserRobotState(spec, propagate);
      if (nullptr != state) {
        part->addRobotState(state);
        nothing = false;
      }
    }
    if (nothing) {
      LOG(WARNING) << "The part RobotState " << hw_root->Attribute("name")
          << "  is empty!";
      delete part;
    }
    hw_root = hw_root->NextSiblingElement("part");
  }

  robot->propagate_.reset(propagate);
  robot->robot_.reset(robot_state);
  LOG(INFO) << "Parser RobotState and PropagateImp successful!";
  return true;
}

PropagateImpBase* RobotParser::parserPropagate(TiXmlElement* root) {
  if ((nullptr == root->Attribute("name"))
        || (nullptr == root->Attribute("type"))) {
    LOG(ERROR) << "Wrong format 'communication' block";
    return nullptr;
  }

  std::string type = root->Attribute("type");
  PropagateImpBase* propagate = nullptr;
  if (0 == type.compare("PropagateImpPcan")) {
    propagate = new PropagateImpPcan(root->Attribute("name"));
  } else {
    LOG(ERROR) << "Unknown communication type: " << type;
  }
  return propagate;
}

RobotStateBase* RobotParser::parserRobotState(TiXmlElement* root, PropagateImpBase* propagate) {
  if (nullptr == root->Attribute("name")) {
    LOG(WARNING) << "Could not found the 'name' attribute of the 'hardware' block";
    return nullptr;
  }

  std::string name = root->Attribute("name");
  if (nullptr == root->Attribute("type")) {
    LOG(WARNING) << "Could not found the 'type' attribute in the "
        << name << " 'hardware' block";
    return nullptr;
  }
  std::string type = root->Attribute("type");
  if (hw_parser_map_.end() == hw_parser_map_.find(type)) {
    LOG(WARNING) << "Could not found the " << type << " interface corresponding to "
        << name << " 'hardware' block";
    return nullptr;
  }

  return hw_parser_map_[type](root, propagate);
}

template <class T>
T* getHWInstance(TiXmlElement* ele) {
  if (nullptr == ele->Attribute("name")) {
    return nullptr;
  }
  return new T(ele->Attribute("name"));
}

RobotStateBase* RobotParser::getActuator(TiXmlElement* root, PropagateImpBase* propagate) {
  Actuator* act = getHWInstance<Actuator>(root);
  if (nullptr == act) {
    LOG(ERROR) << "ERROR in get Actuator!";
    return nullptr;
  }
  // Optional parameters
  auto command = root->FirstChildElement("command");
  if (nullptr != command) {
    Actuator::CmdTypeSharedPtr motor
      = boost::dynamic_pointer_cast<Actuator::CmdType>(act->getCommand(act->getName()));

    if (nullptr != command->Attribute("mode")) {
      std::string mode = command->Attribute("mode");
      if (0 == mode.compare("position")) {
        motor->mode_ = MotorCmd::MODE_POS_;
      } else if (0 == mode.compare("velocity")) {
        motor->mode_ = MotorCmd::MODE_VEL_;
      } else if (0 == mode.compare("torque")) {
        motor->mode_ = MotorCmd::MODE_TOR_;
      } else {
        ; // Nothing to do
      }
    }
    if (nullptr != command->Attribute("value")) {
      std::stringstream ss;
      ss << command->Attribute("value");
      double cmd;
      ss >> cmd;
      motor->command_ = cmd;
    }
    act->setCommand(*motor);
  }

  HWStateSharedPtr s = boost::dynamic_pointer_cast<HWStateBase>(act->motor_state_);
  HWCmdSharedPtr c = boost::dynamic_pointer_cast<HWCommandBase>(act->motor_cmd_);
  if (nullptr != root->Attribute("channel")) {
    propagate->registerHandle(c, root->Attribute("channel"));
    propagate->registerHandle(s, root->Attribute("channel"));
  } else {
    propagate->registerHandle(c);
    propagate->registerHandle(s);
  }

  return act;
}

RobotStateBase* RobotParser::getEncoder(TiXmlElement* root, PropagateImpBase* propagate) {
  Encoder* encoder = getHWInstance<Encoder>(root);

  HWStateSharedPtr s = boost::dynamic_pointer_cast<HWStateBase>(encoder->state_);
  if (nullptr != root->Attribute("channel")) {
    propagate->registerHandle(s, root->Attribute("channel"));
  } else {
    propagate->registerHandle(s);
  }

  return encoder;
}

} /* namespace qr_driver */
