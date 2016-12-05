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

RobotParser::RobotParser() { }

RobotParser::~RobotParser() { }

/**
 * 完成解析函数MAP的初始化， 以及别的初始化工作
 */
void RobotParser::init() {
  RobotParser::hw_parser_map_.insert(
      std::make_pair("Actuator", RobotParser::getActuator));
  RobotParser::hw_parser_map_.insert(
      std::make_pair("Encoder", RobotParser::getEncoder));
}

bool RobotParser::parser(const std::string& filename, QrDriver* robot) {
  RobotParser::init();

  // 初始化XML文档相关内容
  TiXmlDocument* robot_xml = new TiXmlDocument();
  if (!robot_xml->LoadFile(filename)) {
    LOG(ERROR) << "Could not found the "<< filename << ", did you forget define the file?";
    return false;
  }
  // 寻找communication块， 并初始化propagate实例
  TiXmlElement* root = robot_xml->RootElement();  // Robot
  auto com_root = root->FirstChildElement("communication");
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
      nullptr != ch; ch = ch->NextSiblingElement()) {
    // if false in the follow code, need to free this pointer
    propagate->addImp(RobotParser::parserPropagate(ch));
    nothing = false;
  }
  if (nothing) {
    LOG(ERROR) << "Could not found the 'channel' block.";
    return false;
  }

  // 寻找hardware块， 并初始化RobotState实例
  auto hw_root = root->FirstChildElement("hardware");
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
      spec != nullptr; spec = spec->NextSiblingElement()) {
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
