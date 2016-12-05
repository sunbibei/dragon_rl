/*
 * main.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */
/*
#include <iostream>
#include <boost/shared_ptr.hpp>

class Base {
public:
  Base():n(0) {};
  virtual ~Base() {};

  virtual void toString() {
    std::cout << "Base " << n << std::endl;
  }

private:
  double n;
};

class Driver : public Base {
public:
  Driver() : d('d') {};
  virtual ~Driver() {};

  virtual void toString() {
    std::cout << "Driver " << d << std::endl;
  }

private:
  char d;
};

int main() {
  boost::shared_ptr<Base> b(new Driver());
  b->toString();

  boost::shared_ptr<Driver> d(new Driver());
  boost::shared_ptr<Base> b1 = d;

  b1->toString();
  return 0;
}
*/

#include <propagate_interface/propagate_imp_base.h>
#include "qr_driver.h"

#include "hardware_interface/robot_state_base.h"
#include "hardware_interface/actuator.h"
#include "hardware_interface/encoder.h"
#include "robot_parser.h"

#include <iostream>

using namespace qr_driver;

int main(int argc, char* argv[]) {
  std::string xml = "robot.xml";
  if (2 == argc) {
    xml = argv[1];
  }
  QrDriver* robot_ = QrDriver::getInstance();

  if (!robot_->init(xml) || !robot_->start()) {
    LOG(FATAL) << "Launch the robot fail!";
    return -1;
  }

  LOG(INFO) << "Hello, World!";

  char c = 'y';
  HWStateSharedPtr state;
  HWCmdSharedPtr cmd;
  while (('y' == c) || ('Y' == c)) {
    state = robot_->robot_->getState("actuator1");
    //Encoder::StateTypeSharedPtr act_state
    //  = boost::dynamic_pointer_cast<Encoder::StateType>(state);
    std::cout << state->toString() << std::endl;
    /*
    act_state = boost::dynamic_pointer_cast<Actuator::StateType>(robot_->robot_->getState("actuator2"));
    std::cout << act_state->toString() << std::endl;
    Encoder::StateTypeSharedPtr enc_state
      = boost::dynamic_pointer_cast<Encoder::StateType>(robot_->robot_->getState("encoder1"));
    std::cout << enc_state->toString() << std::endl;
    enc_state = boost::dynamic_pointer_cast<Encoder::StateType>(robot_->robot_->getState("encoder2"));
    std::cout << enc_state->toString() << std::endl;

    Actuator::CmdTypeSharedPtr act_cmd
      = boost::dynamic_pointer_cast<Actuator::CmdType>(robot_->robot_->getCommand("actuator1"));
    std::cout << act_cmd->toString() << std::endl;
    act_cmd = boost::dynamic_pointer_cast<Actuator::CmdType>(robot_->robot_->getCommand("actuator2"));
    std::cout << act_cmd->toString() << std::endl;
    */

    std::cout << "Continue? ";
    std::cin >> c;
  }
}

