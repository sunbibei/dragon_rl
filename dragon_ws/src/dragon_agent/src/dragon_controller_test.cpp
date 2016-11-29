/*
 * dragon_controller_test.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: silence
 */

#include <dragon_agent/dragon_controller_test.h>
#include <pluginlib/class_list_macros.h>

template <typename T>
std::string to_string(T value) {
    //create an output string stream
    std::ostringstream os ;
    //throw the value into the string stream
    os << value ;
    //convert the string stream into a string and return
    return os.str() ;
}

namespace dragon_agent {

DragonControllerTest::DragonControllerTest()
    : robot_(nullptr) { }

DragonControllerTest::~DragonControllerTest() {
  // Nothing to do here
}

bool DragonControllerTest::init(
    hardware_interface::EffortJointInterface* robot,
    ros::NodeHandle& nh) {
  robot_ = robot;

  // Pull out joint states.
  int joint_index;

  // Put together joint states for the active arm.
  joint_index = 1;
  while (true) {
    // Check if the parameter for this active joint exists.
    std::string joint_name;
    std::string param_name = std::string("/joint_name_" + to_string(joint_index));
    if(!nh.getParam(param_name.c_str(), joint_name))
        break;

    // Push back the joint state and name.
    joint_handles_.push_back(robot_->getHandle(joint_name));
    joint_names_.push_back(joint_name);
    // Increment joint index.
    joint_index++;
  }

  joint_torques_.resize(joint_handles_.size());
  for (auto& jnt : joint_torques_) {
    jnt = 5;
  }
  ROS_INFO_STREAM("num_joint_state: " + to_string(joint_handles_.size()));
  
  return true;
}

void DragonControllerTest::starting(const ros::Time&) {
  ;
}

void DragonControllerTest::stopping(const ros::Time&) {
  ;
}

void DragonControllerTest::update(const ros::Time&, const ros::Duration&) {
  for (unsigned i = 0; i < joint_handles_.size(); i++) {
    joint_handles_[i].setCommand(joint_torques_[i]);
  }
}

} /* namespace dragon_agent */

PLUGINLIB_EXPORT_CLASS(dragon_agent::DragonControllerTest, controller_interface::ControllerBase)