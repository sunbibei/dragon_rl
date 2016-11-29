/*
 * dragon_controller_test.h
 *
 * The DragonControllerTest is uesed to test the gazebo_ros_control, and role as
 * A prototype for the agent.
 *
 *  Created on: Nov 27, 2016
 *      Author: silence
 */


#ifndef INCLUDE_DRAGON_AGENT_DRAGON_CONTROLLER_TEST_H_
#define INCLUDE_DRAGON_AGENT_DRAGON_CONTROLLER_TEST_H_

#include <vector>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <ros/ros.h>

namespace dragon_agent {

class DragonControllerTest
  : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    
public:
  DragonControllerTest();
  virtual ~DragonControllerTest();

public:
  bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh);
  void starting(const ros::Time&);
  void stopping(const ros::Time&);
  void update(const ros::Time&, const ros::Duration&);

private:
  // This is a pointer to the robot state(EffortJointInterface), which we get when initialized and have to keep after that.
  hardware_interface::EffortJointInterface* robot_;
  // Active arm joint states.
  std::vector<hardware_interface::JointHandle> joint_handles_;
  // Active arm joint names.
  std::vector<std::string> joint_names_;

  std::vector<double> joint_torques_;
};

} /* namespace dragon_agent */

#endif /* INCLUDE_DRAGON_AGENT_DRAGON_CONTROLLER_TEST_H_ */
