/*
 * gps_leg_plugin.h
 *
 *  Created on: Nov 27, 2016
 *      Author: silence
 */

#ifndef INCLUDE_DRAGON_AGENT_GPS_LEG_PLUGIN_H_
#define INCLUDE_DRAGON_AGENT_GPS_LEG_PLUGIN_H_

#include <gps_agent_lib/robotplugin.h>

#include <vector>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace dragon_agent {

class GPSLegPlugin
    : public gps_control::RobotPlugin,
      public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  GPSLegPlugin();
  virtual ~GPSLegPlugin();

public:
  bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh);
  void starting(const ros::Time&);
  void stopping(const ros::Time&);
  void update(const ros::Time&, const ros::Duration&);

  // These two interface is the abstract function in the RobotPlugin.
  // Get current time.
  virtual ros::Time get_current_time() const {return last_update_time_;};
  // Get current encoder readings (robot-dependent).
  virtual void get_joint_encoder_readings(Eigen::VectorXd &angles, gps::ActuatorType arm) const;

private:
  bool initialize_fk_solver(ros::NodeHandle&);
  bool initialize_joint(ros::NodeHandle&);

private:
  // This is a pointer to the robot state(EffortJointInterface), which we get when initialized and have to keep after that.
  hardware_interface::EffortJointInterface* robot_;
  // Active arm joint states.
  std::vector<hardware_interface::JointHandle> joint_handles_;
  // Active arm joint names.
  std::vector<std::string> joint_names_;

  // Time of last state update, only for the function -- @get_current_time().
  ros::Time last_update_time_;
  // Counter for keeping track of controller steps.
  int controller_counter_;
  // Length of controller steps in ms. default = 50(Indeed, Useless!!)
  int controller_step_length_;
};

} /* namespace dragon_agent */

#endif /* INCLUDE_DRAGON_AGENT_GPS_LEG_PLUGIN_H_ */
