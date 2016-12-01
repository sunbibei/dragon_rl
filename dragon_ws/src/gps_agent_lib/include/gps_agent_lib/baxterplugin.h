/*
This is the PR2-specific version of the robot plugin.
*/
#pragma once

// Headers.
#include <Eigen/Dense>
#include <pluginlib/class_list_macros.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <effort_controllers/joint_effort_controller.h>

// Superclass.
#include "gps_agent_lib/robotplugin.h"
#include "gps_agent_lib/controller.h"
#include "gps_agent_lib/positioncontroller.h"
#include "gps_agent_lib/encodersensor.h"
#include "gps/proto/gps.pb.h"

namespace gps_control
{

class GPSBaxterPlugin: public RobotPlugin, public controller_interface::Controller<
hardware_interface::EffortJointInterface>
{
public:
    // Constructor (this should do nothing).
    GPSBaxterPlugin();
    // Destructor.
    virtual ~GPSBaxterPlugin();
    bool init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& nh);
    void starting(const ros::Time&);
    void stopping(const ros::Time&);
    void update(const ros::Time&, const ros::Duration&);
    // Accessors.
    // Get current time.
    virtual ros::Time get_current_time() const;
    // Get current encoder readings (robot-dependent).
    virtual void get_joint_encoder_readings(Eigen::VectorXd &angles, gps::ActuatorType arm) const;

private:
    // PR2-specific chain object necessary to construct the KDL chain.
    // pr2_mechanism_model::Chain passive_arm_chain_, active_arm_chain_;
    // This is a pointer to the robot state(EffortJointInterface), which we get when initialized and have to keep after that.
    hardware_interface::EffortJointInterface* robot_;

    // Create an effort-based joint effort controller for every joint
    // std::vector<boost::shared_ptr<effort_controllers::JointEffortController> > effort_controllers_;

    // Passive arm joint states.
    std::vector<hardware_interface::JointHandle> passive_arm_joint_handle_;
    // Active arm joint states.
    std::vector<hardware_interface::JointHandle> active_arm_joint_handle_;
    // Passive arm joint names.
    std::vector<std::string> passive_arm_joint_names_;
    // Active arm joint names.
    std::vector<std::string> active_arm_joint_names_;
    // Time of last state update.
    ros::Time last_update_time_;
    // Counter for keeping track of controller steps.
    int controller_counter_;
    // Length of controller steps in ms.
    int controller_step_length_;
};

} // end namespace gps_control
