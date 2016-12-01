/*
 * gps_leg_plugin.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: silence
 */

#include <dragon_agent/gps_leg_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <gps_agent_lib/positioncontroller.h>

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

GPSLegPlugin::GPSLegPlugin()
    : robot_(nullptr), last_update_time_(0),
      controller_counter_(0), controller_step_length_(50)
{ }

GPSLegPlugin::~GPSLegPlugin() {
  // Nothing to do here
}

bool GPSLegPlugin::init(
    hardware_interface::EffortJointInterface* robot,
    ros::NodeHandle& nh) {
  robot_ = robot;

  // Create KDL chains, solvers, etc.
  // Prepare for back to the rest pose.
  if (!initialize_fk_solver(nh)) return false;
  // Push back the joint state and name.
  if (!initialize_joint(nh)) return false;

  active_arm_torques_.resize(joint_handles_.size());
  passive_arm_torques_.resize(joint_handles_.size());
  // Initialize ROS subscribers/publishers, sensors, and position controllers.
  // Note that this must be done after the FK solvers are created, because the sensors
  // will ask to use these FK solvers!
  initialize(nh);

  return true;
}

bool GPSLegPlugin::initialize_fk_solver(ros::NodeHandle& nh) {
  // Variables.
  std::string root_name, tip_name;
  // Create FK solvers.
  // Get the name of the root.
  if(!nh.getParam("root_name", root_name)) {
     ROS_ERROR("Property root_name not found in namespace: '%s'", nh.getNamespace().c_str());
     return false;
  }
  // Get active and passive arm end-effector names.
  if(!nh.getParam("tip_name", tip_name)) {
     ROS_ERROR("Property tip_name not found in namespace: '%s'", nh.getNamespace().c_str());
     return false;
  }

  // KDL chains.
  urdf::Model robot_model;
  // TODO: by parameter?
  robot_model.initParam("robot_description");
  // Create active arm chain.
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)){
    ROS_ERROR("Could not convert urdf into kdl tree");
    return false;
  }

  if (!kdl_tree.getChain(root_name, tip_name, active_arm_fk_chain_)){
    ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name.c_str(), tip_name.c_str());
    return false;
  }
  // Pose solvers.
  active_arm_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(active_arm_fk_chain_));
  passive_arm_fk_solver_ = active_arm_fk_solver_;

  active_arm_jac_solver_.reset(new KDL::ChainJntToJacSolver(active_arm_fk_chain_));
  passive_arm_jac_solver_ = active_arm_jac_solver_;

  return true;
}

bool GPSLegPlugin::initialize_joint(ros::NodeHandle& nh) {
  // Pull out joint states.
  int joint_index = 1;
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

  // Validate that the number of joints in the chain equals the length of the active arm joint state.
  ROS_INFO_STREAM("num_fk_chain: " + to_string(active_arm_fk_chain_.getNrOfJoints()));
  ROS_INFO_STREAM("num_joint_state: " + to_string(joint_handles_.size()));
  if (active_arm_fk_chain_.getNrOfJoints() != joint_handles_.size()) {
      ROS_ERROR("Number of joints in the active arm FK chain does not match the number of joints in the active arm joint state!");
      return false;
  }

  std::string joint_name;
  if(!nh.getParam("/tool_joint", joint_name)) {
    ROS_ERROR("Could not found the tool joint parameter, Did you forget setting the parameter for tool joint?");
    return false;
  }
  // Get the tool joint state and name.
  tool_joint_handle_ = robot_->getHandle(joint_name);
  tool_joint_name_ = joint_name;
  ROS_INFO_STREAM("tool joint name: " + tool_joint_name_);

  return true;
}

void GPSLegPlugin::starting(const ros::Time&) {
  ;
}

void GPSLegPlugin::stopping(const ros::Time&) {
  ;
}

void GPSLegPlugin::update(const ros::Time& time, const ros::Duration& duration) {
  // Get current time.
  last_update_time_ = time;

  // Check if this is a controller step based on the current controller frequency.
  controller_counter_++;
  if (controller_counter_ >= controller_step_length_) controller_counter_ = 0;
  bool is_controller_step = (controller_counter_ == 0);

  // ROS_ERROR("Updating The All Sensors (%d)", controller_counter_);
  // Update the sensors and fill in the current step sample.
  update_sensors(last_update_time_, is_controller_step);

  // ROS_ERROR("Updating The All Controllers (%d)", controller_counter_);
  // Update the controllers.
  update_controllers(last_update_time_, is_controller_step);

  // Store the torques.
  for (unsigned i = 0; i < joint_handles_.size(); i++)
    joint_handles_[i].setCommand(active_arm_torques_[i]);
}

// Initialize position controllers.
void GPSLegPlugin::initialize_position_controllers(ros::NodeHandle& n)
{
    // Create passive arm position controller.
    // TODO: fix this to be something that comes out of the robot itself
    // Silence: Get The number of Joint by the ROS Parameters
    int joint_num = 2;
    if (!n.getParam("/joint_num", joint_num)) {
        ROS_WARN("Property joint_num not found in namespace: '/'");
        ROS_WARN("Using the default joint_num: joint_num = %d", joint_num);
    }
    bool is_use_aux_arm = false;
    if (n.getParam("/use_aux_arm", is_use_aux_arm) && is_use_aux_arm) {
      ROS_WARN("Configuration the AUXILIARY_ARM");
      passive_arm_controller_.reset(new gps_control::PositionController(n, gps::AUXILIARY_ARM, joint_num));
    }

    // Create active arm position controller.
    active_arm_controller_.reset(new gps_control::PositionController(n, gps::TRIAL_ARM, joint_num));
}

// Update the controllers at each time step.
void GPSLegPlugin::update_controllers(ros::Time current_time, bool is_controller_step)
{
    // Update passive arm controller.
    // TODO - don't pass in wrong sample if used
  if (nullptr != passive_arm_controller_.get())
    passive_arm_controller_->update(this, current_time, current_time_step_sample_, passive_arm_torques_);

  bool trial_init = trial_controller_ != NULL && trial_controller_->is_configured() && controller_initialized_;
  if(!is_controller_step && trial_init){
      return;
  }

  // If we have a trial controller, update that, otherwise update position controller.
  if (trial_init) trial_controller_->update(this, current_time, current_time_step_sample_, active_arm_torques_);
  else active_arm_controller_->update(this, current_time, current_time_step_sample_, active_arm_torques_);

  // Check if the trial controller finished and delete it.
  if (trial_init && trial_controller_->is_finished()) {

      // Publish sample after trial completion
      publish_sample_report(current_time_step_sample_, trial_controller_->get_trial_length());
      //Clear the trial controller.
      trial_controller_->reset(current_time);
      trial_controller_.reset(NULL);

      // Set the active arm controller to NO_CONTROL.
      gps_control::OptionsMap options;
      options["mode"] = gps::NO_CONTROL;
      active_arm_controller_->configure_controller(options);

      // Switch the sensors to run at full frequency.
      for (int sensor = 0; sensor < gps_control::TotalSensorTypes; sensor++)
      {
          //sensors_[sensor]->set_update(active_arm_controller_->get_update_delay());
      }
  }
  if (active_arm_controller_->report_waiting){
    if (active_arm_controller_->is_finished()){
        publish_sample_report(current_time_step_sample_);
        active_arm_controller_->report_waiting = false;
    }
  }
  if ((nullptr != passive_arm_controller_.get()) && passive_arm_controller_->report_waiting){
      if (passive_arm_controller_->is_finished()){
          publish_sample_report(current_time_step_sample_);
          passive_arm_controller_->report_waiting = false;
      }
  }

}

// Get current encoder readings (robot-dependent).
void GPSLegPlugin::get_joint_encoder_readings(Eigen::VectorXd &angles, gps::ActuatorType arm) const {
    if (arm == gps::AUXILIARY_ARM)
    {
        return;
    }
    else if (arm == gps::TRIAL_ARM)
    {
        if (angles.rows() != joint_handles_.size())
          angles.resize(joint_handles_.size());

        for (unsigned i = 0; i < angles.size(); i++)
            angles(i) = joint_handles_[i].getPosition();
    }
    else
    {
        ROS_ERROR("Unknown ArmType %i requested for joint encoder readings!", arm);
    }
}

// Get current tool joint encoder readings (robot-dependent).
void GPSLegPlugin::get_tool_joint_encoder_readings(Eigen::VectorXd &angles, gps::ActuatorType arm) const {
    if (arm == gps::AUXILIARY_ARM)
    {
        return;
    }
    else if (arm == gps::TRIAL_ARM)
    {
      angles.resize(1);
      angles(0) = tool_joint_handle_.getPosition();
    }
    else
    {
        ROS_ERROR("Unknown ArmType %i requested for tool joint encoder readings!", arm);
    }
}

} /* namespace dragon_agent */

PLUGINLIB_EXPORT_CLASS(dragon_agent::GPSLegPlugin, controller_interface::ControllerBase)
