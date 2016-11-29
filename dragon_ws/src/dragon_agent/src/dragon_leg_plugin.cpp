#include "dragon_agent/dragon_leg_plugin.h"

#include <gps_agent_pkg/positioncontroller.h>
#include <gps_agent_pkg/trialcontroller.h>
#include <gps_agent_pkg/encodersensor.h>
#include <gps_agent_pkg/util.h>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace dragon_agent {

// Plugin constructor.
GPSDragonLegPlugin::GPSDragonLegPlugin()
    : controller_counter_(0),
      controller_step_length_(50)
{
    // Some basic variable initialization.
}

// Destructor.
GPSDragonLegPlugin::~GPSDragonLegPlugin()
{
    // Nothing to do here, since all instance variables are destructed automatically.
}

// Initialize the object and store the robot state.
bool GPSDragonLegPlugin::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n)
{
    ROS_WARN("Silence: controller init() is called!");
    // Variables.
    std::string root_name, tip_name;

    // Store the robot state.
    robot_ = robot;

    // Create FK solvers.
    // Get the name of the root.
    if(!n.getParam("root_name", root_name)) {
        ROS_ERROR("Property root_name not found in namespace: '%s'", n.getNamespace().c_str());
        return false;
    }

    // Get active and passive arm end-effector names.
    if(!n.getParam("tip_name", tip_name)) {
        ROS_ERROR("Property tip_name not found in namespace: '%s'", n.getNamespace().c_str());
        return false;
    }

    // Create KDL chains, solvers, etc.
    // KDL chains.
    urdf::Model robot_model;
    // TODO: by parameter?
    robot_model.initParam("robot_description");
    // Create active arm chain.
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)){
      ROS_ERROR("Could not convert urdf into kdl tree");
      return -1;
    }

    bool res;
    try{
      res = kdl_tree.getChain(root_name, tip_name, active_arm_fk_chain_);
    }
    catch(...){
      res = false;
    }
    if (!res){
      ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name.c_str(), tip_name.c_str());
      return false;
    }

    // Pose solvers.
    active_arm_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(active_arm_fk_chain_));

    // Jacobian sovlers.
    active_arm_jac_solver_.reset(new KDL::ChainJntToJacSolver(active_arm_fk_chain_));

    // Pull out joint states.
    int joint_index;

    // Put together joint states for the active arm.
    joint_index = 1;
    while (true)
    {
        // Check if the parameter for this active joint exists.
        std::string joint_name;
        std::string param_name = std::string("/joint_name_" + to_string(joint_index));
        if(!n.getParam(param_name.c_str(), joint_name))
            break;

        // Push back the joint state and name.
        active_arm_joint_handle_.push_back(robot_->getHandle(joint_name));
        active_arm_joint_names_.push_back(joint_name);

        // Increment joint index.
        joint_index++;
    }
    // Validate that the number of joints in the chain equals the length of the active arm joint state.
    ROS_INFO_STREAM("num_fk_chain: " + to_string(active_arm_fk_chain_.getNrOfJoints()));
    ROS_INFO_STREAM("num_joint_state: " + to_string(active_arm_joint_handle_.size()));
    if (active_arm_fk_chain_.getNrOfJoints() != active_arm_joint_handle_.size())
    {
        ROS_ERROR("Number of joints in the active arm FK chain does not match the number of joints in the active arm joint state!");
        return false;
    }

    // Allocate torques array.
    active_arm_torques_.resize(active_arm_fk_chain_.getNrOfJoints());

    // Initialize ROS subscribers/publishers, sensors, and position controllers.
    // Note that this must be done after the FK solvers are created, because the sensors
    // will ask to use these FK solvers!
    initialize(n);

    ROS_WARN("Silence: controller requestStart() is called!");
    return true;
}

// This is called by the controller manager before starting the controller.
void GPSDragonLegPlugin::starting(const ros::Time& time)
{
    ROS_WARN("Silence: controller starting() is called!");
    // Get current time.
    last_update_time_ = time;
    controller_counter_ = 0;

    // Reset all the sensors. This is important for sensors that try to keep
    // track of the previous state somehow.
    //for (int sensor = 0; sensor < TotalSensorTypes; sensor++)
    for (int sensor = 0; sensor < 1; sensor++)
    {
        sensors_[sensor]->reset(this,last_update_time_);
    }

    // Reset position controllers.
    passive_arm_controller_->reset(last_update_time_);
    active_arm_controller_->reset(last_update_time_);

    // Reset trial controller, if any.
    if (trial_controller_ != NULL) trial_controller_->reset(last_update_time_);
}

// This is called by the controller manager before stopping the controller.
void GPSDragonLegPlugin::stopping(const ros::Time& time)
{
    // Nothing to do here.
}

// This is the main update function called by the realtime thread when the controller is running.
void GPSDragonLegPlugin::update(const ros::Time& time, const ros::Duration& period)
{
    // Get current time.
    last_update_time_ = time;

    // Check if this is a controller step based on the current controller frequency.
    controller_counter_++;
    if (controller_counter_ >= controller_step_length_) controller_counter_ = 0;
    bool is_controller_step = (controller_counter_ == 0);

    // Update the sensors and fill in the current step sample.
    update_sensors(last_update_time_,is_controller_step);

    // Update the controllers.
    update_controllers(last_update_time_,is_controller_step);

    // Store the torques.
    for (unsigned i = 0; i < active_arm_joint_handle_.size(); i++)
        // active_arm_joint_handle_[i].commanded_effort_ = active_arm_torques_[i];
        active_arm_joint_handle_[i].setCommand(active_arm_torques_[i]);
}

// Get current time.
ros::Time GPSDragonLegPlugin::get_current_time() const
{
    return last_update_time_;
}

// Get current encoder readings (robot-dependent).
void GPSDragonLegPlugin::get_joint_encoder_readings(Eigen::VectorXd &angles, gps::ActuatorType arm) const
{
    if (arm == gps::AUXILIARY_ARM)
    {
        return;
    }
    else if (arm == gps::TRIAL_ARM)
    {
        if (angles.rows() != active_arm_joint_handle_.size())
            angles.resize(active_arm_joint_handle_.size());
        for (unsigned i = 0; i < angles.size(); i++)
            angles(i) = active_arm_joint_handle_[i].getPosition();
    }
    else
    {
        ROS_ERROR("Unknown ArmType %i requested for joint encoder xxx readings!",arm);
    }
}

} // end namespace gps_control

// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(dragon_agent, GPSDragonLegPlugin,
						dragon_agent::GPSDragonLegPlugin,
						controller_interface::ControllerBase)