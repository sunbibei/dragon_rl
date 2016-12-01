#include "gps_agent_lib/baxterplugin.h"
#include "gps_agent_lib/positioncontroller.h"
#include "gps_agent_lib/trialcontroller.h"
#include "gps_agent_lib/encodersensor.h"
#include "gps_agent_lib/util.h"

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace gps_control {

// Plugin constructor.
GPSBaxterPlugin::GPSBaxterPlugin()
{
    // Some basic variable initialization.
    controller_counter_ = 0;
    controller_step_length_ = 50;
}

// Destructor.
GPSBaxterPlugin::~GPSBaxterPlugin()
{
    // Nothing to do here, since all instance variables are destructed automatically.
}

// Initialize the object and store the robot state.
bool GPSBaxterPlugin::init(hardware_interface::EffortJointInterface* robot, ros::NodeHandle& n)
{
    ROS_WARN("Silence: controller init() is called!");
    // Variables.
    std::string root_name, active_tip_name, passive_tip_name;

    // Store the robot state.
    robot_ = robot;

    // Create FK solvers.
    // Get the name of the root.
    if(!n.getParam("root_name", root_name)) {
        ROS_ERROR("Property root_name not found in namespace: '%s'", n.getNamespace().c_str());
        return false;
    }

    // Get active and passive arm end-effector names.
    if(!n.getParam("active_tip_name", active_tip_name)) {
        ROS_ERROR("Property active_tip_name not found in namespace: '%s'", n.getNamespace().c_str());
        return false;
    }
    if(!n.getParam("passive_tip_name", passive_tip_name)) {
        ROS_ERROR("Property passive_tip_name not found in namespace: '%s'", n.getNamespace().c_str());
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
      res = kdl_tree.getChain(root_name, active_tip_name, active_arm_fk_chain_);
      res &= kdl_tree.getChain(root_name, passive_tip_name, passive_arm_fk_chain_);
    }
    catch(...){
      res = false;
    }
    if (!res){
      ROS_ERROR("Controller could not use the chain from '%s' to '%s'", root_name.c_str(), active_tip_name.c_str());
      return false;
    }

    // Pose solvers.
    passive_arm_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(passive_arm_fk_chain_));
    active_arm_fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(active_arm_fk_chain_));

    // Jacobian sovlers.
    passive_arm_jac_solver_.reset(new KDL::ChainJntToJacSolver(passive_arm_fk_chain_));
    active_arm_jac_solver_.reset(new KDL::ChainJntToJacSolver(active_arm_fk_chain_));

    // Pull out joint states.
    int joint_index;

    // Put together joint states for the active arm.
    joint_index = 1;
    while (true)
    {
        // Check if the parameter for this active joint exists.
        std::string joint_name;
        std::string param_name = std::string("/active_arm_joint_name_" + to_string(joint_index));
        if(!n.getParam(param_name.c_str(), joint_name))
            break;

        // Push back the joint state and name.
        // pr2_mechanism_model::JointState* jointState = robot_->getJointState(joint_name);
        // if fail, it will be throw exception.
        // hardware_interface::JointHandle jointHandle = ;
        active_arm_joint_handle_.push_back(robot_->getHandle(joint_name));
        // if (jointHandle == NULL)
        //     ROS_INFO_STREAM("jointHandle: " + joint_name + " is null");

        active_arm_joint_names_.push_back(joint_name);

        // Increment joint index.
        joint_index++;
    }
    // Validate that the number of joints in the chain equals the length of the active arm joint state.
    ROS_INFO_STREAM("num_fk_chain: " + to_string(active_arm_fk_chain_.getNrOfJoints()));
    ROS_INFO_STREAM("num_joint_state: " + to_string(active_arm_joint_handle_.size()));
    if (active_arm_fk_chain_.getNrOfJoints() != active_arm_joint_handle_.size())
    {
        // ROS_INFO_STREAM("num_fk_chain: " + to_string(active_arm_fk_chain_.getNrOfJoints()));
        // ROS_INFO_STREAM("num_joint_state: " + to_string(active_arm_joint_handle_.size()));
        ROS_ERROR("Number of joints in the active arm FK chain does not match the number of joints in the active arm joint state!");
        return false;
    }

    // Put together joint states for the passive arm.
    joint_index = 1;
    while (true)
    {
        // Check if the parameter for this passive joint exists.
        std::string joint_name;
        std::string param_name = std::string("/passive_arm_joint_name_" + to_string(joint_index));
        if(!n.getParam(param_name, joint_name))
            break;

        // Push back the joint state and name.
        // pr2_mechanism_model::JointState* jointState = robot_->getJointState(joint_name);
        // hardware_interface::JointHandle jointHandle = ;
        passive_arm_joint_handle_.push_back(robot_->getHandle(joint_name));
        // if (jointHandle == NULL)
        //     ROS_INFO_STREAM("jointHandle: " + joint_name + " is null");
        passive_arm_joint_names_.push_back(joint_name);

        // Increment joint index.
        joint_index++;
    }
    // Validate that the number of joints in the chain equals the length of the active arm joint state.
    if (passive_arm_fk_chain_.getNrOfJoints() != passive_arm_joint_handle_.size())
    {
        ROS_INFO_STREAM("num_fk_chain: " + to_string(passive_arm_fk_chain_.getNrOfJoints()));
        ROS_INFO_STREAM("num_joint_state: " + to_string(passive_arm_joint_handle_.size()));
        ROS_ERROR("Number of joints in the passive arm FK chain does not match the number of joints in the passive arm joint state!");
        return false;
    }

    // Allocate torques array.
    active_arm_torques_.resize(active_arm_fk_chain_.getNrOfJoints());
    passive_arm_torques_.resize(passive_arm_fk_chain_.getNrOfJoints());

    // Initialize ROS subscribers/publishers, sensors, and position controllers.
    // Note that this must be done after the FK solvers are created, because the sensors
    // will ask to use these FK solvers!
    initialize(n);

    ROS_WARN("Silence: controller requestStart() is called!");
    // startRequest(ros::Time::now());
    // Tell the PR2 controller manager that we initialized everything successfully.
    return true;
}

// This is called by the controller manager before starting the controller.
void GPSBaxterPlugin::starting(const ros::Time& time)
{
    ROS_WARN("Silence: controller starting() is called!");
    // Get current time.
    // last_update_time_ = robot_->getTime();
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
void GPSBaxterPlugin::stopping(const ros::Time& time)
{
    // Nothing to do here.
}

// This is the main update function called by the realtime thread when the controller is running.
void GPSBaxterPlugin::update(const ros::Time& time, const ros::Duration& period)
{
    // Get current time.
    // last_update_time_ = robot_->getTime();
    last_update_time_ = time;
    // ROS_WARN("Silence: UPDATING... ...");

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

    for (unsigned i = 0; i < passive_arm_joint_handle_.size(); i++)
        // passive_arm_joint_handle_[i].commanded_effort_ = passive_arm_torques_[i];
        passive_arm_joint_handle_[i].setCommand(passive_arm_torques_[i]);
}

// Get current time.
ros::Time GPSBaxterPlugin::get_current_time() const
{
    return last_update_time_;
}

// Get current encoder readings (robot-dependent).
void GPSBaxterPlugin::get_joint_encoder_readings(Eigen::VectorXd &angles, gps::ActuatorType arm) const
{
    if (arm == gps::AUXILIARY_ARM)
    {
        if (angles.rows() != passive_arm_joint_handle_.size())
            angles.resize(passive_arm_joint_handle_.size());
        for (unsigned i = 0; i < angles.size(); i++)
            angles(i) = passive_arm_joint_handle_[i].getPosition();
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
        ROS_ERROR("Unknown ArmType %i requested for joint encoder readings!",arm);
    }
}

} // end namespace gps_control

// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(gps_agent_pkg, GPSBaxterPlugin,
						gps_control::GPSBaxterPlugin,
						controller_interface::ControllerBase)
// PLUGINLIB_EXPORT_CLASS( gps_control::GPSBaxterPlugin, controller_interface::ControllerBase )
