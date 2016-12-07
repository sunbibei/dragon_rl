/*
 * qr_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#include <glog/logging.h>
#include <qr_ros_wrapper.h>

#include "hardware_interface/qr_hardware_interface.h"
#include "hardware_interface/encoder.h"
#include "hardware_interface/actuator.h"

namespace qr_driver {

QrRosWrapper* QrRosWrapper::instance_ = nullptr;

QrRosWrapper* QrRosWrapper::getInstance() {
  if (nullptr == instance_) {
    instance_ = new QrRosWrapper;
    LOG(INFO) << "Create the singleton instance: QrRosWrapper";
  }

  LOG(INFO) << "Return the singleton instance: QrRosWrapper";
  return instance_;
}

QrRosWrapper::QrRosWrapper()
  : alive_(true),
    as_(nh_, "follow_joint_trajectory",
      boost::bind(&QrRosWrapper::goalCB, this, _1),
      boost::bind(&QrRosWrapper::cancelCB, this, _1),
      false),
    has_goal_(false),
    robot_(nullptr),
    rt_duration_(1000/50),
    ros_ctrl_duration_(1000/100),
    rt_publish_thread_(nullptr),
    ros_control_thread_(nullptr),
    use_ros_control_(true) {
  google::InitGoogleLogging("qr_driver");
  // google::SetLogDestination(google::GLOG_INFO, "/path/to/log/INFO_");
  // google::LogMessage::Init();
  FLAGS_colorlogtostderr = true;
  google::FlushLogFiles(google::GLOG_INFO);
  ; // Nothing to do here, all of variables initialize in the method @start()
}

bool QrRosWrapper::start() {
  bool debug = false;
  ros::param::get("~debug", debug);
  if (debug) {
    google::SetStderrLogging(google::GLOG_INFO);
  } else {
    google::SetStderrLogging(google::GLOG_WARNING);
  }
  
  robot_ = QrDriver::getInstance();
  std::string speci = "../robot.xml";
  if (ros::param::get("~robot_xml", speci)) {
    if (!robot_->initFromFile(speci)) {
      LOG(ERROR) << "Launch the robot fail from " << speci << " file";
      return false;
    }
  }
  if (ros::param::get("~robot_xml_string", speci)) {
    if (!robot_->initFromParam(speci)) {
      LOG(ERROR) << "Launch the robot fail from parameter 'robot_xml_string'";
      return false;
    }
  }
  if (!robot_->isInit()) {
    LOG(ERROR) << "Start QrRosWrapper FAIL";
    return false;
  }

  ros::param::get("~use_ros_control", use_ros_control_);
  if (use_ros_control_) {
    hardware_interface_.reset(
              new qr_driver::QrHardwareInterface(nh_, robot_));
    controller_manager_.reset(
        new controller_manager::ControllerManager(
            hardware_interface_.get(), nh_));
  }

  if (robot_->start()) {
    if (use_ros_control_) {
      double frequency = 100.0;
      ros::param::get("~ctrl_loop_frequency", frequency);
      if (frequency > 0)
        ros_ctrl_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

      ros_control_thread_ = new std::thread(
          boost::bind(&QrRosWrapper::rosControlLoop, this));
      LOG(INFO) << "The control thread for this driver has been started";
    } else {
      // start actionserver
      has_goal_ = false;
      as_.start();

      double frequency = 50.0;
      ros::param::get("~rt_frequency", frequency);
      if (frequency > 0)
        rt_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

      // 若启动ros_control, 则使用joint_state_controller来发布该/joint_states
      rt_publish_thread_ = new std::thread(
          boost::bind(&QrRosWrapper::publishRTMsg, this));
      LOG(INFO) << "The action server for this driver has been started";
    }
  } else {
    LOG(ERROR) << "The robot thread which for the real-time message has failed";
    return false;
  }

  // For debug
  cmd_sub_ = nh_.subscribe<std_msgs::Int32>("debug", 100,
      &QrRosWrapper::cbForDebug, this);

  return true;
}

void QrRosWrapper::halt() {
  alive_ = false;
  if (nullptr != rt_publish_thread_) rt_publish_thread_->join();
  if (nullptr != ros_control_thread_) ros_control_thread_->join();
  if (nullptr != robot_) robot_->halt();
}

void QrRosWrapper::goalCB(
    actionlib::ServerGoalHandle<
        control_msgs::FollowJointTrajectoryAction> gh) {
  std::string buf;
  LOG(INFO) << "on_goal";
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal
    = *gh.getGoal(); //make a copy that we can modify
  if (has_goal_) {
    LOG(WARNING) << "Received new goal while still executing previous trajectory. "
        << "Canceling previous trajectory";
    has_goal_ = false;
    robot_->stopTraj();
    result_.error_code = -100; // nothing is defined for this...?
    result_.error_string = "Received another trajectory";
    goal_handle_.setAborted(result_, result_.error_string);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  goal_handle_ = gh;
  if (!validateJointNames()) {
    std::string outp_joint_names = "";
    for (unsigned int i = 0; i < goal.trajectory.joint_names.size();
        i++) {
      outp_joint_names += goal.trajectory.joint_names[i] + " ";
    }
    result_.error_code = result_.INVALID_JOINTS;
    result_.error_string =
        "Received a goal with incorrect joint names: "
            + outp_joint_names;
    gh.setRejected(result_, result_.error_string);
    LOG(ERROR) << result_.error_string;
    return;
  }
  if (!has_positions()) {
    result_.error_code = result_.INVALID_GOAL;
    result_.error_string = "Received a goal without positions";
    gh.setRejected(result_, result_.error_string);
    LOG(ERROR) << result_.error_string;
    return;
  }

  if (!has_velocities()) {
    result_.error_code = result_.INVALID_GOAL;
    result_.error_string = "Received a goal without velocities";
    gh.setRejected(result_, result_.error_string);
    LOG(ERROR) << result_.error_string;
    return;
  }

  if (!traj_is_finite()) {
    result_.error_string = "Received a goal with infinities or NaNs";
    result_.error_code = result_.INVALID_GOAL;
    gh.setRejected(result_, result_.error_string);
    LOG(ERROR) << result_.error_string;
    return;
  }

  /* TODO 未实现
  if (!has_limited_velocities()) {
    result_.error_code = result_.INVALID_GOAL;
    result_.error_string =
        "Received a goal with velocities that are higher than "
            + std::to_string(max_velocity_);
    gh.setRejected(result_, result_.error_string);
    LOG(ERROR) << result_.error_string;
    return;
  }
  */

  reorder_traj_joints(goal.trajectory);

  if (!start_positions_match(goal.trajectory, 0.01)) {
    result_.error_code = result_.INVALID_GOAL;
    result_.error_string = "Goal start doesn't match current pose";
    gh.setRejected(result_, result_.error_string);
    LOG(ERROR) << result_.error_string;
    return;
  }

  std::vector<double> timestamps;
  std::vector<std::vector<double>> positions, velocities;
  if (goal.trajectory.points[0].time_from_start.toSec() != 0.) {
    LOG(WARNING) << "Trajectory's first point should be the current position, "
        << "with time_from_start set to 0.0 - Inserting point in malformed trajectory";
    timestamps.push_back(0.0);
    positions.push_back(robot_->getJointPositions());
    velocities.push_back(robot_->getJointVelocities());
  }
  for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
    timestamps.push_back(
        goal.trajectory.points[i].time_from_start.toSec());
    positions.push_back(goal.trajectory.points[i].positions);
    velocities.push_back(goal.trajectory.points[i].velocities);

  }

  goal_handle_.setAccepted();
  has_goal_ = true;
  std::thread(&QrRosWrapper::trajThread, this, timestamps, positions,
      velocities).detach();
}

void QrRosWrapper::trajThread(std::vector<double> timestamps,
      std::vector<std::vector<double>> positions,
      std::vector<std::vector<double>> velocities) {
  robot_->doTraj(timestamps, positions, velocities);
  if (has_goal_) {
    result_.error_code = result_.SUCCESSFUL;
    goal_handle_.setSucceeded(result_);
    has_goal_ = false;
  }
}

void QrRosWrapper::cancelCB(
    actionlib::ServerGoalHandle<
        control_msgs::FollowJointTrajectoryAction> gh) {
  // set the action state to preempted
  LOG(INFO) << "on_cancel";
  if (has_goal_) {
    if (gh == goal_handle_) {
      robot_->stopTraj();
      has_goal_ = false;
    }
  }
  result_.error_code = -100; //nothing is defined for this...?
  result_.error_string = "Goal cancelled by client";
  gh.setCanceled(result_);

  LOG(INFO) << result_.error_string;
}

void QrRosWrapper::publishRTMsg() {
  ros::Publisher joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  std::chrono::high_resolution_clock::time_point t0;
  std::chrono::milliseconds sleep_time;

  t0 = std::chrono::high_resolution_clock::now();
  while (alive_ && ros::ok()) {
    // TODO 待实现
    sensor_msgs::JointState joint_msg;
    joint_msg.header.stamp = ros::Time::now();
    robot_->getJointStates(joint_msg);
    joint_pub.publish(joint_msg);

    // 控制发布Message的频率
    sleep_time = rt_duration_ - std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - t0);
    if (sleep_time.count() > 0) {
      // keep real-time message frequency
      std::this_thread::sleep_for(sleep_time);
    }
    t0 = std::chrono::high_resolution_clock::now();
  }
}

void QrRosWrapper::rosControlLoop() {
  ros::Duration elapsed_time;
  struct timespec last_time, current_time;
  static const double BILLION = 1000000000.0;
  clock_gettime(CLOCK_MONOTONIC, &last_time);

  std::chrono::high_resolution_clock::time_point t0;
  std::chrono::milliseconds sleep_time;

  t0 = std::chrono::high_resolution_clock::now();
  while (alive_ && ros::ok()) {
    // Input
    hardware_interface_->read();
    //robot_.rt_interface_->robot_state_->setControllerUpdated();

    // Control
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time = ros::Duration(current_time.tv_sec - last_time.tv_sec + (current_time.tv_nsec - last_time.tv_nsec)/ BILLION);
    controller_manager_->update(ros::Time::now(), elapsed_time);
    last_time = current_time;

    // Output
    hardware_interface_->write();

    // 控制发布循环的频率
    sleep_time = ros_ctrl_duration_ - std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - t0);
    // LOG(INFO) << "sleep_time: " << sleep_time.count();
    if (sleep_time.count() > 0) {
      // keep real-time message frequency
      std::this_thread::sleep_for(sleep_time);
    }
    t0 = std::chrono::high_resolution_clock::now();
  }
}

QrRosWrapper::~QrRosWrapper() {
  halt();
  if (nullptr != robot_) {
    delete robot_;
    robot_ = nullptr;
  }
  if (nullptr != instance_) {
    delete instance_;
  }

  google::ShutdownGoogleLogging();
}

void QrRosWrapper::cbForDebug(const std_msgs::Int32ConstPtr& msg) {
  /*// 实现方式0
  LOG(INFO) << "test write style 0";
  for (auto& jnt : robot_->joint_res_map_) {
    Actuator::CmdType cmd(jnt.actuator_names_[0]);
    cmd.mode_ = Actuator::CmdType::MODE_POS_;
    cmd.command_ = msg->data;
    robot_->addCommand(cmd);
  }
  // 实现方式1
  LOG(INFO) << "test write style 1";
  for (auto& jnt : robot_->joint_res_map_) {
    Actuator::CmdTypeSharedPtr cmd(new Actuator::CmdType(jnt.actuator_names_[0]));
    cmd->mode_ = Actuator::CmdType::MODE_POS_;
    cmd->command_ = msg->data;
    robot_->addCommand(cmd);
  }*/
  // 实现方式2
  LOG(INFO) << "test write style 2";
  std::vector<HWCmdSharedPtr> cmd_vec;
  for (auto& jnt : robot_->joint_res_map_) {
    Actuator::CmdTypeSharedPtr cmd(new Actuator::CmdType(jnt.actuator_names_[0]));
    cmd->mode_ = Actuator::CmdType::MODE_POS_;
    cmd->command_ = msg->data;
    cmd_vec.push_back(cmd);
  }
  robot_->addCommand(cmd_vec);
}

bool QrRosWrapper::validateJointNames() {
  std::vector<std::string> actual_joint_names = robot_->getJointNames();
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
      *goal_handle_.getGoal();
  if (goal.trajectory.joint_names.size() != actual_joint_names.size())
    return false;

  for (unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
    unsigned int j;
    for (j = 0; j < actual_joint_names.size(); j++) {
      if (goal.trajectory.joint_names[i] == actual_joint_names[j])
        break;
    }
    if (goal.trajectory.joint_names[i] == actual_joint_names[j]) {
      actual_joint_names.erase(actual_joint_names.begin() + j);
    } else {
      return false;
    }
  }

  return true;
}

bool QrRosWrapper::has_velocities() {
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
      *goal_handle_.getGoal();
  for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
    if (goal.trajectory.points[i].positions.size()
        != goal.trajectory.points[i].velocities.size())
      return false;
  }
  return true;
}

bool QrRosWrapper::has_positions() {
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
      *goal_handle_.getGoal();
  if (goal.trajectory.points.size() == 0)
    return false;
  for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
    if (goal.trajectory.points[i].positions.size()
        != goal.trajectory.joint_names.size())
      return false;
  }
  return true;
}

bool QrRosWrapper::traj_is_finite() {
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::Goal goal =
      *goal_handle_.getGoal();
  for (unsigned int i = 0; i < goal.trajectory.points.size(); i++) {
    for (unsigned int j = 0;
        j < goal.trajectory.points[i].velocities.size(); j++) {
      if (!std::isfinite(goal.trajectory.points[i].positions[j]))
        return false;
      if (!std::isfinite(goal.trajectory.points[i].velocities[j]))
        return false;
    }
  }
  return true;
}

void QrRosWrapper::reorder_traj_joints(trajectory_msgs::JointTrajectory& traj) {
  /* Reorders trajectory - destructive */
  std::vector<std::string> actual_joint_names = robot_->getJointNames();
  std::vector<unsigned int> mapping;
  mapping.resize(actual_joint_names.size(), actual_joint_names.size());
  for (unsigned int i = 0; i < traj.joint_names.size(); i++) {
    for (unsigned int j = 0; j < actual_joint_names.size(); j++) {
      if (traj.joint_names[i] == actual_joint_names[j])
        mapping[j] = i;
    }
  }
  traj.joint_names = actual_joint_names;
  std::vector<trajectory_msgs::JointTrajectoryPoint> new_traj;
  for (unsigned int i = 0; i < traj.points.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint new_point;
    for (unsigned int j = 0; j < traj.points[i].positions.size(); j++) {
      new_point.positions.push_back(
          traj.points[i].positions[mapping[j]]);
      new_point.velocities.push_back(
          traj.points[i].velocities[mapping[j]]);
      if (traj.points[i].accelerations.size() != 0)
        new_point.accelerations.push_back(
            traj.points[i].accelerations[mapping[j]]);
    }
    new_point.time_from_start = traj.points[i].time_from_start;
    new_traj.push_back(new_point);
  }
  traj.points = new_traj;
}

bool QrRosWrapper::start_positions_match(const trajectory_msgs::JointTrajectory &traj, double eps) {
  for (size_t i = 0; i < traj.points[0].positions.size(); i++) {
    std::vector<double> qActual = robot_->getJointPositions();
    if( fabs(traj.points[0].positions[i] - qActual[i]) > eps ) {
      return false;
    }
  }
  return true;
}

} /* namespace qr_driver */
