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
    as_(nh_,
      "follow_joint_trajectory",
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
  ; // Nothing to do here, all of variables initialize in the method @start()
}

bool QrRosWrapper::start() {
  bool debug = false;
  ros::param::get("~debug", debug);
  if (debug) {
    google::FlushLogFiles(-1);
    google::LogToStderr();
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
  ;
}

void QrRosWrapper::cancelCB(
    actionlib::ServerGoalHandle<
        control_msgs::FollowJointTrajectoryAction> gh) {
  ;
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

} /* namespace qr_driver */
