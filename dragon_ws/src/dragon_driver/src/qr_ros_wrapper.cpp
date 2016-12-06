/*
 * qr_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#include <glog/logging.h>
#include <qr_ros_wrapper.h>

#include "hardware_interface/qr_hardware_interface.h"

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
    rt_publish_thread_(nullptr),
    ros_control_thread_(nullptr),
    use_ros_control_(false) {
  google::InitGoogleLogging("qr_driver");
  google::FlushLogFiles(-1);
  google::LogToStderr();
  ; // Nothing to do here, all of variables initialize in the method @start()
}

bool QrRosWrapper::start() {
  robot_ = QrDriver::getInstance();
  std::string speci = "../robot.xml";
  ros::param::get("~robot_xml_string", speci);
  if (!robot_->initParam(speci)) {
    LOG(ERROR) << "Launch the robot fail from parameter 'robot_xml_string'";
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
      ros_control_thread_ = new std::thread(
          boost::bind(&QrRosWrapper::rosControlLoop, this));
      LOG(INFO) << "The control thread for this driver has been started";
    }
    // start actionserver
    has_goal_ = false;
    as_.start();

    rt_publish_thread_ = new std::thread(
        boost::bind(&QrRosWrapper::publishRTMsg, this));
    LOG(INFO) << "The action server for this driver has been started";
  } else {
    LOG(ERROR) << "The robot thread which for the real-time message has failed";
    return false;
  }

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
  }
}

QrRosWrapper::~QrRosWrapper() {
  halt();
  if (nullptr != instance_) {
    delete instance_;
  }
}

} /* namespace qr_driver */
