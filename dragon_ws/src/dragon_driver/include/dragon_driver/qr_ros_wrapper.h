/*
 * qr_ros_wrapper.h
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#ifndef INCLUDE_QR_ROS_WRAPPER_H_
#define INCLUDE_QR_ROS_WRAPPER_H_

#include "qr_driver.h"
#include "robot_parser.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/server_goal_handle.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <controller_manager/controller_manager.h>

namespace qr_driver {

class QrHardwareInterface;

class QrRosWrapper {

public:
  ~QrRosWrapper();
  // 获取QuadrupedRobotDriver对象实例
  static QrRosWrapper* getInstance();

  bool start();
  void halt();

  void goalCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction>);
  void cancelCB(actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction>);

private:
  QrRosWrapper();

  // 发布实时消息， 例如"/joint_states"
  void publishRTMsg();
  void rosControlLoop();

  // 测试消息回调函数
  void cbForDebug(const std_msgs::Int32ConstPtr&);
  ros::Subscriber cmd_sub_;

private:
  static QrRosWrapper* instance_;
  bool alive_;

  ros::NodeHandle nh_;
  // FollowJointTrjectoryAction服务器相关变量。 实现FollowJointTrajectory功能
  actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  actionlib::ServerGoalHandle<control_msgs::FollowJointTrajectoryAction> goal_handle_;
  bool has_goal_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;

  QrDriver* robot_;
  std::chrono::milliseconds rt_duration_; // 实时消息发布频率， 默认是50Hz(使用周期表示, 即20ms）
  std::chrono::milliseconds ros_ctrl_duration_; // ros_control_thread_循环频率， 默认是100Hz(使用周期表示, 即10ms）
  std::thread* rt_publish_thread_; // 该线程发布实时消息
  std::thread* ros_control_thread_; // 若启动ros_control机制， 该线程维护ros_control的正常流程
  bool use_ros_control_;
  boost::shared_ptr<QrHardwareInterface> hardware_interface_;
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};

} /* namespace qr_driver */

#endif /* INCLUDE_QR_ROS_WRAPPER_H_ */
