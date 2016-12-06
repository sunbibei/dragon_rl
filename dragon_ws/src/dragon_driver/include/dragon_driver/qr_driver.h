/*
 * qr_driver.h
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#ifndef QUADRUPED_ROBOT_DRIVER_H_
#define QUADRUPED_ROBOT_DRIVER_H_

#include <thread>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/JointState.h>

namespace qr_driver {

class HWCommandBase;
class RobotStateBase;
class PropagateImpBase;
typedef boost::shared_ptr<HWCommandBase> HWCmdSharedPtr;
typedef boost::shared_ptr<PropagateImpBase> PropagateSharedPtr;
typedef boost::shared_ptr<RobotStateBase> RobotStateSharedPtr;

class QrDriver {
public:

  PropagateSharedPtr  propagate_;
  RobotStateSharedPtr robot_;

  typedef struct {
    std::string joint_name_;
    std::vector<std::string> actuator_names_;
    std::vector<std::string> encoder_names_;
  } JointResMap;
  std::vector<JointResMap> joint_res_map_;

  // 初始化所有变量, 以及线程等.
  bool initFromFile(const std::string& xml = "robot.xml");
  bool initFromParam(const std::string& param);
  bool isInit() const { return ((nullptr != propagate_) && (nullptr != robot_));}
  // 开始运行
  bool start();
  // 停止运行
  void halt();
  // 设定命令, 并传递给机器人
  void addCommand(const HWCommandBase&);
  // 设定命令, 并传递给机器人
  void addCommand(const HWCmdSharedPtr&);
  // 设定命令, 并传递给机器人
  void addCommand(const std::vector<HWCmdSharedPtr>&);
  /**
   * 获取Joint的名称
   */
  std::vector<std::string> getJointNames();
  /**
   * Actual joint positions
   */
  std::vector<double> getJointPositions();
  /**
   * Actual joint velocities
   */
  std::vector<double> getJointVelocities();
  /**
   * Actual joint torques TODO NO IMPLEMENTS
   */
  std::vector<double> getJointTorques();
  /**
   * Actual JointState( Recommended )
   */
  void getJointStates(sensor_msgs::JointState&);

public:
  ~QrDriver();
  // 获取QuadrupedRobotDriver对象实例
  static QrDriver* getInstance();

private:
  QrDriver();

  /*
   * 创建硬件接口的工厂方法
   * 从robot.xml文件中读取配置信息， 完成propagate_和robot_的初始化。
   */
  static QrDriver* instance_;

  void runPropagate();

  // 用于保存每一次写入命令的名称列表
  std::vector<std::string> cmd_vec_;

  bool new_command_;
  bool keepalive_;
  bool connected_;
  std::thread propagate_thread_;
};

} /* namespace quadruped_robot_driver */

#endif /* QUADRUPED_ROBOT_DRIVER_H_ */
