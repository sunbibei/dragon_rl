/*
 * qr_driver.h
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#ifndef QUADRUPED_ROBOT_DRIVER_H_
#define QUADRUPED_ROBOT_DRIVER_H_

#include <thread>

namespace qr_driver {

class HWCommandBase;
class RobotStateBase;
class PropagateImpBase;

class QrDriver {
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

public:
  ~QrDriver();
  // 获取QuadrupedRobotDriver对象实例
  static QrDriver* getInstance();

  typedef boost::shared_ptr<PropagateImpBase> PropagateSharedPtr;
  typedef boost::shared_ptr<RobotStateBase> RobotStateSharedPtr;
  PropagateSharedPtr  propagate_;
  RobotStateSharedPtr robot_;

public:
  // 初始化所有变量, 以及线程等.
  bool init(const std::string& xml = "robot.xml");
  // 开始运行
  bool start();
  // 停止运行
  void halt();
  // 设定命令, 并传递给机器人
  void addCommand(HWCommandBase&);
};

} /* namespace quadruped_robot_driver */

#endif /* QUADRUPED_ROBOT_DRIVER_H_ */
