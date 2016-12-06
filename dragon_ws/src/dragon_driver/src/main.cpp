/*
 * test_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#include <propagate_interface/propagate_imp_base.h>
#include "qr_driver.h"

#include "hardware_interface/robot_state_base.h"
#include "hardware_interface/actuator.h"
#include "hardware_interface/encoder.h"
#include "robot_parser.h"

#include "qr_ros_wrapper.h"

#include <iostream>

int main(int argc, char* argv[]) {
  bool use_sim_time = false;

  ros::init(argc, argv, "qr_driver");
  ros::NodeHandle nh;
  if (ros::param::get("use_sim_time", use_sim_time)) {
    LOG(INFO) << ("use_sim_time is set!!");
  }

  qr_driver::QrRosWrapper* interface = qr_driver::QrRosWrapper::getInstance();
  if (nullptr == interface) {
    LOG(FATAL) << "Can't get the instance of QrRosWrapper!";
    return -1;
  }
  interface->start();

  ros::AsyncSpinner spinner(3);
  spinner.start();

  ros::waitForShutdown();

  interface->halt();
  delete interface;
  interface = nullptr;

  return 0;
}
