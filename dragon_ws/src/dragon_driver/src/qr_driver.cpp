/*
 * qr_driver.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#include "hardware_interface/robot_state_base.h"
#include "propagate_interface/propagate_imp_base.h"
#include "robot_parser.h"

#include <thread>
#include <glog/logging.h>
#include <glog/log_severity.h>
#include <qr_driver.h>
#include <tinyxml.h>

namespace qr_driver {

QrDriver* QrDriver::instance_ = nullptr;
QrDriver* QrDriver::getInstance() {
  if (nullptr == instance_) {
    instance_ = new QrDriver;
    LOG(INFO) << "Create the singleton instance: QuadrupedDriver";
  }

  LOG(INFO) << "Return the singleton instance: QuadrupedDriver";
  return instance_;
}

QrDriver::QrDriver()
    : new_command_(false), keepalive_(true), connected_(false) {
  google::InitGoogleLogging("qr_driver");
  google::FlushLogFiles(-1);
  google::LogToStderr();
}

QrDriver::~QrDriver() {
  halt();
}

bool QrDriver::init(const std::string& xml) {
  if (((nullptr == robot_) || (nullptr == propagate_))
      && (!RobotParser::parser(xml, this))) {
    LOG(ERROR) << "The initialization FAIL in the QrDriver";
    return false;
  }

  LOG(INFO) << "The initialization has successful";
  connected_ = propagate_->init();
  return connected_;
}

bool QrDriver::start() {
  propagate_thread_ = std::thread(&QrDriver::runPropagate, this);
  LOG(INFO) << "The propagate thread has started to run!";
  return true;
}

void QrDriver::runPropagate() {
  while (keepalive_) {
    while (connected_ && keepalive_) {
      // Everything is OK!
      connected_ = propagate_->read();
      if (new_command_) {
        connected_ = propagate_->write(cmd_vec_);
        new_command_ = false;
        cmd_vec_.clear();
      }
    }
    if (keepalive_) {
      //reconnect
      LOG(WARNING) << "Disconnected! In order to Keepalive, we try to reconnect... ...";
      int count = 0;
      while (keepalive_ && !connected_) {
        LOG(WARNING) << "Attempt to reconnect (" << count++ << " times)";
        connected_ = propagate_->init();
        if (!connected_) {
          // wait for 500ms
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
      } // end while(keepalive_ && !connected_)
    } // end if (keepalive_)
  } // end while (keepalive_)
}

void QrDriver::halt() {
  keepalive_ = false;
  propagate_thread_.join();

  LOG(INFO) << "QrDrvier halt... ...";
}

void QrDriver::addCommand(HWCommandBase& cmd) {
  cmd_vec_.push_back(cmd.name_);
  robot_->setCommand(cmd);
  new_command_ = true;
}


} /* namespace quadruped_robot_driver */
