/*
 * robot_parser.h
 *
 *  Created on: Dec 3, 2016
 *      Author: silence
 */

#ifndef INCLUDE_ROBOT_PARSER_H_
#define INCLUDE_ROBOT_PARSER_H_

#include <string>
#include <map>
#include <tinyxml.h>

namespace qr_driver {

class QrDriver;
class RobotStateBase;
class PropagateImpBase;

class RobotParser {
public:
  typedef RobotStateBase* (*ParserFunPtr)(TiXmlElement*, PropagateImpBase*);
  typedef std::map<std::string, ParserFunPtr> ParserMap;
  static ParserMap hw_parser_map_;
  static TiXmlElement* xml_root_;

  static bool parserFromFile(const std::string&, QrDriver*);
  static bool parserFromParam(const std::string&, QrDriver*);

private:
  RobotParser();
  ~RobotParser();

  static bool init(const std::string&);
  static bool initParam(const std::string&);
  static bool parserRobotStateAndPropagate(QrDriver* robot);
  static bool parserJointStates(QrDriver* robot);

  static PropagateImpBase* parserPropagate(TiXmlElement* root);
  /**
   * 在解析RobotState的过程中， 将状态句柄填入通讯接口中。
   */
  static RobotStateBase* parserRobotState(TiXmlElement* root, PropagateImpBase*);
  static RobotStateBase* getActuator(TiXmlElement* root, PropagateImpBase*);
  static RobotStateBase* getEncoder(TiXmlElement* root, PropagateImpBase*);
};

} /* namespace qr_driver */

#endif /* INCLUDE_ROBOT_PARSER_H_ */
