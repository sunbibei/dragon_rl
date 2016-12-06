/*
 * propagate_imp_pcan.h
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#ifndef INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_
#define INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_

#include <propagate_interface/propagate_imp_base.h>
#include <hardware_interface/actuator.h>
#include <hardware_interface/encoder.h>

#include <map>

namespace qr_driver {

class PropagateImpPcan: public PropagateImpBase {
public:
  PropagateImpPcan(const std::string& name);
  virtual ~PropagateImpPcan();

  virtual bool init();
  virtual void stop();

  // 完成数据的读写.
  virtual bool write(const std::vector<std::string>&);
  virtual bool read();
};

} /* namespace qr_driver */

#endif /* INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_ */
