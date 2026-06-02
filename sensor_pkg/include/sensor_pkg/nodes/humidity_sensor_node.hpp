#pragma once

#include "sensor_pkg/nodes/generic_modbus_sensor_node.hpp"

namespace sensor_pkg
{

class HumiditySensorNode : public GenericModbusSensorNode
{
public:
  HumiditySensorNode();
};

}  // namespace sensor_pkg
