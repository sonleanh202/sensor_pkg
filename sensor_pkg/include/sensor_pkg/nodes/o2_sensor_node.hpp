#pragma once

#include "sensor_pkg/nodes/generic_modbus_sensor_node.hpp"

namespace sensor_pkg
{

class O2SensorNode : public GenericModbusSensorNode
{
public:
  O2SensorNode();
};

}  // namespace sensor_pkg
