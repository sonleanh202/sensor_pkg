#pragma once

#include "sensor_pkg/nodes/generic_modbus_sensor_node.hpp"

namespace sensor_pkg
{

class Co2SensorNode : public GenericModbusSensorNode
{
public:
  Co2SensorNode();
};

}  // namespace sensor_pkg
