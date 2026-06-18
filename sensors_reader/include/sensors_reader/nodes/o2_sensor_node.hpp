#pragma once

#include "sensors_reader/nodes/generic_modbus_sensor_node.hpp"

namespace sensors_reader
{

class O2SensorNode : public GenericModbusSensorNode
{
public:
  O2SensorNode();
};

}  // namespace sensors_reader