#pragma once

#include "sensors_reader/nodes/generic_modbus_sensor_node.hpp"

namespace sensors_reader
{

class Co2SensorNode : public GenericModbusSensorNode
{
public:
  Co2SensorNode();
};

}  // namespace sensors_reader
