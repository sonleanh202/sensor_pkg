#pragma once

#include "sensor_reader/nodes/generic_modbus_sensor_node.hpp"

namespace sensor_reader
{

class Co2SensorNode : public GenericModbusSensorNode
{
public:
  Co2SensorNode();
};

} 