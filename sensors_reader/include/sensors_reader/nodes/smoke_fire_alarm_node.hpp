#pragma once

#include "sensors_reader/nodes/generic_modbus_sensor_node.hpp"

namespace sensors_reader
{

class SmokeFireAlarmNode : public GenericModbusSensorNode
{
public:
  SmokeFireAlarmNode();
};

}  // namespace sensors_reader
