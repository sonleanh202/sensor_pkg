#pragma once

#include "sensor_reader/nodes/generic_modbus_sensor_node.hpp"

namespace sensor_reader
{

class SmokeFireAlarmNode : public GenericModbusSensorNode
{
public:
  SmokeFireAlarmNode();
};

}  
