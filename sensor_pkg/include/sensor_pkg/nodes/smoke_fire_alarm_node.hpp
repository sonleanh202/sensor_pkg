#pragma once

#include "sensor_pkg/nodes/generic_modbus_sensor_node.hpp"

namespace sensor_pkg
{

class SmokeFireAlarmNode : public GenericModbusSensorNode
{
public:
  SmokeFireAlarmNode();
};

}  // namespace sensor_pkg
