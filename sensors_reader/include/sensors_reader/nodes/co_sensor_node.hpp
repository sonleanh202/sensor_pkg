#pragma once

#include "sensors_reader/nodes/generic_modbus_sensor_node.hpp"

namespace sensors_reader
{

class CoSensorNode : public GenericModbusSensorNode
{
public:
  CoSensorNode();
};

}  // namespace sensors_reader
