#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensors_reader/nodes/co_sensor_node.hpp"

namespace sensors_reader
{

namespace
{
SensorDefaults make_co_defaults()
{
  SensorDefaults defaults;
  defaults.sensor_name = "CO";
  defaults.sensor_model = "ES-CO-01";
  defaults.quantity = "carbon_monoxide";
  defaults.unit = "ppm";
  defaults.topic_name = "/sensor/co";

  // CO reads two registers: 5 and 6
  defaults.read_register = 5;
  defaults.register_count = 2;

  defaults.scale = 1.0;
  defaults.offset = 0.0;
  defaults.signed_value = false;
  defaults.alarm_when_nonzero = false;
  defaults.warning_threshold = 35.0;
  defaults.alarm_threshold = 100.0;
  defaults.enabled = true;
  defaults.notes = "Read CO from two Modbus registers: 5 and 6.";
  return defaults;
}
}  // namespace

CoSensorNode::CoSensorNode()
: GenericModbusSensorNode("co_sensor_node", make_co_defaults())
{
}

}  // namespace sensors_reader

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensors_reader::CoSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}