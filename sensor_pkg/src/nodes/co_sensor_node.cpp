#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_pkg/nodes/co_sensor_node.hpp"

namespace sensor_pkg
{

namespace
{
SensorDefaults make_co_defaults()
{
  SensorDefaults defaults;
  defaults.sensor_name = "co";
  defaults.sensor_model = "ES-CO-01";
  defaults.quantity = "carbon_monoxide";
  defaults.unit = "ppm";
  defaults.topic_name = "/sensors/co";
  defaults.read_register = 2;
  defaults.register_count = 1;
  defaults.scale = 1.0;
  defaults.offset = 0.0;
  defaults.signed_value = false;
  defaults.alarm_when_nonzero = false;
  defaults.warning_threshold = 35.0;
  defaults.alarm_threshold = 100.0;
  defaults.enabled = true;
  defaults.notes = "Read CO from Modbus holding register 0x0002.";
  return defaults;
}
}  // namespace

CoSensorNode::CoSensorNode()
: GenericModbusSensorNode("co_sensor_node", make_co_defaults())
{
}

}  // namespace sensor_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_pkg::CoSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
