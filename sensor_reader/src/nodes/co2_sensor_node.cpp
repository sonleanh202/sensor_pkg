#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_reader/nodes/co2_sensor_node.hpp"

namespace sensor_reader
{

namespace
{
SensorDefaults make_co2_defaults()
{
  SensorDefaults defaults;
  defaults.sensor_name = "co2";
  defaults.sensor_model = "ES-CO2-01";
  defaults.quantity = "carbon_dioxide";
  defaults.unit = "ppm";
  defaults.topic_name = "/sensors/co2";
  defaults.read_register = 2;
  defaults.register_count = 1;
  defaults.scale = 1.0;
  defaults.offset = 0.0;
  defaults.signed_value = false;
  defaults.enabled = true;
  defaults.notes = "Read CO2 from Modbus holding register 0x0002.";
  return defaults;
}
}  // namespace

Co2SensorNode::Co2SensorNode()
: GenericModbusSensorNode("co2_sensor_node", make_co2_defaults())
{
}

}  // namespace sensor_reader

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_reader::Co2SensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
