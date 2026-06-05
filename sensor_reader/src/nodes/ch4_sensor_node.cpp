#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_reader/nodes/ch4_sensor_node.hpp"

namespace sensor_reader
{

namespace
{
SensorDefaults make_ch4_defaults()
{
  SensorDefaults defaults;
  defaults.sensor_name = "ch4";
  defaults.sensor_model = "ES-CH4-01";
  defaults.quantity = "methane";
  defaults.unit = "%LEL";
  defaults.topic_name = "/sensors/ch4";
  defaults.read_register = 2;
  defaults.register_count = 1;
  defaults.scale = 1.0;
  defaults.offset = 0.0;
  defaults.signed_value = false;
  defaults.enabled = true;
  defaults.notes = "Read CH4 from Modbus holding register 0x0002.";
  return defaults;
}
}  // namespace

Ch4SensorNode::Ch4SensorNode()
: GenericModbusSensorNode("ch4_sensor_node", make_ch4_defaults())
{
}

}  // namespace sensor_reader

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_reader::Ch4SensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
