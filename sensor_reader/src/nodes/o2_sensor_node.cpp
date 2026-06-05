#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_reader/nodes/o2_sensor_node.hpp"

namespace sensor_reader
{

namespace
{
SensorDefaults make_o2_defaults()
{
  SensorDefaults defaults;
  defaults.sensor_name = "o2";
  defaults.sensor_model = "ES-O2-01";
  defaults.quantity = "oxygen";
  defaults.unit = "%VOL";
  defaults.topic_name = "/sensors/o2";
  defaults.read_register = 2;
  defaults.register_count = 1;
  defaults.scale = 0.1;
  defaults.offset = 0.0;
  defaults.signed_value = false;
  defaults.enabled = true;
  defaults.notes = "Read O2 from Modbus holding register 0x0002; raw unit is 0.1 %VOL.";
  return defaults;
}
}  // namespace

O2SensorNode::O2SensorNode()
: GenericModbusSensorNode("o2_sensor_node", make_o2_defaults())
{
}

}  // namespace sensor_reader

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_reader::O2SensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
