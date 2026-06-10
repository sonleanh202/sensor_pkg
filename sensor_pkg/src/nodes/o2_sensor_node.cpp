#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_pkg/nodes/o2_sensor_node.hpp"

namespace sensor_pkg
{

namespace
{
SensorDefaults make_o2_defaults()
{
  SensorDefaults defaults;
  defaults.sensor_name = "O2";
  defaults.sensor_model = "ES-O2-01";
  defaults.quantity = "oxygen";
  defaults.unit = "%VOL";
  defaults.topic_name = "/sensors/o2";

  defaults.read_register = 2;
  defaults.register_count = 2;

  defaults.scale = 0.1;
  defaults.offset = 0.0;
  defaults.signed_value = false;
  defaults.alarm_when_nonzero = false;
  defaults.warning_threshold = -1.0;
  defaults.alarm_threshold = 19.5;
  defaults.enabled = true;
  defaults.notes = "Read O2 from two Modbus registers starting at 0x0002; raw unit is 0.1 %VOL.";
  return defaults;
}
}  // namespace

O2SensorNode::O2SensorNode()
: GenericModbusSensorNode("o2_sensor_node", make_o2_defaults())
{
}

}  // namespace sensor_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_pkg::O2SensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}