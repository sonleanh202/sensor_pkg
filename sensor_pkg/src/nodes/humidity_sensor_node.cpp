#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_pkg/nodes/humidity_sensor_node.hpp"

namespace sensor_pkg
{

namespace
{
SensorDefaults make_humidity_defaults()
{
  SensorDefaults defaults;
  defaults.sensor_name = "humidity";
  defaults.sensor_model = "RT WS N01 QT";
  defaults.quantity = "relative_humidity";
  defaults.unit = "%RH";
  defaults.topic_name = "/sensors/humidity";
  defaults.read_register = 0;
  defaults.register_count = 1;
  defaults.scale = 0.1;
  defaults.offset = 0.0;
  defaults.signed_value = false;
  defaults.alarm_when_nonzero = false;
  defaults.warning_threshold = 80.0;
  defaults.alarm_threshold = 90.0;
  defaults.enabled = true;
  defaults.notes = "Assumed compatible with RS-WS-N01 family: humidity register 0x0000, scale 0.1 %RH.";
  return defaults;
}
}  // namespace

HumiditySensorNode::HumiditySensorNode()
: GenericModbusSensorNode("humidity_sensor_node", make_humidity_defaults())
{
}

}  // namespace sensor_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_pkg::HumiditySensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
