#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_reader/nodes/smoke_fire_alarm_node.hpp"

namespace sensor_reader
{

namespace
{
SensorDefaults make_smoke_defaults()
{
  SensorDefaults defaults;
  defaults.sensor_name = "smoke_fire_alarm";
  defaults.sensor_model = "ES-SD-01";
  defaults.quantity = "smoke_alarm_status";
  defaults.unit = "state";
  defaults.topic_name = "/sensors/smoke_fire_alarm";
  defaults.read_register = 3;
  defaults.register_count = 1;
  defaults.scale = 1.0;
  defaults.offset = 0.0;
  defaults.signed_value = false;
  defaults.enabled = true;
  defaults.notes = "Read smoke/fire alarm status from 0x0003; 0=normal, 1=alarm.";
  return defaults;
}
}  

SmokeFireAlarmNode::SmokeFireAlarmNode()
: GenericModbusSensorNode("smoke_fire_alarm_node", make_smoke_defaults())
{
}

} 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_reader::SmokeFireAlarmNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
