#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/sensor.hpp"
#include "sensors_reader/common/sensor_defaults.hpp"

namespace sensors_reader
{

class GenericModbusSensorNode : public rclcpp::Node
{
public:
  GenericModbusSensorNode(
    const std::string & node_name,
    const SensorDefaults & defaults);

private:
  int32_t decode_raw_value(const std::vector<uint16_t> & registers) const;
  std::string evaluate_status(double value, int32_t raw_value) const;
  void timer_callback();

  rclcpp::Publisher<interfaces::msg::Sensor>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string port_;
  int baudrate_;
  char parity_;
  int data_bits_;
  int stop_bits_;
  int slave_id_;
  int read_register_;
  int register_count_;
  double scale_;
  double offset_;
  bool signed_value_;
  bool alarm_when_nonzero_;
  double warning_threshold_;
  double alarm_threshold_;
  bool enabled_;
  int response_timeout_ms_;
  int poll_interval_ms_;
  std::string bus_lock_file_;
  std::string sensor_name_;
  std::string sensor_model_;
  std::string quantity_;
  std::string unit_;
  std::string topic_name_;
  std::string notes_;
};

}  // namespace sensors_reader