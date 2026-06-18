#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/sensor.hpp"

namespace sensors_reader
{

class HumiditySensorNode : public rclcpp::Node
{
public:
  HumiditySensorNode();

private:
  void timer_callback();
  int32_t decode_signed(uint16_t raw) const;
  std::string evaluate_humidity_status(double humidity) const;

  std::string port_;
  int baudrate_;
  char parity_;
  int data_bits_;
  int stop_bits_;
  int slave_id_;
  int response_timeout_ms_;
  int poll_interval_ms_;
  std::string bus_lock_file_;
  bool enabled_;

  int humidity_register_;
  int temperature_register_;
  double humidity_scale_;
  double temperature_scale_;
  double temperature_offset_;
  bool temperature_signed_;

  double humidity_warning_threshold_;
  double humidity_alarm_threshold_;

  std::string sensor_model_;
  std::string humidity_sensor_name_;
  std::string temperature_sensor_name_;
  std::string humidity_topic_name_;
  std::string temperature_topic_name_;
  std::string humidity_unit_;
  std::string temperature_unit_;
  std::string humidity_notes_;
  std::string temperature_notes_;

  rclcpp::Publisher<interfaces::msg::Sensor>::SharedPtr humidity_publisher_;
  rclcpp::Publisher<interfaces::msg::Sensor>::SharedPtr temperature_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sensors_reader