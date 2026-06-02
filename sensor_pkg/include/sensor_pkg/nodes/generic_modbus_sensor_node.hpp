#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_pkg/common/sensor_defaults.hpp"
#include "interfaces/msg/sensor.hpp"

namespace sensor_pkg
{

class GenericModbusSensorNode : public rclcpp::Node
{
public:
  GenericModbusSensorNode(const std::string & node_name, const SensorDefaults & defaults);

protected:
  int32_t decode_raw_value(uint16_t raw_register) const;

private:
  std::string format_thresholds() const;
  std::string evaluate_status(double value, int32_t raw_value) const;

  void timer_callback();

  rclcpp::Publisher<interfaces::msg::Sensor>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string port_;
  std::string topic_name_;
  std::string sensor_name_;
  std::string sensor_model_;
  std::string quantity_;
  std::string unit_;
  std::string notes_;
  std::string bus_lock_file_;

  int baudrate_{4800};
  int data_bits_{8};
  int stop_bits_{1};
  int slave_id_{1};
  int read_register_{0};
  int register_count_{1};
  int response_timeout_ms_{1000};
  int poll_interval_ms_{5000};
  double scale_{1.0};
  double offset_{0.0};
  bool signed_value_{false};
  bool alarm_when_nonzero_{false};
  double warning_threshold_{-1.0};
  double alarm_threshold_{-1.0};
  bool enabled_{true};
  char parity_{'N'};
};

}  // namespace sensor_pkg
