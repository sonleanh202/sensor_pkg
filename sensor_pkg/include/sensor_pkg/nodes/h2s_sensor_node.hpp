#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_pkg/drivers/sen0467_client.hpp"
#include "interfaces/msg/sensor.hpp"

namespace sensor_pkg
{

class H2S_SensorNode : public rclcpp::Node
{
public:
  H2S_SensorNode();

private:
  void timer_callback();
  std::string evaluate_status(int ppm) const;

  std::string port_;
  int baudrate_;
  int address_;
  int response_timeout_ms_;
  int poll_interval_ms_;
  std::string bus_lock_file_;
  std::string sensor_name_;
  std::string sensor_model_;
  std::string quantity_;
  std::string unit_;
  std::string topic_name_;
  std::string notes_;
  double warning_threshold_;
  double alarm_threshold_;
  bool enabled_;

  bool passive_mode_configured_;

  rclcpp::Publisher<interfaces::msg::Sensor>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<Sen0467UartToRs485Client> client_;
};

}  // namespace sensor_pkg