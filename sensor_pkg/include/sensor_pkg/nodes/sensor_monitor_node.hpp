#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/sensor.hpp"

namespace sensor_pkg
{

struct SensorSnapshot
{
  bool available{false};
  double value{0.0};
  int32_t raw_value{0};
  std::string unit;
  bool alarm{false};
  int64_t last_update_ns{0};
};

class SensorMonitorNode : public rclcpp::Node
{
public:
  SensorMonitorNode();

private:
  using SensorMsg = interfaces::msg::Sensor;

  void update_snapshot(SensorSnapshot & snapshot, const SensorMsg::SharedPtr msg);
  void smoke_callback(const SensorMsg::SharedPtr msg);
  void print_summary();

  bool is_fresh(const SensorSnapshot & snapshot) const;
  std::string format_value(const SensorSnapshot & snapshot) const;
  std::string smoke_status() const;

  int summary_interval_ms_;
  int stale_timeout_ms_;

  SensorSnapshot ch4_;
  SensorSnapshot co2_;
  SensorSnapshot temperature_;
  SensorSnapshot humidity_;
  SensorSnapshot h2s_;
  SensorSnapshot co_;
  SensorSnapshot o2_;
  SensorSnapshot smoke_;

  rclcpp::Subscription<SensorMsg>::SharedPtr ch4_sub_;
  rclcpp::Subscription<SensorMsg>::SharedPtr co2_sub_;
  rclcpp::Subscription<SensorMsg>::SharedPtr temperature_sub_;
  rclcpp::Subscription<SensorMsg>::SharedPtr humidity_sub_;
  rclcpp::Subscription<SensorMsg>::SharedPtr h2s_sub_;
  rclcpp::Subscription<SensorMsg>::SharedPtr co_sub_;
  rclcpp::Subscription<SensorMsg>::SharedPtr o2_sub_;
  rclcpp::Subscription<SensorMsg>::SharedPtr smoke_sub_;

  rclcpp::TimerBase::SharedPtr summary_timer_;
};

}  // namespace sensor_pkg