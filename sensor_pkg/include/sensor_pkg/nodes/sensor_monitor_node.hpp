#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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
};

class SensorMonitorNode : public rclcpp::Node
{
public:
  SensorMonitorNode();

private:
  using SensorMsg = interfaces::msg::Sensor;

  void update_snapshot(SensorSnapshot & snapshot, const SensorMsg::SharedPtr msg);

  void ch4_callback(const SensorMsg::SharedPtr msg);
  void co2_callback(const SensorMsg::SharedPtr msg);
  void temperature_callback(const SensorMsg::SharedPtr msg);
  void humidity_callback(const SensorMsg::SharedPtr msg);
  void h2s_callback(const SensorMsg::SharedPtr msg);
  void co_callback(const SensorMsg::SharedPtr msg);
  void o2_callback(const SensorMsg::SharedPtr msg);
  void smoke_callback(const SensorMsg::SharedPtr msg);

  void print_summary();

  std::string format_value(const SensorSnapshot & snapshot) const;

  std::string evaluate_high_threshold_status(
    const SensorSnapshot & snapshot,
    double warn_threshold,
    double alarm_threshold) const;

  std::string evaluate_low_threshold_status(
    const SensorSnapshot & snapshot,
    double warn_threshold,
    double alarm_threshold) const;

  std::string smoke_status() const;

  int summary_interval_ms_;

  std::string ch4_topic_;
  std::string co2_topic_;
  std::string temperature_topic_;
  std::string humidity_topic_;
  std::string h2s_topic_;
  std::string co_topic_;
  std::string o2_topic_;
  std::string smoke_topic_;

  double ch4_warn_;
  double ch4_alarm_;
  double co2_warn_;
  double co2_alarm_;
  double h2s_warn_;
  double h2s_alarm_;
  double co_warn_;
  double co_alarm_;
  double o2_warn_low_;
  double o2_alarm_low_;
  double humidity_warn_;
  double humidity_alarm_;

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