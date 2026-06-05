#include "sensor_pkg/nodes/sensor_monitor_node.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace sensor_pkg
{

SensorMonitorNode::SensorMonitorNode()
: Node("sensor_monitor_node")
{
  this->declare_parameter<int>("summary_interval_ms", 5000);

  const auto ch4_topic = this->declare_parameter<std::string>("ch4_topic", "/sensors/ch4");
  const auto co2_topic = this->declare_parameter<std::string>("co2_topic", "/sensors/co2");
  const auto temperature_topic =
    this->declare_parameter<std::string>("temperature_topic", "/sensors/temperature");
  const auto humidity_topic =
    this->declare_parameter<std::string>("humidity_topic", "/sensors/humidity");
  const auto h2s_topic = this->declare_parameter<std::string>("h2s_topic", "/sensors/h2s");
  const auto co_topic = this->declare_parameter<std::string>("co_topic", "/sensors/co");
  const auto o2_topic = this->declare_parameter<std::string>("o2_topic", "/sensors/o2");
  const auto smoke_topic =
    this->declare_parameter<std::string>("smoke_topic", "/sensors/smoke_fire_alarm");

  const int summary_interval_ms = this->get_parameter("summary_interval_ms").as_int();

  ch4_sub_ = this->create_subscription<SensorMsg>(
    ch4_topic, 10,
    [this](const SensorMsg::SharedPtr msg) { update_snapshot(ch4_, msg); });

  co2_sub_ = this->create_subscription<SensorMsg>(
    co2_topic, 10,
    [this](const SensorMsg::SharedPtr msg) { update_snapshot(co2_, msg); });

  temperature_sub_ = this->create_subscription<SensorMsg>(
    temperature_topic, 10,
    [this](const SensorMsg::SharedPtr msg) { update_snapshot(temperature_, msg); });

  humidity_sub_ = this->create_subscription<SensorMsg>(
    humidity_topic, 10,
    [this](const SensorMsg::SharedPtr msg) { update_snapshot(humidity_, msg); });

  h2s_sub_ = this->create_subscription<SensorMsg>(
    h2s_topic, 10,
    [this](const SensorMsg::SharedPtr msg) { update_snapshot(h2s_, msg); });

  co_sub_ = this->create_subscription<SensorMsg>(
    co_topic, 10,
    [this](const SensorMsg::SharedPtr msg) { update_snapshot(co_, msg); });

  o2_sub_ = this->create_subscription<SensorMsg>(
    o2_topic, 10,
    [this](const SensorMsg::SharedPtr msg) { update_snapshot(o2_, msg); });

  smoke_sub_ = this->create_subscription<SensorMsg>(
    smoke_topic, 10,
    [this](const SensorMsg::SharedPtr msg) { smoke_callback(msg); });

  summary_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(summary_interval_ms),
    [this]() { print_summary(); });
}

void SensorMonitorNode::update_snapshot(
  SensorSnapshot & snapshot,
  const SensorMsg::SharedPtr msg)
{
  snapshot.available = true;
  snapshot.value = msg->value;
  snapshot.raw_value = msg->raw_value;
  snapshot.unit = msg->unit;
}

void SensorMonitorNode::smoke_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(smoke_, msg);

  // Nếu rơi vào cảnh báo thì báo liên tục
  if (smoke_.available && smoke_.raw_value != 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Smoke Alarm: WARN");
  }
}

std::string SensorMonitorNode::format_value(const SensorSnapshot & snapshot) const
{
  if (!snapshot.available) {
    return "N/A";
  }

  const bool integer_like =
    snapshot.unit == "ppm" ||
    snapshot.unit == "%LEL" ||
    snapshot.unit == "state";

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(integer_like ? 0 : 1)
      << snapshot.value << " " << snapshot.unit;
  return oss.str();
}

std::string SensorMonitorNode::smoke_status() const
{
  if (!smoke_.available) {
    return "N/A";
  }

  if (smoke_.raw_value != 0) {
    return "WARN";
  }

  return "OK";
}

void SensorMonitorNode::print_summary()
{
  std::vector<std::string> lines;

  if (temperature_.available || humidity_.available) {
    std::ostringstream oss;

    if (temperature_.available) {
      oss << "Temperature: " << format_value(temperature_);
    }

    if (humidity_.available) {
      if (temperature_.available) {
        oss << " | ";
      }
      oss << "Humidity: " << format_value(humidity_);
    }

    lines.emplace_back(oss.str());
  }

  if (ch4_.available) {
    lines.emplace_back("CH4: " + format_value(ch4_));
  }

  if (co2_.available) {
    lines.emplace_back("CO2: " + format_value(co2_));
  }

  if (h2s_.available) {
    lines.emplace_back("H2S: " + format_value(h2s_));
  }

  if (co_.available) {
    lines.emplace_back("CO: " + format_value(co_));
  }

  if (o2_.available) {
    lines.emplace_back("O2: " + format_value(o2_));
  }

  if (smoke_.available) {
    lines.emplace_back("Smoke Alarm: " + smoke_status());
  }

  if (lines.empty()) {
    return;
  }

  std::ostringstream oss;
  oss << "\n";
  for (size_t i = 0; i < lines.size(); ++i) {
    oss << lines[i];
    if (i + 1 < lines.size()) {
      oss << "\n";
    }
  }

  RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_pkg::SensorMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}