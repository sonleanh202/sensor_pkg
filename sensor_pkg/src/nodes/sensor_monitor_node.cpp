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

  this->declare_parameter<std::string>("ch4_topic", "/sensors/ch4");
  this->declare_parameter<std::string>("co2_topic", "/sensors/co2");
  this->declare_parameter<std::string>("temperature_topic", "/sensors/temperature");
  this->declare_parameter<std::string>("humidity_topic", "/sensors/humidity");
  this->declare_parameter<std::string>("h2s_topic", "/sensors/h2s");
  this->declare_parameter<std::string>("co_topic", "/sensors/co");
  this->declare_parameter<std::string>("o2_topic", "/sensors/o2");
  this->declare_parameter<std::string>("smoke_topic", "/sensors/smoke_fire_alarm");

  this->declare_parameter<double>("ch4_warn", 10.0);
  this->declare_parameter<double>("ch4_alarm", 20.0);
  this->declare_parameter<double>("co2_warn", 1000.0);
  this->declare_parameter<double>("co2_alarm", 2000.0);
  this->declare_parameter<double>("h2s_warn", 5.0);
  this->declare_parameter<double>("h2s_alarm", 10.0);
  this->declare_parameter<double>("co_warn", 35.0);
  this->declare_parameter<double>("co_alarm", 100.0);
  this->declare_parameter<double>("o2_warn_low", 20.0);
  this->declare_parameter<double>("o2_alarm_low", 19.5);
  this->declare_parameter<double>("humidity_warn", 80.0);
  this->declare_parameter<double>("humidity_alarm", 90.0);

  summary_interval_ms_ = this->get_parameter("summary_interval_ms").as_int();

  ch4_topic_ = this->get_parameter("ch4_topic").as_string();
  co2_topic_ = this->get_parameter("co2_topic").as_string();
  temperature_topic_ = this->get_parameter("temperature_topic").as_string();
  humidity_topic_ = this->get_parameter("humidity_topic").as_string();
  h2s_topic_ = this->get_parameter("h2s_topic").as_string();
  co_topic_ = this->get_parameter("co_topic").as_string();
  o2_topic_ = this->get_parameter("o2_topic").as_string();
  smoke_topic_ = this->get_parameter("smoke_topic").as_string();

  ch4_warn_ = this->get_parameter("ch4_warn").as_double();
  ch4_alarm_ = this->get_parameter("ch4_alarm").as_double();
  co2_warn_ = this->get_parameter("co2_warn").as_double();
  co2_alarm_ = this->get_parameter("co2_alarm").as_double();
  h2s_warn_ = this->get_parameter("h2s_warn").as_double();
  h2s_alarm_ = this->get_parameter("h2s_alarm").as_double();
  co_warn_ = this->get_parameter("co_warn").as_double();
  co_alarm_ = this->get_parameter("co_alarm").as_double();
  o2_warn_low_ = this->get_parameter("o2_warn_low").as_double();
  o2_alarm_low_ = this->get_parameter("o2_alarm_low").as_double();
  humidity_warn_ = this->get_parameter("humidity_warn").as_double();
  humidity_alarm_ = this->get_parameter("humidity_alarm").as_double();

  ch4_sub_ = this->create_subscription<SensorMsg>(
    ch4_topic_, 10,
    [this](const SensorMsg::SharedPtr msg) { ch4_callback(msg); });

  co2_sub_ = this->create_subscription<SensorMsg>(
    co2_topic_, 10,
    [this](const SensorMsg::SharedPtr msg) { co2_callback(msg); });

  temperature_sub_ = this->create_subscription<SensorMsg>(
    temperature_topic_, 10,
    [this](const SensorMsg::SharedPtr msg) { temperature_callback(msg); });

  humidity_sub_ = this->create_subscription<SensorMsg>(
    humidity_topic_, 10,
    [this](const SensorMsg::SharedPtr msg) { humidity_callback(msg); });

  h2s_sub_ = this->create_subscription<SensorMsg>(
    h2s_topic_, 10,
    [this](const SensorMsg::SharedPtr msg) { h2s_callback(msg); });

  co_sub_ = this->create_subscription<SensorMsg>(
    co_topic_, 10,
    [this](const SensorMsg::SharedPtr msg) { co_callback(msg); });

  o2_sub_ = this->create_subscription<SensorMsg>(
    o2_topic_, 10,
    [this](const SensorMsg::SharedPtr msg) { o2_callback(msg); });

  smoke_sub_ = this->create_subscription<SensorMsg>(
    smoke_topic_, 10,
    [this](const SensorMsg::SharedPtr msg) { smoke_callback(msg); });

  summary_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(summary_interval_ms_),
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
  snapshot.alarm = msg->alarm;
}

void SensorMonitorNode::ch4_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(ch4_, msg);
}

void SensorMonitorNode::co2_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(co2_, msg);
}

void SensorMonitorNode::temperature_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(temperature_, msg);
}

void SensorMonitorNode::humidity_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(humidity_, msg);
}

void SensorMonitorNode::h2s_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(h2s_, msg);
}

void SensorMonitorNode::co_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(co_, msg);
}

void SensorMonitorNode::o2_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(o2_, msg);
}

void SensorMonitorNode::smoke_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(smoke_, msg);

  if (smoke_.available && (smoke_.raw_value != 0 || smoke_.alarm)) {
    RCLCPP_WARN(
      this->get_logger(),
      "Smoke Alarm: %s",
      smoke_status().c_str());
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
  oss << std::fixed << std::setprecision(integer_like ? 0 : 1) << snapshot.value;
  oss << " " << snapshot.unit;
  return oss.str();
}

std::string SensorMonitorNode::evaluate_high_threshold_status(
  const SensorSnapshot & snapshot,
  double warn_threshold,
  double alarm_threshold) const
{
  if (!snapshot.available) {
    return "N/A";
  }

  if (snapshot.value >= alarm_threshold) {
    return "ALARM";
  }
  if (snapshot.value >= warn_threshold) {
    return "WARN";
  }
  return "OK";
}

std::string SensorMonitorNode::evaluate_low_threshold_status(
  const SensorSnapshot & snapshot,
  double warn_threshold,
  double alarm_threshold) const
{
  if (!snapshot.available) {
    return "N/A";
  }

  if (snapshot.value <= alarm_threshold) {
    return "ALARM";
  }
  if (snapshot.value <= warn_threshold) {
    return "WARN";
  }
  return "OK";
}

std::string SensorMonitorNode::smoke_status() const
{
  if (!smoke_.available) {
    return "N/A";
  }

  if (smoke_.raw_value != 0 || smoke_.alarm) {
    return "WARN | Fire detected";
  }

  return "OK | No fire";
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
      oss << "Humidity: " << format_value(humidity_)
          << " | "
          << evaluate_high_threshold_status(
               humidity_, humidity_warn_, humidity_alarm_);
    }

    lines.emplace_back(oss.str());
  }

  if (ch4_.available) {
    lines.emplace_back(
      "CH4 Concentration: " +
      format_value(ch4_) + " | " +
      evaluate_high_threshold_status(ch4_, ch4_warn_, ch4_alarm_));
  }

  if (co2_.available) {
    lines.emplace_back(
      "CO2 Concentration: " +
      format_value(co2_) + " | " +
      evaluate_high_threshold_status(co2_, co2_warn_, co2_alarm_));
  }

  if (h2s_.available) {
    lines.emplace_back(
      "H2S Concentration: " +
      format_value(h2s_) + " | " +
      evaluate_high_threshold_status(h2s_, h2s_warn_, h2s_alarm_));
  }

  if (co_.available) {
    lines.emplace_back(
      "CO Concentration: " +
      format_value(co_) + " | " +
      evaluate_high_threshold_status(co_, co_warn_, co_alarm_));
  }

  if (o2_.available) {
    lines.emplace_back(
      "O2 Concentration: " +
      format_value(o2_) + " | " +
      evaluate_low_threshold_status(o2_, o2_warn_low_, o2_alarm_low_));
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

}  // namespace sensor_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_pkg::SensorMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}