#include "sensors_reader/nodes/sensor_monitor_node.hpp"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace sensors_reader
{

SensorMonitorNode::SensorMonitorNode()
: Node("sensor_monitor_node")
{
  summary_interval_ms_ = this->declare_parameter<int>("summary_interval_ms", 5000);
  stale_timeout_ms_ = this->declare_parameter<int>("stale_timeout_ms", summary_interval_ms_);

  const auto ch4_topic = this->declare_parameter<std::string>("ch4_topic", "/sensor/ch4");
  const auto co2_topic = this->declare_parameter<std::string>("co2_topic", "/sensor/co2");
  const auto temperature_topic =
    this->declare_parameter<std::string>("temperature_topic", "/sensor/temp");
  const auto humidity_topic =
    this->declare_parameter<std::string>("humidity_topic", "/sensor/humidity");
  const auto h2s_topic = this->declare_parameter<std::string>("h2s_topic", "/sensor/h2s");
  const auto co_topic = this->declare_parameter<std::string>("co_topic", "/sensor/co");
  const auto o2_topic = this->declare_parameter<std::string>("o2_topic", "/sensor/o2");
  const auto smoke_topic =
    this->declare_parameter<std::string>("smoke_topic", "/sensor/smoke_status");

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
    std::chrono::milliseconds(summary_interval_ms_),
    [this]() { print_summary(); });
}

void SensorMonitorNode::update_snapshot(
  SensorSnapshot & snapshot,
  const SensorMsg::SharedPtr msg)
{
  snapshot.available = true;
  snapshot.sensor_name = msg->sensor_name;
  snapshot.quantity = msg->quantity;
  snapshot.value = msg->value;
  snapshot.raw_value = msg->raw_value;
  snapshot.unit = msg->unit;
  snapshot.alarm = msg->alarm;
  snapshot.last_update_ns = this->now().nanoseconds();
}

void SensorMonitorNode::smoke_callback(const SensorMsg::SharedPtr msg)
{
  update_snapshot(smoke_, msg);

  if (smoke_.raw_value == 1 || smoke_.alarm) {
    RCLCPP_WARN(this->get_logger(), "Smoke Status: Alarm");
  }
}

bool SensorMonitorNode::is_fresh(const SensorSnapshot & snapshot) const
{
  if (!snapshot.available || snapshot.last_update_ns == 0) {
    return false;
  }

  const int64_t age_ns = this->now().nanoseconds() - snapshot.last_update_ns;
  return age_ns <= static_cast<int64_t>(stale_timeout_ms_) * 1000000LL;
}

std::string SensorMonitorNode::format_value(const SensorSnapshot & snapshot) const
{
  int precision = 1;

  if (snapshot.sensor_name == "H2S" || snapshot.quantity == "hydrogen_sulfide") {
    precision = 2;
  } else if (
    snapshot.sensor_name == "CO" ||
    snapshot.sensor_name == "CO2" ||
    snapshot.unit == "%LEL" ||
    snapshot.unit == "state")
  {
    precision = 0;
  }

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision)
      << snapshot.value << " " << snapshot.unit;
  return oss.str();
}

std::string SensorMonitorNode::smoke_status() const
{
  if (!is_fresh(smoke_)) {
    return "N/A";
  }

  if (smoke_.raw_value == 1 || smoke_.alarm) {
    return "Alarm";
  }

  return "Normal";
}

void SensorMonitorNode::print_summary()
{
  std::vector<std::string> lines;

  const bool temp_fresh = is_fresh(temperature_);
  const bool hum_fresh = is_fresh(humidity_);

  if (temp_fresh || hum_fresh) {
    std::ostringstream oss;

    if (temp_fresh) {
      oss << "Temperature: " << format_value(temperature_);
    }

    if (hum_fresh) {
      if (temp_fresh) {
        oss << " | ";
      }
      oss << "Humidity: " << format_value(humidity_);
    }

    lines.emplace_back(oss.str());
  }

  if (is_fresh(ch4_)) {
    lines.emplace_back("CH4: " + format_value(ch4_));
  }

  if (is_fresh(co2_)) {
    lines.emplace_back("CO2: " + format_value(co2_));
  }

  if (is_fresh(co_)) {
    lines.emplace_back("CO: " + format_value(co_));
  }

  if (is_fresh(o2_)) {
    lines.emplace_back("O2: " + format_value(o2_));
  }

  if (is_fresh(h2s_)) {
    lines.emplace_back("H2S: " + format_value(h2s_));
  }

  if (is_fresh(smoke_)) {
    lines.emplace_back("Smoke Status: " + smoke_status());
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

}  // namespace sensors_reader

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensors_reader::SensorMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
