#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_pkg/common/bus_lock.hpp"
#include "sensor_pkg/nodes/h2s_sensor_node.hpp"

namespace sensor_pkg
{

H2S_SensorNode::H2S_SensorNode()
: Node("h2s_sensor_node"),
  baudrate_(9600),
  address_(1),
  response_timeout_ms_(1000),
  poll_interval_ms_(5000),
  warning_threshold_(5.0),
  alarm_threshold_(10.0),
  enabled_(true),
  passive_mode_configured_(false)
{
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 9600);
  this->declare_parameter<int>("address", 1);
  this->declare_parameter<int>("response_timeout_ms", 1000);
  this->declare_parameter<int>("poll_interval_ms", 5000);
  this->declare_parameter<std::string>("bus_lock_file", "/tmp/rs485_modbus_bus.lock");
  this->declare_parameter<std::string>("sensor_name", "H2S");
  this->declare_parameter<std::string>("sensor_model", "SEN0467");
  this->declare_parameter<std::string>("quantity", "hydrogen_sulfide");
  this->declare_parameter<std::string>("unit", "ppm");
  this->declare_parameter<std::string>("topic_name", "/sensors/h2s");
  this->declare_parameter<std::string>("notes", "UART-over-RS485 using SEN0467");
  this->declare_parameter<double>("warning_threshold", 5.0);
  this->declare_parameter<double>("alarm_threshold", 10.0);
  this->declare_parameter<bool>("enabled", true);

  port_ = this->get_parameter("port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  address_ = this->get_parameter("address").as_int();
  response_timeout_ms_ = this->get_parameter("response_timeout_ms").as_int();
  poll_interval_ms_ = this->get_parameter("poll_interval_ms").as_int();
  bus_lock_file_ = this->get_parameter("bus_lock_file").as_string();
  sensor_name_ = this->get_parameter("sensor_name").as_string();
  sensor_model_ = this->get_parameter("sensor_model").as_string();
  quantity_ = this->get_parameter("quantity").as_string();
  unit_ = this->get_parameter("unit").as_string();
  topic_name_ = this->get_parameter("topic_name").as_string();
  notes_ = this->get_parameter("notes").as_string();
  warning_threshold_ = this->get_parameter("warning_threshold").as_double();
  alarm_threshold_ = this->get_parameter("alarm_threshold").as_double();
  enabled_ = this->get_parameter("enabled").as_bool();

  client_ = std::make_unique<Sen0467UartToRs485Client>(
    port_, baudrate_, response_timeout_ms_);

  publisher_ =
    this->create_publisher<interfaces::msg::Sensor>(topic_name_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms_),
    std::bind(&H2S_SensorNode::timer_callback, this));
}

std::string H2S_SensorNode::evaluate_status(int ppm) const
{
  if (alarm_threshold_ >= 0.0 && static_cast<double>(ppm) >= alarm_threshold_) {
    return "ALARM";
  }

  if (warning_threshold_ >= 0.0 && static_cast<double>(ppm) >= warning_threshold_) {
    return "WARN";
  }

  return "OK";
}

void H2S_SensorNode::timer_callback()
{
  if (!enabled_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      30000,
      "%s | DISABLED",
      sensor_name_.c_str());
    return;
  }

  try {
    BusLock bus_lock(bus_lock_file_);

    if (!client_->is_connected()) {
      client_->connect();
      passive_mode_configured_ = false;
    }

    if (!passive_mode_configured_) {
      client_->set_passive_mode(static_cast<uint8_t>(address_));
      passive_mode_configured_ = true;
    }

    const int ppm = client_->read_h2s_ppm(static_cast<uint8_t>(address_));
    const std::string status = evaluate_status(ppm);
    const bool alarm = (status == "ALARM");

    interfaces::msg::Sensor msg;
    msg.stamp = this->get_clock()->now();
    msg.sensor_name = sensor_name_;
    msg.sensor_model = sensor_model_;
    msg.quantity = quantity_;
    msg.value = static_cast<double>(ppm);
    msg.unit = unit_;
    msg.raw_value = ppm;
    msg.alarm = alarm;
    msg.slave_id = static_cast<uint16_t>(address_);
    msg.port = port_;
    msg.notes = notes_;

    publisher_->publish(msg);
  } catch (const std::exception & ex) {
    passive_mode_configured_ = false;
    if (client_ && client_->is_connected()) {
      client_->disconnect();
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "%s | READ_ERROR | %s",
      sensor_name_.c_str(),
      ex.what());
  }
}

}  // namespace sensor_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_pkg::H2S_SensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}