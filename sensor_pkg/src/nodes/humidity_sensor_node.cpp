#include "sensor_pkg/nodes/humidity_sensor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "sensor_pkg/common/bus_lock.hpp"
#include "sensor_pkg/drivers/modbus_rtu_client.hpp"

namespace sensor_pkg
{

HumiditySensorNode::HumiditySensorNode()
: Node("humidity_sensor_node")
{
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 9600);
  this->declare_parameter<std::string>("parity", "N");
  this->declare_parameter<int>("data_bits", 8);
  this->declare_parameter<int>("stop_bits", 1);
  this->declare_parameter<int>("slave_id", 7);
  this->declare_parameter<int>("response_timeout_ms", 1000);
  this->declare_parameter<int>("poll_interval_ms", 5000);
  this->declare_parameter<std::string>("bus_lock_file", "/tmp/rs485_modbus_bus.lock");
  this->declare_parameter<bool>("enabled", true);

  this->declare_parameter<int>("humidity_register", 0);
  this->declare_parameter<int>("temperature_register", 1);
  this->declare_parameter<double>("humidity_scale", 0.1);
  this->declare_parameter<double>("temperature_scale", 0.1);
  this->declare_parameter<double>("temperature_offset", 0.0);
  this->declare_parameter<bool>("temperature_signed", true);

  this->declare_parameter<double>("humidity_warning_threshold", 80.0);
  this->declare_parameter<double>("humidity_alarm_threshold", 90.0);

  this->declare_parameter<std::string>("sensor_model", "RT WS N01 QT");
  this->declare_parameter<std::string>("humidity_sensor_name", "Humidity");
  this->declare_parameter<std::string>("temperature_sensor_name", "Temperature");
  this->declare_parameter<std::string>("humidity_topic_name", "/sensors/humidity");
  this->declare_parameter<std::string>("temperature_topic_name", "/sensors/temperature");
  this->declare_parameter<std::string>("humidity_unit", "%RH");
  this->declare_parameter<std::string>("temperature_unit", "\u00B0C");
  this->declare_parameter<std::string>("humidity_notes", "Humidity at register 0");
  this->declare_parameter<std::string>("temperature_notes", "Temperature at register 1");

  port_ = this->get_parameter("port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  data_bits_ = this->get_parameter("data_bits").as_int();
  stop_bits_ = this->get_parameter("stop_bits").as_int();
  slave_id_ = this->get_parameter("slave_id").as_int();
  response_timeout_ms_ = this->get_parameter("response_timeout_ms").as_int();
  poll_interval_ms_ = this->get_parameter("poll_interval_ms").as_int();
  bus_lock_file_ = this->get_parameter("bus_lock_file").as_string();
  enabled_ = this->get_parameter("enabled").as_bool();

  humidity_register_ = this->get_parameter("humidity_register").as_int();
  temperature_register_ = this->get_parameter("temperature_register").as_int();
  humidity_scale_ = this->get_parameter("humidity_scale").as_double();
  temperature_scale_ = this->get_parameter("temperature_scale").as_double();
  temperature_offset_ = this->get_parameter("temperature_offset").as_double();
  temperature_signed_ = this->get_parameter("temperature_signed").as_bool();

  humidity_warning_threshold_ =
    this->get_parameter("humidity_warning_threshold").as_double();
  humidity_alarm_threshold_ =
    this->get_parameter("humidity_alarm_threshold").as_double();

  sensor_model_ = this->get_parameter("sensor_model").as_string();
  humidity_sensor_name_ = this->get_parameter("humidity_sensor_name").as_string();
  temperature_sensor_name_ = this->get_parameter("temperature_sensor_name").as_string();
  humidity_topic_name_ = this->get_parameter("humidity_topic_name").as_string();
  temperature_topic_name_ = this->get_parameter("temperature_topic_name").as_string();
  humidity_unit_ = this->get_parameter("humidity_unit").as_string();
  temperature_unit_ = this->get_parameter("temperature_unit").as_string();
  humidity_notes_ = this->get_parameter("humidity_notes").as_string();
  temperature_notes_ = this->get_parameter("temperature_notes").as_string();

  const std::string parity_string = this->get_parameter("parity").as_string();
  parity_ = parity_string.empty() ? 'N' : parity_string.front();

  humidity_publisher_ =
    this->create_publisher<interfaces::msg::Sensor>(humidity_topic_name_, 10);
  temperature_publisher_ =
    this->create_publisher<interfaces::msg::Sensor>(temperature_topic_name_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms_),
    std::bind(&HumiditySensorNode::timer_callback, this));
}

int32_t HumiditySensorNode::decode_signed(uint16_t raw) const
{
  return static_cast<int32_t>(static_cast<int16_t>(raw));
}

std::string HumiditySensorNode::evaluate_humidity_status(double humidity) const
{
  if (humidity_alarm_threshold_ >= 0.0 && humidity >= humidity_alarm_threshold_) {
    return "ALARM";
  }
  if (humidity_warning_threshold_ >= 0.0 && humidity >= humidity_warning_threshold_) {
    return "WARN";
  }
  return "OK";
}

void HumiditySensorNode::timer_callback()
{
  if (!enabled_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      30000,
      "%s | DISABLED",
      humidity_sensor_name_.c_str());
    return;
  }

  try {
    BusLock bus_lock(bus_lock_file_);

    const int start_register = std::min(humidity_register_, temperature_register_);
    const int register_count =
      std::max(humidity_register_, temperature_register_) - start_register + 1;

    const std::vector<uint16_t> registers = ModbusRtuClient::read_holding_registers(
      port_,
      baudrate_,
      parity_,
      data_bits_,
      stop_bits_,
      slave_id_,
      start_register,
      register_count,
      response_timeout_ms_);

    if (static_cast<int>(registers.size()) < register_count) {
      throw std::runtime_error("Received insufficient register data.");
    }

    const uint16_t humidity_raw_reg =
      registers.at(humidity_register_ - start_register);

    const uint16_t temperature_raw_reg =
      registers.at(temperature_register_ - start_register);

    const int32_t humidity_raw = static_cast<int32_t>(humidity_raw_reg);
    const int32_t temperature_raw = temperature_signed_
      ? decode_signed(temperature_raw_reg)
      : static_cast<int32_t>(temperature_raw_reg);

    const double humidity_value =
      static_cast<double>(humidity_raw) * humidity_scale_;

    const double temperature_value =
      static_cast<double>(temperature_raw) * temperature_scale_ + temperature_offset_;

    const std::string humidity_status = evaluate_humidity_status(humidity_value);
    const bool humidity_alarm = (humidity_status == "ALARM");

    interfaces::msg::Sensor humidity_msg;
    humidity_msg.stamp = this->get_clock()->now();
    humidity_msg.sensor_name = humidity_sensor_name_;
    humidity_msg.sensor_model = sensor_model_;
    humidity_msg.quantity = "humidity";
    humidity_msg.value = humidity_value;
    humidity_msg.unit = humidity_unit_;
    humidity_msg.raw_value = humidity_raw;
    humidity_msg.alarm = humidity_alarm;
    humidity_msg.slave_id = static_cast<uint16_t>(slave_id_);
    humidity_msg.port = port_;
    humidity_msg.notes = humidity_notes_;

    interfaces::msg::Sensor temperature_msg;
    temperature_msg.stamp = humidity_msg.stamp;
    temperature_msg.sensor_name = temperature_sensor_name_;
    temperature_msg.sensor_model = sensor_model_;
    temperature_msg.quantity = "temperature";
    temperature_msg.value = temperature_value;
    temperature_msg.unit = temperature_unit_;
    temperature_msg.raw_value = temperature_raw;
    temperature_msg.alarm = false;
    temperature_msg.slave_id = static_cast<uint16_t>(slave_id_);
    temperature_msg.port = port_;
    temperature_msg.notes = temperature_notes_;

    humidity_publisher_->publish(humidity_msg);
    temperature_publisher_->publish(temperature_msg);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s | READ_ERROR | %s",
      humidity_sensor_name_.c_str(),
      ex.what());
  }
}

}  // namespace sensor_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_pkg::HumiditySensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}