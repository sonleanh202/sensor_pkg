#include "sensor_pkg/nodes/generic_modbus_sensor_node.hpp"

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

GenericModbusSensorNode::GenericModbusSensorNode(
  const std::string & node_name,
  const SensorDefaults & defaults)
: Node(node_name)
{
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<int>("baudrate", 9600);
  this->declare_parameter<std::string>("parity", "N");
  this->declare_parameter<int>("data_bits", 8);
  this->declare_parameter<int>("stop_bits", 1);
  this->declare_parameter<int>("slave_id", 1);
  this->declare_parameter<int>("read_register", defaults.read_register);
  this->declare_parameter<int>("register_count", defaults.register_count);
  this->declare_parameter<std::string>("word_order", "high_low");
  this->declare_parameter<double>("scale", defaults.scale);
  this->declare_parameter<double>("offset", defaults.offset);
  this->declare_parameter<bool>("signed_value", defaults.signed_value);
  this->declare_parameter<bool>("enabled", defaults.enabled);
  this->declare_parameter<int>("response_timeout_ms", 3000);
  this->declare_parameter<int>("poll_interval_ms", 5000);
  this->declare_parameter<std::string>("bus_lock_file", "/tmp/rs485_modbus_bus.lock");
  this->declare_parameter<std::string>("sensor_name", defaults.sensor_name);
  this->declare_parameter<std::string>("sensor_model", defaults.sensor_model);
  this->declare_parameter<std::string>("quantity", defaults.quantity);
  this->declare_parameter<std::string>("unit", defaults.unit);
  this->declare_parameter<std::string>("topic_name", defaults.topic_name);
  this->declare_parameter<std::string>("notes", defaults.notes);

  port_ = this->get_parameter("port").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  data_bits_ = this->get_parameter("data_bits").as_int();
  stop_bits_ = this->get_parameter("stop_bits").as_int();
  slave_id_ = this->get_parameter("slave_id").as_int();
  read_register_ = this->get_parameter("read_register").as_int();
  register_count_ = this->get_parameter("register_count").as_int();
  word_order_ = this->get_parameter("word_order").as_string();
  scale_ = this->get_parameter("scale").as_double();
  offset_ = this->get_parameter("offset").as_double();
  signed_value_ = this->get_parameter("signed_value").as_bool();
  enabled_ = this->get_parameter("enabled").as_bool();
  response_timeout_ms_ = this->get_parameter("response_timeout_ms").as_int();
  poll_interval_ms_ = this->get_parameter("poll_interval_ms").as_int();
  bus_lock_file_ = this->get_parameter("bus_lock_file").as_string();
  sensor_name_ = this->get_parameter("sensor_name").as_string();
  sensor_model_ = this->get_parameter("sensor_model").as_string();
  quantity_ = this->get_parameter("quantity").as_string();
  unit_ = this->get_parameter("unit").as_string();
  topic_name_ = this->get_parameter("topic_name").as_string();
  notes_ = this->get_parameter("notes").as_string();

  const std::string parity_string = this->get_parameter("parity").as_string();
  parity_ = parity_string.empty() ? 'N' : parity_string.front();

  if (register_count_ <= 0) {
    throw std::runtime_error("register_count must be greater than 0");
  }

  publisher_ = this->create_publisher<interfaces::msg::Sensor>(topic_name_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms_),
    std::bind(&GenericModbusSensorNode::timer_callback, this));
}

int32_t GenericModbusSensorNode::decode_raw_value(
  const std::vector<uint16_t> & registers) const
{
  if (registers.empty()) {
    throw std::runtime_error("Received empty register vector.");
  }

  if (registers.size() == 1) {
    if (signed_value_) {
      return static_cast<int32_t>(static_cast<int16_t>(registers[0]));
    }

    return static_cast<int32_t>(registers[0]);
  }

  uint32_t raw_u32 = 0;

  if (word_order_ == "low_high") {
    raw_u32 =
      (static_cast<uint32_t>(registers[1]) << 16) |
      static_cast<uint32_t>(registers[0]);
  } else {
    raw_u32 =
      (static_cast<uint32_t>(registers[0]) << 16) |
      static_cast<uint32_t>(registers[1]);
  }

  if (signed_value_) {
    return static_cast<int32_t>(raw_u32);
  }

  if (raw_u32 > static_cast<uint32_t>(INT32_MAX)) {
    throw std::runtime_error("Decoded unsigned 32-bit value exceeds int32 range.");
  }

  return static_cast<int32_t>(raw_u32);
}

void GenericModbusSensorNode::timer_callback()
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

    const std::vector<uint16_t> registers = ModbusRtuClient::read_holding_registers(
      port_,
      baudrate_,
      parity_,
      data_bits_,
      stop_bits_,
      slave_id_,
      read_register_,
      register_count_,
      response_timeout_ms_);

    const int32_t raw_value = decode_raw_value(registers);
    const double value = static_cast<double>(raw_value) * scale_ + offset_;

    interfaces::msg::Sensor msg;
    msg.stamp = this->get_clock()->now();
    msg.sensor_name = sensor_name_;
    msg.sensor_model = sensor_model_;
    msg.quantity = quantity_;
    msg.value = value;
    msg.unit = unit_;
    msg.raw_value = raw_value;
    msg.slave_id = static_cast<uint16_t>(slave_id_);
    msg.port = port_;
    msg.notes = notes_;

    publisher_->publish(msg);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(),
      "%s | READ_ERROR | %s",
      sensor_name_.c_str(),
      ex.what());
  }
}

}  // namespace sensor_pkg