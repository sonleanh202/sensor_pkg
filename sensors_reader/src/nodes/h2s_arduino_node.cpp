#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/sensor.hpp"

namespace sensors_reader
{

namespace
{
speed_t baud_to_termios(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: throw std::runtime_error("Unsupported baudrate. Use 9600, 19200, 38400, 57600 or 115200.");
  }
}

std::string trim(const std::string & input)
{
  const auto begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

bool parse_double(const std::string & text, double & value)
{
  char * end_ptr = nullptr;
  errno = 0;
  const double parsed = std::strtod(text.c_str(), &end_ptr);
  if (end_ptr == text.c_str() || errno == ERANGE) {
    return false;
  }
  value = parsed;
  return true;
}

bool parse_after_label(const std::string & line, const std::string & label, double & value)
{
  const auto pos = line.find(label);
  if (pos == std::string::npos) {
    return false;
  }

  std::string tail = line.substr(pos + label.size());
  tail = trim(tail);

  char * end_ptr = nullptr;
  errno = 0;
  const double parsed = std::strtod(tail.c_str(), &end_ptr);
  if (end_ptr == tail.c_str() || errno == ERANGE) {
    return false;
  }

  value = parsed;
  return true;
}

bool parse_csv_h2s(const std::string & line, double & raw, double & ppm)
{
  // Expected Arduino line: H2S,<raw_adc>,<estimated_ppm>
  if (line.rfind("H2S,", 0) != 0) {
    return false;
  }

  std::stringstream ss(line);
  std::string token;

  std::getline(ss, token, ',');  // H2S

  if (!std::getline(ss, token, ',')) {
    return false;
  }
  if (!parse_double(trim(token), raw)) {
    return false;
  }

  if (!std::getline(ss, token, ',')) {
    return false;
  }
  if (!parse_double(trim(token), ppm)) {
    return false;
  }

  return true;
}
}  // namespace

class H2sArduinoNode : public rclcpp::Node
{
public:
  H2sArduinoNode()
  : Node("h2s_arduino_node")
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
    baudrate_ = this->declare_parameter<int>("baudrate", 9600);
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/sensor/h2s");
    sensor_name_ = this->declare_parameter<std::string>("sensor_name", "H2S");
    sensor_model_ = this->declare_parameter<std::string>("sensor_model", "SEN0568 + Arduino Nano");
    warning_threshold_ppm_ = this->declare_parameter<double>("warning_threshold_ppm", 5.0);
    alarm_threshold_ppm_ = this->declare_parameter<double>("alarm_threshold_ppm", 10.0);
    read_interval_ms_ = this->declare_parameter<int>("read_interval_ms", 100);
    enabled_ = this->declare_parameter<bool>("enabled", true);

    publisher_ = this->create_publisher<interfaces::msg::Sensor>(topic_name_, 10);

    if (enabled_) {
      open_serial();
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(read_interval_ms_),
      [this]() { read_serial(); });
  }

  ~H2sArduinoNode() override
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
  }

private:
  void open_serial()
  {
    if (fd_ >= 0) {
      return;
    }

    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("Cannot open serial port " + port_ + ": " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      const std::string err = std::strerror(errno);
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("tcgetattr failed on " + port_ + ": " + err);
    }

    cfmakeraw(&tty);

    const speed_t speed = baud_to_termios(baudrate_);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      const std::string err = std::strerror(errno);
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("tcsetattr failed on " + port_ + ": " + err);
    }

    tcflush(fd_, TCIOFLUSH);

    RCLCPP_INFO(
      this->get_logger(),
      "Opened Arduino H2S serial port %s at %d baud, publishing %s",
      port_.c_str(), baudrate_, topic_name_.c_str());
  }

  void read_serial()
  {
    if (!enabled_) {
      return;
    }

    if (fd_ < 0) {
      open_serial();
    }

    char buf[256];

    while (true) {
      const ssize_t n = read(fd_, buf, sizeof(buf));

      if (n > 0) {
        buffer_.append(buf, static_cast<size_t>(n));
      } else if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", std::strerror(errno));
        close(fd_);
        fd_ = -1;
        return;
      }
    }

    size_t newline_pos = std::string::npos;
    while ((newline_pos = buffer_.find('\n')) != std::string::npos) {
      const std::string line = trim(buffer_.substr(0, newline_pos));
      buffer_.erase(0, newline_pos + 1);

      if (!line.empty()) {
        parse_and_publish(line);
      }
    }

    if (buffer_.size() > 1024) {
      buffer_.clear();
    }
  }

  void parse_and_publish(const std::string & line)
  {
    double raw_adc = -1.0;
    double ppm = 0.0;

    bool parsed = parse_csv_h2s(line, raw_adc, ppm);

    if (!parsed) {
      // Also accept lines printed by older Arduino test code:
      // Raw ADC: 967.7 | Voltage: 4.730 V | H2S estimated: 0.33 ppm
      double parsed_raw = -1.0;
      double parsed_ppm = 0.0;
      const bool got_raw = parse_after_label(line, "Raw ADC:", parsed_raw);
      const bool got_ppm = parse_after_label(line, "H2S estimated:", parsed_ppm);

      if (got_ppm) {
        raw_adc = got_raw ? parsed_raw : -1.0;
        ppm = parsed_ppm;
        parsed = true;
      }
    }

    if (!parsed) {
      // Last fallback: a line containing only ppm, for example: 0.32
      parsed = parse_double(line, ppm);
      raw_adc = -1.0;
    }

    if (!parsed) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Cannot parse Arduino H2S line: '%s'",
        line.c_str());
      return;
    }

    if (!std::isfinite(ppm)) {
      return;
    }

    if (ppm < 0.0) {
      ppm = 0.0;
    }

    interfaces::msg::Sensor msg;
    msg.stamp = this->get_clock()->now();
    msg.sensor_name = sensor_name_;
    msg.sensor_model = sensor_model_;
    msg.quantity = "hydrogen_sulfide";
    msg.value = ppm;
    msg.unit = "ppm";
    msg.raw_value = raw_adc >= 0.0 ? static_cast<int32_t>(std::lround(raw_adc)) : -1;
    msg.alarm = ppm >= alarm_threshold_ppm_;
    msg.slave_id = 0;
    msg.port = port_;
    msg.notes = "Estimated H2S ppm from SEN0568 analog sensor via Arduino Nano serial. Needs calibration for real ppm.";

    publisher_->publish(msg);

    if (msg.alarm) {
      RCLCPP_WARN(
        this->get_logger(),
        "H2S: %.2f ppm | raw ADC: %d | Alarm",
        msg.value,
        msg.raw_value);
    }
  }

  std::string port_;
  int baudrate_;
  std::string topic_name_;
  std::string sensor_name_;
  std::string sensor_model_;
  double warning_threshold_ppm_;
  double alarm_threshold_ppm_;
  int read_interval_ms_;
  bool enabled_;

  int fd_{-1};
  std::string buffer_;

  rclcpp::Publisher<interfaces::msg::Sensor>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sensors_reader

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensors_reader::H2sArduinoNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
