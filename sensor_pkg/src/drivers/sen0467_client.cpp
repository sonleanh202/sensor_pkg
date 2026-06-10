#include "sensor_pkg/drivers/sen0467_client.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace sensor_pkg
{

namespace
{

std::string hex_dump(const uint8_t * data, size_t len)
{
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');

  for (size_t i = 0; i < len; ++i) {
    if (i > 0) {
      oss << " ";
    }
    oss << std::setw(2) << static_cast<int>(data[i]);
  }

  return oss.str();
}

template<size_t N>
std::string hex_dump_array(const std::array<uint8_t, N> & data)
{
  return hex_dump(data.data(), data.size());
}

}  // namespace

Sen0467UartToRs485Client::Sen0467UartToRs485Client(
  const std::string & port,
  int baudrate,
  int timeout_ms)
: fd_(-1),
  port_(port),
  baudrate_(baudrate),
  timeout_ms_(timeout_ms)
{
}

Sen0467UartToRs485Client::~Sen0467UartToRs485Client()
{
  disconnect();
}

void Sen0467UartToRs485Client::connect()
{
  if (is_connected()) {
    return;
  }

  open_port();
  configure_port();
  flush_io();

  std::cerr << "[SEN0467] connected port=" << port_
            << " baudrate=" << baudrate_
            << " timeout_ms=" << timeout_ms_
            << std::endl;
}

void Sen0467UartToRs485Client::disconnect()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool Sen0467UartToRs485Client::is_connected() const
{
  return fd_ >= 0;
}

void Sen0467UartToRs485Client::open_port()
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

  if (fd_ < 0) {
    std::ostringstream oss;
    oss << "open(" << port_ << ") failed: " << ::strerror(errno);
    throw std::runtime_error(oss.str());
  }
}

void Sen0467UartToRs485Client::configure_port()
{
  struct termios tty;
  ::memset(&tty, 0, sizeof(tty));

  if (::tcgetattr(fd_, &tty) != 0) {
    std::ostringstream oss;
    oss << "tcgetattr failed: " << ::strerror(errno);
    throw std::runtime_error(oss.str());
  }

  ::cfmakeraw(&tty);

  const speed_t speed = baud_to_speed(baudrate_);
  ::cfsetispeed(&tty, speed);
  ::cfsetospeed(&tty, speed);

  // DFRobot SEN0467 UART: 9600 8N1, no flow control.
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
    std::ostringstream oss;
    oss << "tcsetattr failed: " << ::strerror(errno);
    throw std::runtime_error(oss.str());
  }
}

void Sen0467UartToRs485Client::flush_io()
{
  if (fd_ >= 0) {
    ::tcflush(fd_, TCIOFLUSH);
  }
}

void Sen0467UartToRs485Client::drain_input_until_idle(int idle_ms, int max_wait_ms)
{
  if (fd_ < 0) {
    return;
  }

  std::array<uint8_t, 256> drained{};
  size_t drained_total = 0;

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(max_wait_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    struct timeval tv;
    tv.tv_sec = idle_ms / 1000;
    tv.tv_usec = (idle_ms % 1000) * 1000;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    const int sel = ::select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (sel < 0) {
      if (errno == EINTR) {
        continue;
      }

      std::ostringstream oss;
      oss << "select failed while draining bus: " << ::strerror(errno);
      throw std::runtime_error(oss.str());
    }

    // No byte during idle_ms => bus is quiet enough.
    if (sel == 0) {
      break;
    }

    uint8_t buf[64];
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }

      std::ostringstream oss;
      oss << "read failed while draining bus: " << ::strerror(errno);
      throw std::runtime_error(oss.str());
    }

    if (n > 0 && drained_total < drained.size()) {
      const size_t room = drained.size() - drained_total;
      const size_t copy_n = static_cast<size_t>(n) < room ? static_cast<size_t>(n) : room;
      ::memcpy(drained.data() + drained_total, buf, copy_n);
      drained_total += copy_n;
    }
  }

  if (drained_total > 0) {
    std::cerr << "[SEN0467] drained bus noise before TX: "
              << hex_dump(drained.data(), drained_total)
              << std::endl;
  }
}

void Sen0467UartToRs485Client::write_frame(const std::array<uint8_t, 9> & frame)
{
  std::cerr << "[SEN0467] TX: " << hex_dump_array(frame) << std::endl;

  size_t written_total = 0;

  while (written_total < frame.size()) {
    const ssize_t ret = ::write(
      fd_,
      frame.data() + written_total,
      frame.size() - written_total);

    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }

      std::ostringstream oss;
      oss << "write failed: " << ::strerror(errno);
      throw std::runtime_error(oss.str());
    }

    if (ret == 0) {
      continue;
    }

    written_total += static_cast<size_t>(ret);
  }

  ::tcdrain(fd_);
}

std::array<uint8_t, 9> Sen0467UartToRs485Client::read_frame()
{
  std::array<uint8_t, 9> frame{};
  size_t idx = 0;

  std::array<uint8_t, 512> raw{};
  size_t raw_idx = 0;

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);

  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    const auto remaining_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();

    struct timeval tv;
    tv.tv_sec = static_cast<time_t>(remaining_ms / 1000);
    tv.tv_usec = static_cast<suseconds_t>((remaining_ms % 1000) * 1000);

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);

    const int sel = ::select(fd_ + 1, &readfds, nullptr, nullptr, &tv);

    if (sel < 0) {
      if (errno == EINTR) {
        continue;
      }

      std::ostringstream oss;
      oss << "select failed: " << ::strerror(errno);
      throw std::runtime_error(oss.str());
    }

    if (sel == 0) {
      break;
    }

    uint8_t byte = 0;
    const ssize_t n = ::read(fd_, &byte, 1);

    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN) {
        continue;
      }

      std::ostringstream oss;
      oss << "read failed: " << ::strerror(errno);
      throw std::runtime_error(oss.str());
    }

    if (n == 0) {
      continue;
    }

    if (raw_idx < raw.size()) {
      raw[raw_idx++] = byte;
    }

    // DFRobot frame starts with 0xFF. Modbus frames are ignored here.
    if (idx == 0) {
      if (byte != 0xFF) {
        continue;
      }
      frame[idx++] = byte;
      continue;
    }

    frame[idx++] = byte;

    if (idx == frame.size()) {
      try {
        validate_checksum(frame);
        std::cerr << "[SEN0467] RX: " << hex_dump_array(frame) << std::endl;
        return frame;
      } catch (const std::exception & ex) {
        std::cerr << "[SEN0467] bad frame: "
                  << hex_dump_array(frame)
                  << " | " << ex.what()
                  << std::endl;
        idx = 0;
      }
    }
  }

  if (raw_idx > 0) {
    std::cerr << "[SEN0467] RAW RX before timeout: "
              << hex_dump(raw.data(), raw_idx)
              << std::endl;
  } else {
    std::cerr << "[SEN0467] RAW RX before timeout: <no bytes>" << std::endl;
  }

  throw std::runtime_error("timeout waiting sensor response");
}

speed_t Sen0467UartToRs485Client::baud_to_speed(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    default:
      throw std::runtime_error("unsupported baudrate");
  }
}

uint8_t Sen0467UartToRs485Client::calc_checksum(const std::array<uint8_t, 9> & frame)
{
  uint8_t sum = 0;
  for (size_t i = 1; i <= 7; ++i) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  return static_cast<uint8_t>(~sum + 1);
}

std::array<uint8_t, 9> Sen0467UartToRs485Client::build_frame(
  uint8_t address,
  uint8_t command,
  uint8_t data3,
  uint8_t data4,
  uint8_t data5,
  uint8_t data6,
  uint8_t data7)
{
  std::array<uint8_t, 9> frame{};

  frame[0] = 0xFF;
  frame[1] = address;
  frame[2] = command;
  frame[3] = data3;
  frame[4] = data4;
  frame[5] = data5;
  frame[6] = data6;
  frame[7] = data7;
  frame[8] = calc_checksum(frame);

  return frame;
}

void Sen0467UartToRs485Client::validate_checksum(const std::array<uint8_t, 9> & frame)
{
  const uint8_t expected = calc_checksum(frame);
  if (frame[8] != expected) {
    std::ostringstream oss;
    oss << "checksum mismatch, expected=0x"
        << std::hex << static_cast<int>(expected)
        << ", got=0x"
        << std::hex << static_cast<int>(frame[8]);
    throw std::runtime_error(oss.str());
  }
}

void Sen0467UartToRs485Client::set_passive_mode(uint8_t address)
{
  if (!is_connected()) {
    connect();
  }

  const auto request = build_frame(address, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00);

  // Important when sharing the RS485 bus: wait until old Modbus bytes stop.
  flush_io();
  drain_input_until_idle();
  write_frame(request);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const auto response = read_frame();

  // DFRobot checks ACK at response[2] == 0x01.
  if (response[0] != 0xFF || response[2] != 0x01) {
    std::ostringstream oss;
    oss << "set passive mode failed, response=" << hex_dump_array(response);
    throw std::runtime_error(oss.str());
  }

  flush_io();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

double Sen0467UartToRs485Client::read_h2s_ppm(uint8_t address)
{
  if (!is_connected()) {
    connect();
  }

  const auto request = build_frame(address, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00);

  flush_io();
  drain_input_until_idle();
  write_frame(request);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const auto response = read_frame();

  if (response[0] != 0xFF) {
    std::ostringstream oss;
    oss << "invalid response header, response=" << hex_dump_array(response);
    throw std::runtime_error(oss.str());
  }

  double ppm = static_cast<double>(
    static_cast<int>(response[2]) * 256 + static_cast<int>(response[3]));

  const uint8_t gas_type = response[4];
  const uint8_t decimal_digits = response[5];

  if (decimal_digits == 1) {
    ppm *= 0.1;
  } else if (decimal_digits == 2) {
    ppm *= 0.01;
  }

  std::cerr << "[SEN0467] parsed gas_type=0x"
            << std::hex << static_cast<int>(gas_type)
            << std::dec
            << " decimal_digits=" << static_cast<int>(decimal_digits)
            << " ppm=" << ppm
            << std::endl;

  return ppm;
}

}  // namespace sensor_pkg
