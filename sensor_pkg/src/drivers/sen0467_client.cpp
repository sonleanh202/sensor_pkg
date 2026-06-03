#include "sensor_pkg/drivers/sen0467_client.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include <chrono>
#include <sstream>
#include <stdexcept>

namespace sensor_pkg
{

Sen0467UartToRs485Client::Sen0467UartToRs485Client(
  const std::string & port,
  int baudrate,
  int timeout_ms)
: fd_(-1), port_(port), baudrate_(baudrate), timeout_ms_(timeout_ms)
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

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;   // no parity
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;       // 8 data bits
  tty.c_cflag &= ~CRTSCTS;  // no hw flow control

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

void Sen0467UartToRs485Client::write_frame(const std::array<uint8_t, 9> & frame)
{
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

    written_total += static_cast<size_t>(ret);
  }

  ::tcdrain(fd_);
}

std::array<uint8_t, 9> Sen0467UartToRs485Client::read_frame()
{
  std::array<uint8_t, 9> frame{};
  size_t received = 0;

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);

  while (received < frame.size()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      throw std::runtime_error("timeout waiting sensor response");
    }

    const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - now).count();

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
      throw std::runtime_error("timeout waiting sensor response");
    }

    const ssize_t n = ::read(fd_, frame.data() + received, frame.size() - received);
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

    received += static_cast<size_t>(n);
  }

  return frame;
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
    throw std::runtime_error("checksum mismatch");
  }
}

void Sen0467UartToRs485Client::set_passive_mode(uint8_t address)
{
  if (!is_connected()) {
    connect();
  }

  // FF 01 78 04 00 00 00 00 83
  const auto request = build_frame(address, 0x78, 0x04, 0x00, 0x00, 0x00, 0x00);

  flush_io();
  write_frame(request);

  const auto response = read_frame();
  validate_checksum(response);

  if (response[0] != 0xFF || response[1] != 0x78 || response[2] != 0x01) {
    throw std::runtime_error("set passive mode failed");
  }

  flush_io();
}

int Sen0467UartToRs485Client::read_h2s_ppm(uint8_t address)
{
  if (!is_connected()) {
    connect();
  }

  // FF 01 86 00 00 00 00 00 79
  const auto request = build_frame(address, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00);

  flush_io();
  write_frame(request);

  const auto response = read_frame();
  validate_checksum(response);

  if (response[0] != 0xFF || response[1] != 0x86) {
    throw std::runtime_error("invalid response header");
  }

  const int ppm =
    static_cast<int>(response[2]) * 256 +
    static_cast<int>(response[3]);

  return ppm;
}

}  // namespace sensor_pkg