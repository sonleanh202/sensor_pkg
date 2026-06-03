#pragma once

#include <array>
#include <string>

#include <termios.h>

namespace sensor_pkg
{

class Sen0467UartToRs485Client
{
public:
  Sen0467UartToRs485Client(
    const std::string & port,
    int baudrate,
    int timeout_ms);

  ~Sen0467UartToRs485Client();

  void connect();
  void disconnect();
  bool is_connected() const;

  void set_passive_mode(uint8_t address = 0x01);
  int read_h2s_ppm(uint8_t address = 0x01);

private:
  int fd_;
  std::string port_;
  int baudrate_;
  int timeout_ms_;

  void open_port();
  void configure_port();
  void flush_io();

  void write_frame(const std::array<uint8_t, 9> & frame);
  std::array<uint8_t, 9> read_frame();

  static speed_t baud_to_speed(int baudrate);
  static uint8_t calc_checksum(const std::array<uint8_t, 9> & frame);
  static std::array<uint8_t, 9> build_frame(
    uint8_t address,
    uint8_t command,
    uint8_t data3 = 0x00,
    uint8_t data4 = 0x00,
    uint8_t data5 = 0x00,
    uint8_t data6 = 0x00,
    uint8_t data7 = 0x00);

  static void validate_checksum(const std::array<uint8_t, 9> & frame);
};

}  // namespace sensor_pkg