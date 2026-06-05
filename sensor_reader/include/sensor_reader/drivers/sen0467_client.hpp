#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace sensor_reader
{

class Sen0467UartToRs485Client
{
public:
  Sen0467UartToRs485Client(
    const std::string & port,
    int timeout_ms);

  ~Sen0467UartToRs485Client();

  void connect();
  void disconnect();
  bool is_connected() const;

  void set_passive_mode(uint8_t address);
  int read_h2s_ppm(uint8_t address);

private:
  void open_port();
  void configure_port();
  void flush_io();

  void write_frame(const std::array<uint8_t, 9> & frame);
  std::array<uint8_t, 9> read_frame();

  static uint8_t calc_checksum(const std::array<uint8_t, 9> & frame);

  static std::array<uint8_t, 9> build_frame(
    uint8_t address,
    uint8_t command,
    uint8_t data3,
    uint8_t data4,
    uint8_t data5,
    uint8_t data6,
    uint8_t data7);

  static void validate_checksum(const std::array<uint8_t, 9> & frame);

  int fd_;
  std::string port_;
  int timeout_ms_;
};

}  // namespace sensor_reader