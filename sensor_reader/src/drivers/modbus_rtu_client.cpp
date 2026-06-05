#include "sensor_reader/drivers/modbus_rtu_client.hpp"

#include <cerrno>
#include <stdexcept>
#include <string>
#include <vector>

#include <modbus/modbus.h>

namespace sensor_reader
{

std::vector<uint16_t> ModbusRtuClient::read_holding_registers(
  const std::string & port,
  int baudrate,
  char parity,
  int data_bits,
  int stop_bits,
  int slave_id,
  int start_address,
  int register_count,
  int response_timeout_ms)
{
  modbus_t * ctx = modbus_new_rtu(port.c_str(), baudrate, parity, data_bits, stop_bits);
  if (ctx == nullptr) {
    throw std::runtime_error("modbus_new_rtu() failed");
  }

  try {
    if (modbus_set_slave(ctx, slave_id) == -1) {
      throw std::runtime_error("modbus_set_slave() failed: " + std::string(modbus_strerror(errno)));
    }

    struct timeval timeout;
    timeout.tv_sec = response_timeout_ms / 1000;
    timeout.tv_usec = (response_timeout_ms % 1000) * 1000;
    if (modbus_set_response_timeout(ctx, timeout.tv_sec, timeout.tv_usec) == -1) {
      throw std::runtime_error(
        "modbus_set_response_timeout() failed: " + std::string(modbus_strerror(errno)));
    }

    if (modbus_connect(ctx) == -1) {
      throw std::runtime_error(
        "modbus_connect() failed on port " + port + ": " + std::string(modbus_strerror(errno)));
    }

    std::vector<uint16_t> registers(static_cast<std::size_t>(register_count), 0U);
    const int rc = modbus_read_registers(ctx, start_address, register_count, registers.data());

    if (rc == -1) {
      throw std::runtime_error(
        "modbus_read_registers() failed: " + std::string(modbus_strerror(errno)));
    }

    modbus_close(ctx);
    modbus_free(ctx);
    return registers;
  } catch (...) {
    modbus_close(ctx);
    modbus_free(ctx);
    throw;
  }
}

} 
