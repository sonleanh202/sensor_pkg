#include "sensors_reader/drivers/modbus_rtu_client.hpp"

#include <cerrno>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <modbus/modbus.h>

namespace sensors_reader
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
  int response_timeout_ms,
  int max_retries,
  int retry_delay_ms)
{
  if (register_count <= 0) {
    throw std::runtime_error("register_count must be greater than 0");
  }

  if (max_retries < 0) {
    max_retries = 0;
  }

  modbus_t * ctx = modbus_new_rtu(
    port.c_str(),
    baudrate,
    parity,
    data_bits,
    stop_bits);

  if (ctx == nullptr) {
    throw std::runtime_error("modbus_new_rtu() failed");
  }

  try {
    if (modbus_set_slave(ctx, slave_id) == -1) {
      throw std::runtime_error(
        "modbus_set_slave() failed: " +
        std::string(modbus_strerror(errno)));
    }

    struct timeval response_timeout;
    response_timeout.tv_sec = response_timeout_ms / 1000;
    response_timeout.tv_usec = (response_timeout_ms % 1000) * 1000;

    if (modbus_set_response_timeout(
        ctx,
        response_timeout.tv_sec,
        response_timeout.tv_usec) == -1)
    {
      throw std::runtime_error(
        "modbus_set_response_timeout() failed: " +
        std::string(modbus_strerror(errno)));
    }

    // Timeout giữa các byte trong cùng một frame.
    // Để ngắn hơn response timeout để tránh treo quá lâu khi frame bị vỡ.
    struct timeval byte_timeout;
    byte_timeout.tv_sec = 0;
    byte_timeout.tv_usec = 300000;  // 300 ms

    modbus_set_byte_timeout(
      ctx,
      byte_timeout.tv_sec,
      byte_timeout.tv_usec);

    // Cho libmodbus tự recover khi gặp lỗi link/protocol.
    modbus_set_error_recovery(
      ctx,
      static_cast<modbus_error_recovery_mode>(
        MODBUS_ERROR_RECOVERY_LINK |
        MODBUS_ERROR_RECOVERY_PROTOCOL));

    if (modbus_connect(ctx) == -1) {
      throw std::runtime_error(
        "modbus_connect() failed on port " +
        port +
        ": " +
        std::string(modbus_strerror(errno)));
    }

    std::string last_error;

    for (int attempt = 0; attempt <= max_retries; ++attempt) {
      std::vector<uint16_t> registers(
        static_cast<std::size_t>(register_count),
        0U);

      // Xóa frame cũ/rác trước khi gửi request mới.
      modbus_flush(ctx);

      const int rc = modbus_read_registers(
        ctx,
        start_address,
        register_count,
        registers.data());

      if (rc == register_count) {
        modbus_close(ctx);
        modbus_free(ctx);
        return registers;
      }

      if (rc == -1) {
        last_error = modbus_strerror(errno);
      } else {
        std::ostringstream oss;
        oss << "unexpected register count, expected "
            << register_count
            << ", got "
            << rc;
        last_error = oss.str();
      }

      // Nếu còn lượt retry thì nghỉ ngắn để bus ổn lại.
      if (attempt < max_retries) {
        modbus_flush(ctx);
        std::this_thread::sleep_for(
          std::chrono::milliseconds(retry_delay_ms));
      }
    }

    throw std::runtime_error(
      "modbus_read_registers() failed after retries: " + last_error);
  } catch (...) {
    modbus_close(ctx);
    modbus_free(ctx);
    throw;
  }
}

}  // namespace sensors_reader