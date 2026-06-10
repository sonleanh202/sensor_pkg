#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace sensor_pkg
{

class ModbusRtuClient
{
public:
  static std::vector<uint16_t> read_holding_registers(
    const std::string & port,
    int baudrate,
    char parity,
    int data_bits,
    int stop_bits,
    int slave_id,
    int start_address,
    int register_count,
    int response_timeout_ms,
    int max_retries = 3,
    int retry_delay_ms = 100);
};

}  // namespace sensor_pkg