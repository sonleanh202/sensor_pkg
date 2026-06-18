#pragma once

#include <string>

namespace sensors_reader
{

struct SensorDefaults
{
  std::string sensor_name;
  std::string sensor_model;
  std::string quantity;
  std::string unit;
  std::string topic_name;
  std::string notes;
  int read_register{0};
  int register_count{1};
  double scale{1.0};
  double offset{0.0};
  bool signed_value{false};
  bool alarm_when_nonzero{false};
  double warning_threshold{-1.0};
  double alarm_threshold{-1.0};
  bool enabled{true};
};

}  // namespace sensors_reader
