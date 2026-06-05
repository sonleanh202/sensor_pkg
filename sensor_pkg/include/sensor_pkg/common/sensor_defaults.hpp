#pragma once

#include <string>

namespace sensor_pkg
{

struct SensorDefaults
{
  std::string sensor_name;
  std::string sensor_model;
  std::string quantity;
  std::string unit;
  std::string topic_name;

  int read_register{0};
  int register_count{1};

  double scale{1.0};
  double offset{0.0};
  bool signed_value{false};

  bool enabled{true};
  std::string notes;
};

}  // namespace sensor_pkg