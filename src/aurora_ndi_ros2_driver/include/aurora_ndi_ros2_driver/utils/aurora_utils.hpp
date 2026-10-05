#pragma once

#include <string>
#include <vector>

namespace aurora_ndi_ros2_driver
{
namespace utils
{

void normalize_quaternion(double quat[4]);

bool validate_tool_handle(const std::string& handle);

double calculate_moving_average(const std::vector<double>& data_buffer);

} // namespace utils
} // namespace aurora_ndi_ros2_driver