#include "aurora_ndi_ros2_driver/utils/aurora_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>

namespace aurora_ndi_ros2_driver
{
namespace utils
{

void normalize_quaternion(double quat[4])
{
    // Calculate magnitude
    double magnitude = std::sqrt(quat[0]*quat[0] + quat[1]*quat[1] + 
                                quat[2]*quat[2] + quat[3]*quat[3]);
    
    // Avoid division by zero
    if (magnitude < 1e-8) {
        // Set to identity quaternion
        quat[0] = 0.0; // qx
        quat[1] = 0.0; // qy  
        quat[2] = 0.0; // qz
        quat[3] = 1.0; // qw
        return;
    }
    
    // Normalize
    quat[0] /= magnitude;
    quat[1] /= magnitude;
    quat[2] /= magnitude;
    quat[3] /= magnitude;
}

bool validate_tool_handle(const std::string& handle)
{
    // Tool handle should be exactly 2 hexadecimal characters
    if (handle.length() != 2) {
        return false;
    }
    
    return std::all_of(handle.begin(), handle.end(), [](char c) {
        return std::isxdigit(c);
    });
}

double calculate_moving_average(const std::vector<double>& data_buffer)
{
    if (data_buffer.empty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (double value : data_buffer) {
        sum += value;
    }

    return sum / data_buffer.size();
}

} // namespace utils
} // namespace aurora_ndi_ros2_driver