#include "aurora_ndi_ros2_driver/filters/aurora_data_filter.hpp"
#include "aurora_ndi_ros2_driver/utils/aurora_utils.hpp"
#include <cmath>
#include <algorithm>

namespace aurora_ndi_ros2_driver
{

AuroraDataFilter::AuroraDataFilter(const FilterParameters& params)
    : params_(params)
{
}

AuroraDataFilter::FilteredData AuroraDataFilter::applyFilter(const FilteredData& new_data)
{
    if (!params_.enable_filtering) {
        return new_data;
    }

    // Add new data to buffer
    data_buffer_.push_back(new_data);

    // Maintain window size
    if (static_cast<int>(data_buffer_.size()) > params_.window_size) {
        data_buffer_.pop_front();
    }

    // Need at least 2 samples for filtering
    if (data_buffer_.size() < 2) {
        return new_data;
    }

    FilteredData filtered_data = new_data;

    // Filter position components
    for (int i = 0; i < 3; ++i) {
        std::vector<double> position_values;
        for (const auto& data : data_buffer_) {
            if (data.visible) {
                position_values.push_back(data.position[i]);
            }
        }

        if (!position_values.empty()) {
            filtered_data.position[i] = utils::calculate_moving_average(position_values);
        }
    }

    // Filter orientation components
    for (int i = 0; i < 4; ++i) {
        std::vector<double> orientation_values;
        for (const auto& data : data_buffer_) {
            if (data.visible) {
                orientation_values.push_back(data.orientation[i]);
            }
        }

        if (!orientation_values.empty()) {
            filtered_data.orientation[i] = utils::calculate_moving_average(orientation_values);
        }
    }

    // Normalize quaternion
    utils::normalize_quaternion(filtered_data.orientation.data());

    return filtered_data;
}

void AuroraDataFilter::reset()
{
    data_buffer_.clear();
}

} // namespace aurora_ndi_ros2_driver
