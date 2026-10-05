#pragma once

#include <deque>
#include <vector>
#include <array>

namespace aurora_ndi_ros2_driver
{

/**
 * @brief Handles data filtering for Aurora sensor data
 *
 * This class provides:
 * - Moving average filtering for position and orientation
 */
class AuroraDataFilter
{
public:
    struct FilteredData {
        std::array<double, 3> position{{0.0, 0.0, 0.0}};
        std::array<double, 4> orientation{{0.0, 0.0, 0.0, 1.0}};
        double error{0.0};
        bool visible{false};
    };

    struct FilterParameters {
        bool enable_filtering{true};
        int window_size{4};
    };

    AuroraDataFilter(const FilterParameters& params);

    /**
     * @brief Apply moving average filter to new data
     * @param new_data New measurement data
     * @return Filtered data
     */
    FilteredData applyFilter(const FilteredData& new_data);

    /**
     * @brief Reset filter state
     */
    void reset();

private:
    FilterParameters params_;
    std::deque<FilteredData> data_buffer_;
};

} // namespace aurora_ndi_ros2_driver
