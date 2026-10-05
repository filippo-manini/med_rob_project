#ifndef AURORA_NDI_ROS2_DRIVER__LOWPASS_FILTER_NODE_HPP_
#define AURORA_NDI_ROS2_DRIVER__LOWPASS_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "aurora_ndi_ros2_driver/msg/aurora_data.hpp"

namespace aurora_ndi_ros2_driver
{

class LowpassFilterNode : public rclcpp::Node
{
public:
    LowpassFilterNode();
    ~LowpassFilterNode() = default;

private:
    // Configuration parameters
    struct Parameters
    {
        std::string input_topic;
        std::string output_topic_suffix;
        bool enable_lowpass_filter;

        // Low-pass filter parameters
        double cutoff_frequency_hz;       // Cutoff frequency in Hz
        double sample_rate_hz;            // Expected sample rate in Hz

        // Filtering options
        bool require_visible;
        double max_error_threshold;
    };

    // Low-pass filter state for position (3D)
    struct PositionFilterState
    {
        Eigen::Vector3d filtered_value;
        double alpha;  // Filter coefficient (0 = only old data, 1 = only new data)
        bool initialized;

        PositionFilterState() : initialized(false), alpha(1.0)
        {
            filtered_value.setZero();
        }
    };

    // Low-pass filter state for orientation (quaternion - 4D)
    struct OrientationFilterState
    {
        Eigen::Vector4d filtered_value;
        double alpha;  // Filter coefficient
        bool initialized;

        OrientationFilterState() : initialized(false), alpha(1.0)
        {
            filtered_value.setZero();
            filtered_value(3) = 1.0;  // Initialize w=1 for identity quaternion
        }
    };

    // ROS interfaces
    rclcpp::Subscription<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr subscriber_;
    rclcpp::Publisher<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr publisher_;

    // Filter states
    PositionFilterState position_filter_;
    OrientationFilterState orientation_filter_;

    // Parameters
    Parameters params_;

    // Methods
    void declare_parameters();
    void load_parameters();
    bool validate_parameters();
    void sensor_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg);

    // Low-pass filter functions
    void initialize_position_filter(const Eigen::Vector3d& initial_position);
    void initialize_orientation_filter(const Eigen::Vector4d& initial_orientation);
    Eigen::Vector3d apply_lowpass_position(const Eigen::Vector3d& measurement);
    Eigen::Vector4d apply_lowpass_orientation(const Eigen::Vector4d& measurement);
    Eigen::Vector4d normalize_quaternion(const Eigen::Vector4d& q);
    double compute_alpha(double cutoff_freq_hz, double sample_rate_hz);
};

} // namespace aurora_ndi_ros2_driver

#endif // AURORA_NDI_ROS2_DRIVER__LOWPASS_FILTER_NODE_HPP_
