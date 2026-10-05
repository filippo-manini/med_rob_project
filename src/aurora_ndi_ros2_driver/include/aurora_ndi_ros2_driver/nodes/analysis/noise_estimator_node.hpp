#ifndef NOISE_ESTIMATOR_NODE_HPP
#define NOISE_ESTIMATOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <aurora_ndi_ros2_driver/msg/aurora_data.hpp>

#include <vector>
#include <string>
#include <mutex>
#include <array>
#include <cmath>

namespace aurora_ndi_ros2_driver
{

/**
 * @brief Node for estimating noise characteristics of Aurora sensors
 *
 * This node subscribes to Aurora sensor data and computes statistical
 * properties of the noise (mean, std deviation, RMS) for both position
 * and orientation. It's designed to run with the sensor held stationary
 * to characterize the baseline noise, which can then be used to configure
 * the Kalman filter parameters.
 *
 * The node can operate in two modes:
 * 1. Continuous mode: continuously updates statistics
 * 2. Batch mode: collects N samples then outputs results and exits
 */
class NoiseEstimatorNode : public rclcpp::Node
{
public:
    NoiseEstimatorNode();
    ~NoiseEstimatorNode() = default;

private:
    struct Parameters {
        // Sensor configuration
        int num_sensors;
        std::vector<std::string> topic_names;

        // Data collection parameters
        int min_samples;              // Minimum samples before computing statistics
        int max_samples;              // Maximum samples to collect (0 = unlimited)
        double collection_duration_sec; // Max duration for data collection (0 = unlimited)

        // Operating mode
        bool batch_mode;              // If true, exit after collecting max_samples
        bool auto_start;              // If true, start collecting immediately
        double publish_rate_hz;       // Rate to publish statistics (continuous mode)

        // Filtering
        bool require_visible;         // Only use samples where sensor is visible
        double max_error_threshold;   // Reject samples with error > threshold (0 = no filter)

        // Output
        bool save_to_file;            // Save results to YAML file
        std::string output_file;      // Output file path
        bool verbose;                 // Print detailed statistics
    };

    struct SensorStatistics {
        // Sample data storage
        std::vector<std::array<double, 3>> position_samples;
        std::vector<std::array<double, 4>> orientation_samples;
        std::vector<double> error_samples;

        // Position statistics (x, y, z)
        std::array<double, 3> position_mean{{0.0, 0.0, 0.0}};
        std::array<double, 3> position_std{{0.0, 0.0, 0.0}};
        std::array<double, 3> position_min{{0.0, 0.0, 0.0}};
        std::array<double, 3> position_max{{0.0, 0.0, 0.0}};
        double position_rms{0.0};

        // Orientation statistics (qx, qy, qz, qw)
        std::array<double, 4> orientation_mean{{0.0, 0.0, 0.0, 1.0}};
        std::array<double, 4> orientation_std{{0.0, 0.0, 0.0, 0.0}};

        // Error statistics
        double error_mean{0.0};
        double error_std{0.0};
        double error_max{0.0};

        // Sample counts
        size_t total_samples{0};
        size_t valid_samples{0};
        size_t rejected_samples{0};

        // Timing
        rclcpp::Time first_sample_time;
        rclcpp::Time last_sample_time;
    };

    // Parameter management
    void declare_parameters();
    void load_parameters();
    bool validate_parameters();

    // Data collection callbacks
    void sensor_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg, int sensor_index);

    // Statistics computation
    void compute_statistics(int sensor_index);
    void compute_position_statistics(SensorStatistics& stats);
    void compute_orientation_statistics(SensorStatistics& stats);
    void compute_error_statistics(SensorStatistics& stats);

    // Output functions
    void publish_statistics_callback();
    void print_statistics();
    void print_sensor_statistics(int sensor_index, const SensorStatistics& stats);
    void save_statistics_to_file();

    // Utility functions
    bool is_collection_complete();
    void shutdown_node();

    // Member variables
    Parameters params_;
    std::vector<SensorStatistics> sensor_stats_;
    std::vector<rclcpp::Subscription<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr> subscribers_;
    rclcpp::TimerBase::SharedPtr statistics_timer_;

    std::mutex data_mutex_;
    rclcpp::Time collection_start_time_;
    bool collection_started_{false};
    bool collection_complete_{false};
};

} // namespace aurora_ndi_ros2_driver

#endif // NOISE_ESTIMATOR_NODE_HPP
