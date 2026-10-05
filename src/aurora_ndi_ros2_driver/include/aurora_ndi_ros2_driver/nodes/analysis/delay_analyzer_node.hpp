#ifndef DELAY_ANALYZER_NODE_HPP
#define DELAY_ANALYZER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <aurora_ndi_ros2_driver/msg/aurora_data.hpp>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <chrono>
#include <map>

namespace aurora_ndi_ros2_driver
{

class DelayAnalyzerNode : public rclcpp::Node
{
public:
    DelayAnalyzerNode();
    ~DelayAnalyzerNode() = default;

private:
    struct DelayStatistics {
        double mean_delay_ms{0.0};
        double min_delay_ms{0.0};
        double max_delay_ms{0.0};
        double std_dev_ms{0.0};
        size_t sample_count{0};
    };

    void raw_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg);
    void lowpass_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg);
    void kalman_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg);

    void timer_callback();
    void compute_statistics();
    void print_results();

    DelayStatistics compute_delay_stats(const std::vector<double>& delays);

    // Subscribers
    rclcpp::Subscription<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr raw_sub_;
    rclcpp::Subscription<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr lowpass_sub_;
    rclcpp::Subscription<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr kalman_sub_;

    // Timer for shutdown
    rclcpp::TimerBase::SharedPtr shutdown_timer_;

    // Map message timestamp to wall clock arrival time
    // Key: message timestamp (from msg->header.stamp)
    // Value: wall clock time when message was received
    std::map<int64_t, rclcpp::Time> raw_arrival_times_;
    std::map<int64_t, rclcpp::Time> lowpass_arrival_times_;

    // Delay measurements (in milliseconds)
    std::vector<double> raw_to_lowpass_delays_;
    std::vector<double> raw_to_kalman_delays_;
    std::vector<double> lowpass_to_kalman_delays_;

    // Configuration
    std::string raw_topic_;
    std::string lowpass_topic_;
    std::string kalman_topic_;
    double duration_sec_;

    // Start time
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace aurora_ndi_ros2_driver

#endif // DELAY_ANALYZER_NODE_HPP
