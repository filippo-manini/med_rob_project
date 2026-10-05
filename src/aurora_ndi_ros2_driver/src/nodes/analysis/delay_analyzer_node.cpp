#include "aurora_ndi_ros2_driver/nodes/analysis/delay_analyzer_node.hpp"
#include <numeric>
#include <cmath>
#include <iomanip>

using namespace aurora_ndi_ros2_driver;

DelayAnalyzerNode::DelayAnalyzerNode() : Node("delay_analyzer_node")
{
    // Declare parameters
    this->declare_parameter("raw_topic", std::string("aurora/sensor0"));
    this->declare_parameter("lowpass_topic", std::string("aurora/sensor0/lowpass_filter"));
    this->declare_parameter("kalman_topic", std::string("aurora/sensor0/kalman_filter"));
    this->declare_parameter("duration_sec", 30.0);

    // Get parameters
    raw_topic_ = this->get_parameter("raw_topic").as_string();
    lowpass_topic_ = this->get_parameter("lowpass_topic").as_string();
    kalman_topic_ = this->get_parameter("kalman_topic").as_string();
    duration_sec_ = this->get_parameter("duration_sec").as_double();

    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "Delay Analyzer Node Starting");
    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "Raw topic:     %s", raw_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Lowpass topic: %s", lowpass_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Kalman topic:  %s", kalman_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Duration:      %.1f seconds", duration_sec_);
    RCLCPP_INFO(this->get_logger(), "===================================");

    // Create subscribers
    raw_sub_ = this->create_subscription<aurora_ndi_ros2_driver::msg::AuroraData>(
        raw_topic_, 10,
        std::bind(&DelayAnalyzerNode::raw_callback, this, std::placeholders::_1));

    lowpass_sub_ = this->create_subscription<aurora_ndi_ros2_driver::msg::AuroraData>(
        lowpass_topic_, 10,
        std::bind(&DelayAnalyzerNode::lowpass_callback, this, std::placeholders::_1));

    kalman_sub_ = this->create_subscription<aurora_ndi_ros2_driver::msg::AuroraData>(
        kalman_topic_, 10,
        std::bind(&DelayAnalyzerNode::kalman_callback, this, std::placeholders::_1));

    // Create shutdown timer
    shutdown_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(duration_sec_),
        std::bind(&DelayAnalyzerNode::timer_callback, this));

    // Record start time
    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "Collecting data for %.1f seconds...", duration_sec_);
}

void DelayAnalyzerNode::raw_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg)
{
    if (!msg->visible) return;

    // Store arrival time (wall clock) for this message timestamp
    rclcpp::Time msg_stamp(msg->header.stamp);
    rclcpp::Time arrival_time = this->now();

    // Use nanoseconds as unique key for the timestamp
    int64_t stamp_key = msg_stamp.nanoseconds();
    raw_arrival_times_[stamp_key] = arrival_time;

    // Clean up old entries (keep only last 1000)
    if (raw_arrival_times_.size() > 1000) {
        raw_arrival_times_.erase(raw_arrival_times_.begin());
    }
}

void DelayAnalyzerNode::lowpass_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg)
{
    if (!msg->visible) return;

    // Get message timestamp and arrival time
    rclcpp::Time msg_stamp(msg->header.stamp);
    rclcpp::Time arrival_time = this->now();
    int64_t stamp_key = msg_stamp.nanoseconds();

    // Store lowpass arrival time for later comparison with Kalman
    lowpass_arrival_times_[stamp_key] = arrival_time;

    // Find corresponding RAW message with same timestamp
    auto raw_it = raw_arrival_times_.find(stamp_key);
    if (raw_it != raw_arrival_times_.end()) {
        // Compute actual processing delay
        double delay_ms = (arrival_time - raw_it->second).seconds() * 1000.0;
        raw_to_lowpass_delays_.push_back(delay_ms);
    }

    // Clean up old entries
    if (lowpass_arrival_times_.size() > 1000) {
        lowpass_arrival_times_.erase(lowpass_arrival_times_.begin());
    }
}

void DelayAnalyzerNode::kalman_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg)
{
    if (!msg->visible) return;

    // Get message timestamp and arrival time
    rclcpp::Time msg_stamp(msg->header.stamp);
    rclcpp::Time arrival_time = this->now();
    int64_t stamp_key = msg_stamp.nanoseconds();

    // Find corresponding RAW message with same timestamp
    auto raw_it = raw_arrival_times_.find(stamp_key);
    if (raw_it != raw_arrival_times_.end()) {
        // Compute actual processing delay from RAW to KALMAN
        double delay_ms = (arrival_time - raw_it->second).seconds() * 1000.0;
        raw_to_kalman_delays_.push_back(delay_ms);
    }

    // Find corresponding LOWPASS message with same timestamp
    auto lowpass_it = lowpass_arrival_times_.find(stamp_key);
    if (lowpass_it != lowpass_arrival_times_.end()) {
        // Compute actual processing delay from LOWPASS to KALMAN
        double delay_ms = (arrival_time - lowpass_it->second).seconds() * 1000.0;
        lowpass_to_kalman_delays_.push_back(delay_ms);
    }
}

void DelayAnalyzerNode::timer_callback()
{
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "Data collection completed!");
    RCLCPP_INFO(this->get_logger(), "===================================");

    compute_statistics();
    print_results();

    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "Shutting down...");
    rclcpp::shutdown();
}

DelayAnalyzerNode::DelayStatistics DelayAnalyzerNode::compute_delay_stats(const std::vector<double>& delays)
{
    DelayStatistics stats;

    if (delays.empty()) {
        return stats;
    }

    stats.sample_count = delays.size();

    // Mean
    stats.mean_delay_ms = std::accumulate(delays.begin(), delays.end(), 0.0) / delays.size();

    // Min and Max
    stats.min_delay_ms = *std::min_element(delays.begin(), delays.end());
    stats.max_delay_ms = *std::max_element(delays.begin(), delays.end());

    // Standard deviation
    double variance = 0.0;
    for (double delay : delays) {
        variance += std::pow(delay - stats.mean_delay_ms, 2);
    }
    variance /= delays.size();
    stats.std_dev_ms = std::sqrt(variance);

    return stats;
}

void DelayAnalyzerNode::compute_statistics()
{
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "Computing statistics...");
}

void DelayAnalyzerNode::print_results()
{
    RCLCPP_INFO(this->get_logger(), "");
    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "      DELAY ANALYSIS RESULTS      ");
    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "");

    // Raw to Lowpass
    if (!raw_to_lowpass_delays_.empty()) {
        auto stats = compute_delay_stats(raw_to_lowpass_delays_);
        RCLCPP_INFO(this->get_logger(), "RAW -> LOWPASS FILTER:");
        RCLCPP_INFO(this->get_logger(), "  Samples:   %zu", stats.sample_count);
        RCLCPP_INFO(this->get_logger(), "  Mean:      %.3f ms", stats.mean_delay_ms);
        RCLCPP_INFO(this->get_logger(), "  Std Dev:   %.3f ms", stats.std_dev_ms);
        RCLCPP_INFO(this->get_logger(), "  Min:       %.3f ms", stats.min_delay_ms);
        RCLCPP_INFO(this->get_logger(), "  Max:       %.3f ms", stats.max_delay_ms);
        RCLCPP_INFO(this->get_logger(), "");
    } else {
        RCLCPP_WARN(this->get_logger(), "RAW -> LOWPASS FILTER: No data collected");
        RCLCPP_INFO(this->get_logger(), "");
    }

    // Raw to Kalman
    if (!raw_to_kalman_delays_.empty()) {
        auto stats = compute_delay_stats(raw_to_kalman_delays_);
        RCLCPP_INFO(this->get_logger(), "RAW -> KALMAN FILTER:");
        RCLCPP_INFO(this->get_logger(), "  Samples:   %zu", stats.sample_count);
        RCLCPP_INFO(this->get_logger(), "  Mean:      %.3f ms", stats.mean_delay_ms);
        RCLCPP_INFO(this->get_logger(), "  Std Dev:   %.3f ms", stats.std_dev_ms);
        RCLCPP_INFO(this->get_logger(), "  Min:       %.3f ms", stats.min_delay_ms);
        RCLCPP_INFO(this->get_logger(), "  Max:       %.3f ms", stats.max_delay_ms);
        RCLCPP_INFO(this->get_logger(), "");
    } else {
        RCLCPP_WARN(this->get_logger(), "RAW -> KALMAN FILTER: No data collected");
        RCLCPP_INFO(this->get_logger(), "");
    }

    // Lowpass to Kalman
    if (!lowpass_to_kalman_delays_.empty()) {
        auto stats = compute_delay_stats(lowpass_to_kalman_delays_);
        RCLCPP_INFO(this->get_logger(), "LOWPASS -> KALMAN FILTER:");
        RCLCPP_INFO(this->get_logger(), "  Samples:   %zu", stats.sample_count);
        RCLCPP_INFO(this->get_logger(), "  Mean:      %.3f ms", stats.mean_delay_ms);
        RCLCPP_INFO(this->get_logger(), "  Std Dev:   %.3f ms", stats.std_dev_ms);
        RCLCPP_INFO(this->get_logger(), "  Min:       %.3f ms", stats.min_delay_ms);
        RCLCPP_INFO(this->get_logger(), "  Max:       %.3f ms", stats.max_delay_ms);
        RCLCPP_INFO(this->get_logger(), "");
    } else {
        RCLCPP_WARN(this->get_logger(), "LOWPASS -> KALMAN FILTER: No data collected");
        RCLCPP_INFO(this->get_logger(), "");
    }

    RCLCPP_INFO(this->get_logger(), "===================================");

    // Summary
    if (raw_to_lowpass_delays_.empty() && raw_to_kalman_delays_.empty() && lowpass_to_kalman_delays_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "");
        RCLCPP_ERROR(this->get_logger(), "WARNING: No delay data was collected!");
        RCLCPP_ERROR(this->get_logger(), "Please verify that the topics are publishing data:");
        RCLCPP_ERROR(this->get_logger(), "  - %s", raw_topic_.c_str());
        RCLCPP_ERROR(this->get_logger(), "  - %s", lowpass_topic_.c_str());
        RCLCPP_ERROR(this->get_logger(), "  - %s", kalman_topic_.c_str());
        RCLCPP_ERROR(this->get_logger(), "");
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<DelayAnalyzerNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
