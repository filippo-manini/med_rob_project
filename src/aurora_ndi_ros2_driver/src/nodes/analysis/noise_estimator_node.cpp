#include "aurora_ndi_ros2_driver/nodes/analysis/noise_estimator_node.hpp"

#include <fstream>
#include <iomanip>
#include <numeric>
#include <algorithm>
#include <cmath>

using namespace aurora_ndi_ros2_driver;

NoiseEstimatorNode::NoiseEstimatorNode() : Node("noise_estimator_node")
{
    declare_parameters();
    load_parameters();

    if (!validate_parameters()) {
        RCLCPP_FATAL(this->get_logger(), "Parameter validation failed. Shutting down.");
        rclcpp::shutdown();
        return;
    }

    // Initialize statistics storage for each sensor
    sensor_stats_.resize(params_.num_sensors);

    // Create subscribers for each sensor
    subscribers_.resize(params_.num_sensors);
    for (int i = 0; i < params_.num_sensors; ++i) {
        subscribers_[i] = this->create_subscription<aurora_ndi_ros2_driver::msg::AuroraData>(
            params_.topic_names[i],
            10,
            [this, i](const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg) {
                this->sensor_callback(msg, i);
            });

        RCLCPP_INFO(this->get_logger(), "Subscribed to sensor %d on topic: %s",
                    i, params_.topic_names[i].c_str());
    }

    // Create timer for continuous statistics publishing
    if (!params_.batch_mode && params_.publish_rate_hz > 0.0) {
        auto timer_period = std::chrono::duration<double>(1.0 / params_.publish_rate_hz);
        statistics_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&NoiseEstimatorNode::publish_statistics_callback, this));
    }

    if (params_.auto_start) {
        collection_started_ = true;
        collection_start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Data collection started automatically");
    }

    RCLCPP_INFO(this->get_logger(),
                "Noise Estimator Node initialized for %d sensors\n"
                "  Mode: %s\n"
                "  Min samples: %d\n"
                "  Max samples: %d\n"
                "  Max duration: %.1f sec\n"
                "  Require visible: %s\n"
                "  Max error threshold: %.2f mm",
                params_.num_sensors,
                params_.batch_mode ? "BATCH" : "CONTINUOUS",
                params_.min_samples,
                params_.max_samples,
                params_.collection_duration_sec,
                params_.require_visible ? "YES" : "NO",
                params_.max_error_threshold);

    if (params_.batch_mode) {
        RCLCPP_INFO(this->get_logger(), "\n==============================================");
        RCLCPP_INFO(this->get_logger(), "  KEEP THE SENSOR STATIONARY!");
        RCLCPP_INFO(this->get_logger(), "  Collecting noise data...");
        RCLCPP_INFO(this->get_logger(), "==============================================\n");
    }
}

void NoiseEstimatorNode::declare_parameters()
{
    // Sensor configuration
    this->declare_parameter("num_sensors", 1);
    this->declare_parameter("topic_names", std::vector<std::string>{"/aurora_data_sensor0"});

    // Data collection parameters
    this->declare_parameter("min_samples", 100);
    this->declare_parameter("max_samples", 1000);
    this->declare_parameter("collection_duration_sec", 30.0);

    // Operating mode
    this->declare_parameter("batch_mode", true);
    this->declare_parameter("auto_start", true);
    this->declare_parameter("publish_rate_hz", 1.0);

    // Filtering
    this->declare_parameter("require_visible", true);
    this->declare_parameter("max_error_threshold", 5.0);

    // Output
    this->declare_parameter("save_to_file", true);
    this->declare_parameter("output_file", "noise_calibration_results.yaml");
    this->declare_parameter("verbose", true);
}

void NoiseEstimatorNode::load_parameters()
{
    params_.num_sensors = this->get_parameter("num_sensors").as_int();
    params_.topic_names = this->get_parameter("topic_names").as_string_array();

    params_.min_samples = this->get_parameter("min_samples").as_int();
    params_.max_samples = this->get_parameter("max_samples").as_int();
    params_.collection_duration_sec = this->get_parameter("collection_duration_sec").as_double();

    params_.batch_mode = this->get_parameter("batch_mode").as_bool();
    params_.auto_start = this->get_parameter("auto_start").as_bool();
    params_.publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();

    params_.require_visible = this->get_parameter("require_visible").as_bool();
    params_.max_error_threshold = this->get_parameter("max_error_threshold").as_double();

    params_.save_to_file = this->get_parameter("save_to_file").as_bool();
    params_.output_file = this->get_parameter("output_file").as_string();
    params_.verbose = this->get_parameter("verbose").as_bool();
}

bool NoiseEstimatorNode::validate_parameters()
{
    bool valid = true;

    if (params_.num_sensors < 1 || params_.num_sensors > 4) {
        RCLCPP_ERROR(this->get_logger(), "Invalid num_sensors: %d (must be 1-4)", params_.num_sensors);
        valid = false;
    }

    if (static_cast<int>(params_.topic_names.size()) != params_.num_sensors) {
        RCLCPP_ERROR(this->get_logger(), "topic_names size (%zu) doesn't match num_sensors (%d)",
                     params_.topic_names.size(), params_.num_sensors);
        valid = false;
    }

    if (params_.min_samples < 10) {
        RCLCPP_ERROR(this->get_logger(), "min_samples (%d) must be at least 10", params_.min_samples);
        valid = false;
    }

    if (params_.max_samples > 0 && params_.max_samples < params_.min_samples) {
        RCLCPP_ERROR(this->get_logger(), "max_samples (%d) must be >= min_samples (%d)",
                     params_.max_samples, params_.min_samples);
        valid = false;
    }

    return valid;
}

void NoiseEstimatorNode::sensor_callback(
    const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg, int sensor_index)
{
    if (!collection_started_) {
        return;
    }

    if (collection_complete_) {
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    auto& stats = sensor_stats_[sensor_index];
    stats.total_samples++;

    // Initialize timing on first sample
    if (stats.total_samples == 1) {
        stats.first_sample_time = msg->header.stamp;
    }
    stats.last_sample_time = msg->header.stamp;

    // Apply filters
    if (params_.require_visible && !msg->visible) {
        stats.rejected_samples++;
        return;
    }

    if (params_.max_error_threshold > 0.0 && msg->error > params_.max_error_threshold) {
        stats.rejected_samples++;
        return;
    }

    // Store valid sample
    std::array<double, 3> position = {msg->position.x, msg->position.y, msg->position.z};
    std::array<double, 4> orientation = {msg->orientation.x, msg->orientation.y,
                                          msg->orientation.z, msg->orientation.w};

    stats.position_samples.push_back(position);
    stats.orientation_samples.push_back(orientation);
    stats.error_samples.push_back(msg->error);
    stats.valid_samples++;

    // Check if collection is complete
    if (is_collection_complete()) {
        collection_complete_ = true;

        // Compute final statistics
        for (int i = 0; i < params_.num_sensors; ++i) {
            compute_statistics(i);
        }

        // Print and save results
        print_statistics();

        if (params_.save_to_file) {
            save_statistics_to_file();
        }

        // Shutdown in batch mode
        if (params_.batch_mode) {
            RCLCPP_INFO(this->get_logger(), "\nData collection complete. Shutting down...");
            shutdown_node();
        }
    } else {
        // Progress update every 50 samples in batch mode
        if (params_.batch_mode && stats.valid_samples % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "Sensor %d: Collected %zu/%d valid samples",
                        sensor_index, stats.valid_samples, params_.max_samples);
        }
    }
}

bool NoiseEstimatorNode::is_collection_complete()
{
    // Check if any sensor has reached max_samples
    if (params_.max_samples > 0) {
        for (const auto& stats : sensor_stats_) {
            if (stats.valid_samples >= static_cast<size_t>(params_.max_samples)) {
                return true;
            }
        }
    }

    // Check if duration exceeded
    if (params_.collection_duration_sec > 0.0) {
        double elapsed = (this->now() - collection_start_time_).seconds();
        if (elapsed >= params_.collection_duration_sec) {
            return true;
        }
    }

    return false;
}

void NoiseEstimatorNode::compute_statistics(int sensor_index)
{
    auto& stats = sensor_stats_[sensor_index];

    if (stats.valid_samples < static_cast<size_t>(params_.min_samples)) {
        RCLCPP_WARN(this->get_logger(),
                    "Sensor %d: Insufficient samples (%zu < %d) for reliable statistics",
                    sensor_index, stats.valid_samples, params_.min_samples);
        return;
    }

    compute_position_statistics(stats);
    compute_orientation_statistics(stats);
    compute_error_statistics(stats);
}

void NoiseEstimatorNode::compute_position_statistics(SensorStatistics& stats)
{
    const size_t n = stats.position_samples.size();
    if (n == 0) return;

    // Compute mean
    for (int axis = 0; axis < 3; ++axis) {
        double sum = 0.0;
        for (const auto& pos : stats.position_samples) {
            sum += pos[axis];
        }
        stats.position_mean[axis] = sum / n;
    }

    // Compute std deviation and min/max
    for (int axis = 0; axis < 3; ++axis) {
        double sum_sq = 0.0;
        stats.position_min[axis] = stats.position_samples[0][axis];
        stats.position_max[axis] = stats.position_samples[0][axis];

        for (const auto& pos : stats.position_samples) {
            double diff = pos[axis] - stats.position_mean[axis];
            sum_sq += diff * diff;

            stats.position_min[axis] = std::min(stats.position_min[axis], pos[axis]);
            stats.position_max[axis] = std::max(stats.position_max[axis], pos[axis]);
        }

        stats.position_std[axis] = std::sqrt(sum_sq / n);
    }

    // Compute RMS deviation from mean position
    double sum_rms = 0.0;
    for (const auto& pos : stats.position_samples) {
        double dist_sq = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            double diff = pos[axis] - stats.position_mean[axis];
            dist_sq += diff * diff;
        }
        sum_rms += std::sqrt(dist_sq);
    }
    stats.position_rms = sum_rms / n;
}

void NoiseEstimatorNode::compute_orientation_statistics(SensorStatistics& stats)
{
    const size_t n = stats.orientation_samples.size();
    if (n == 0) return;

    // Compute mean quaternion (simple average - sufficient for small variations)
    for (int i = 0; i < 4; ++i) {
        double sum = 0.0;
        for (const auto& quat : stats.orientation_samples) {
            sum += quat[i];
        }
        stats.orientation_mean[i] = sum / n;
    }

    // Normalize mean quaternion
    double norm = std::sqrt(stats.orientation_mean[0] * stats.orientation_mean[0] +
                           stats.orientation_mean[1] * stats.orientation_mean[1] +
                           stats.orientation_mean[2] * stats.orientation_mean[2] +
                           stats.orientation_mean[3] * stats.orientation_mean[3]);
    for (int i = 0; i < 4; ++i) {
        stats.orientation_mean[i] /= norm;
    }

    // Compute std deviation for each quaternion component
    for (int i = 0; i < 4; ++i) {
        double sum_sq = 0.0;
        for (const auto& quat : stats.orientation_samples) {
            double diff = quat[i] - stats.orientation_mean[i];
            sum_sq += diff * diff;
        }
        stats.orientation_std[i] = std::sqrt(sum_sq / n);
    }
}

void NoiseEstimatorNode::compute_error_statistics(SensorStatistics& stats)
{
    const size_t n = stats.error_samples.size();
    if (n == 0) return;

    // Compute mean
    double sum = std::accumulate(stats.error_samples.begin(), stats.error_samples.end(), 0.0);
    stats.error_mean = sum / n;

    // Compute std deviation
    double sum_sq = 0.0;
    for (double err : stats.error_samples) {
        double diff = err - stats.error_mean;
        sum_sq += diff * diff;
    }
    stats.error_std = std::sqrt(sum_sq / n);

    // Find max
    stats.error_max = *std::max_element(stats.error_samples.begin(), stats.error_samples.end());
}

void NoiseEstimatorNode::publish_statistics_callback()
{
    if (!collection_started_ || collection_complete_) {
        return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);

    // Compute current statistics
    for (int i = 0; i < params_.num_sensors; ++i) {
        if (sensor_stats_[i].valid_samples >= static_cast<size_t>(params_.min_samples)) {
            compute_statistics(i);
        }
    }

    if (params_.verbose) {
        print_statistics();
    }
}

void NoiseEstimatorNode::print_statistics()
{
    RCLCPP_INFO(this->get_logger(), "\n============================================");
    RCLCPP_INFO(this->get_logger(), "  NOISE ESTIMATION RESULTS");
    RCLCPP_INFO(this->get_logger(), "============================================");

    for (int i = 0; i < params_.num_sensors; ++i) {
        print_sensor_statistics(i, sensor_stats_[i]);
    }

    RCLCPP_INFO(this->get_logger(), "============================================\n");
}

void NoiseEstimatorNode::print_sensor_statistics(int sensor_index, const SensorStatistics& stats)
{
    if (stats.valid_samples == 0) {
        RCLCPP_WARN(this->get_logger(), "Sensor %d: No valid samples collected", sensor_index);
        return;
    }

    double collection_duration = 0.0;
    if (stats.valid_samples > 1) {
        collection_duration = (stats.last_sample_time - stats.first_sample_time).seconds();
    }

    RCLCPP_INFO(this->get_logger(), "\n--- Sensor %d (Topic: %s) ---",
                sensor_index, params_.topic_names[sensor_index].c_str());
    RCLCPP_INFO(this->get_logger(), "Samples: %zu valid / %zu total (%.1f%% rejected)",
                stats.valid_samples, stats.total_samples,
                100.0 * stats.rejected_samples / std::max(stats.total_samples, size_t(1)));
    RCLCPP_INFO(this->get_logger(), "Duration: %.2f seconds", collection_duration);

    RCLCPP_INFO(this->get_logger(), "\nPosition Noise (mm):");
    RCLCPP_INFO(this->get_logger(), "  Mean:    [%.3f, %.3f, %.3f]",
                stats.position_mean[0], stats.position_mean[1], stats.position_mean[2]);
    RCLCPP_INFO(this->get_logger(), "  Std Dev: [%.3f, %.3f, %.3f]",
                stats.position_std[0], stats.position_std[1], stats.position_std[2]);
    RCLCPP_INFO(this->get_logger(), "  Range X: [%.3f, %.3f] (%.3f mm)",
                stats.position_min[0], stats.position_max[0],
                stats.position_max[0] - stats.position_min[0]);
    RCLCPP_INFO(this->get_logger(), "  Range Y: [%.3f, %.3f] (%.3f mm)",
                stats.position_min[1], stats.position_max[1],
                stats.position_max[1] - stats.position_min[1]);
    RCLCPP_INFO(this->get_logger(), "  Range Z: [%.3f, %.3f] (%.3f mm)",
                stats.position_min[2], stats.position_max[2],
                stats.position_max[2] - stats.position_min[2]);
    RCLCPP_INFO(this->get_logger(), "  RMS deviation: %.3f mm", stats.position_rms);

    RCLCPP_INFO(this->get_logger(), "\nOrientation Noise:");
    RCLCPP_INFO(this->get_logger(), "  Mean Quaternion: [%.4f, %.4f, %.4f, %.4f]",
                stats.orientation_mean[0], stats.orientation_mean[1],
                stats.orientation_mean[2], stats.orientation_mean[3]);
    RCLCPP_INFO(this->get_logger(), "  Std Dev:         [%.4f, %.4f, %.4f, %.4f]",
                stats.orientation_std[0], stats.orientation_std[1],
                stats.orientation_std[2], stats.orientation_std[3]);

    RCLCPP_INFO(this->get_logger(), "\nError Statistics (mm):");
    RCLCPP_INFO(this->get_logger(), "  Mean:    %.3f", stats.error_mean);
    RCLCPP_INFO(this->get_logger(), "  Std Dev: %.3f", stats.error_std);
    RCLCPP_INFO(this->get_logger(), "  Max:     %.3f", stats.error_max);

    // Recommendations for Kalman filter
    RCLCPP_INFO(this->get_logger(), "\n>>> RECOMMENDED KALMAN FILTER PARAMETERS <<<");
    RCLCPP_INFO(this->get_logger(), "  measurement_noise_position: %.4f  # R matrix (position variance)",
                std::pow(std::max({stats.position_std[0], stats.position_std[1], stats.position_std[2]}), 2));
    RCLCPP_INFO(this->get_logger(), "  measurement_noise_orientation: %.6f  # R matrix (orientation variance)",
                std::pow(std::max({stats.orientation_std[0], stats.orientation_std[1],
                                   stats.orientation_std[2], stats.orientation_std[3]}), 2));
}

void NoiseEstimatorNode::save_statistics_to_file()
{
    std::ofstream file(params_.output_file);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", params_.output_file.c_str());
        return;
    }

    file << "# Noise Calibration Results\n";
    file << "# Generated by NoiseEstimatorNode\n";
    file << "# Timestamp: " << this->now().seconds() << "\n\n";

    file << "noise_calibration:\n";
    file << "  num_sensors: " << params_.num_sensors << "\n\n";

    for (int i = 0; i < params_.num_sensors; ++i) {
        const auto& stats = sensor_stats_[i];

        if (stats.valid_samples == 0) {
            continue;
        }

        file << "  sensor_" << i << ":\n";
        file << "    topic: \"" << params_.topic_names[i] << "\"\n";
        file << "    samples:\n";
        file << "      valid: " << stats.valid_samples << "\n";
        file << "      total: " << stats.total_samples << "\n";
        file << "      rejected: " << stats.rejected_samples << "\n";

        file << "    position:\n";
        file << "      mean: [" << stats.position_mean[0] << ", "
             << stats.position_mean[1] << ", " << stats.position_mean[2] << "]\n";
        file << "      std: [" << stats.position_std[0] << ", "
             << stats.position_std[1] << ", " << stats.position_std[2] << "]\n";
        file << "      rms: " << stats.position_rms << "\n";

        file << "    orientation:\n";
        file << "      mean: [" << stats.orientation_mean[0] << ", "
             << stats.orientation_mean[1] << ", " << stats.orientation_mean[2] << ", "
             << stats.orientation_mean[3] << "]\n";
        file << "      std: [" << stats.orientation_std[0] << ", "
             << stats.orientation_std[1] << ", " << stats.orientation_std[2] << ", "
             << stats.orientation_std[3] << "]\n";

        file << "    error:\n";
        file << "      mean: " << stats.error_mean << "\n";
        file << "      std: " << stats.error_std << "\n";
        file << "      max: " << stats.error_max << "\n";

        // Recommended Kalman parameters
        double pos_variance = std::pow(std::max({stats.position_std[0],
                                                  stats.position_std[1],
                                                  stats.position_std[2]}), 2);
        double orient_variance = std::pow(std::max({stats.orientation_std[0],
                                                     stats.orientation_std[1],
                                                     stats.orientation_std[2],
                                                     stats.orientation_std[3]}), 2);

        file << "    recommended_kalman_params:\n";
        file << "      measurement_noise_position: " << pos_variance << "\n";
        file << "      measurement_noise_orientation: " << orient_variance << "\n";
        file << "\n";
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Statistics saved to: %s", params_.output_file.c_str());
}

void NoiseEstimatorNode::shutdown_node()
{
    rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<NoiseEstimatorNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
