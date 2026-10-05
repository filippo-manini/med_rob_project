#include "aurora_ndi_ros2_driver/filters/lowpass_filter_node.hpp"
#include <cmath>

using namespace aurora_ndi_ros2_driver;

LowpassFilterNode::LowpassFilterNode() : Node("lowpass_filter_node")
{
    declare_parameters();
    load_parameters();

    if (!validate_parameters()) {
        RCLCPP_FATAL(this->get_logger(), "Parameter validation failed. Shutting down.");
        rclcpp::shutdown();
        return;
    }

    // Compute filter coefficient alpha
    position_filter_.alpha = compute_alpha(params_.cutoff_frequency_hz, params_.sample_rate_hz);
    orientation_filter_.alpha = position_filter_.alpha;  // Same alpha for both

    // Create subscriber to input topic
    subscriber_ = this->create_subscription<aurora_ndi_ros2_driver::msg::AuroraData>(
        params_.input_topic,
        10,
        std::bind(&LowpassFilterNode::sensor_callback, this, std::placeholders::_1));

    // Create publisher for filtered data (only if enabled)
    if (params_.enable_lowpass_filter) {
        std::string output_topic = params_.input_topic + params_.output_topic_suffix;
        publisher_ = this->create_publisher<aurora_ndi_ros2_driver::msg::AuroraData>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(),
                    "Low-pass Filter Node initialized\n"
                    "  Input topic:  %s\n"
                    "  Output topic: %s\n"
                    "  Cutoff frequency: %.2f Hz\n"
                    "  Sample rate: %.2f Hz\n"
                    "  Filter alpha: %.4f (0=max smoothing, 1=no smoothing)\n"
                    "  Require visible: %s\n"
                    "  Max error threshold: %.2f mm",
                    params_.input_topic.c_str(),
                    output_topic.c_str(),
                    params_.cutoff_frequency_hz,
                    params_.sample_rate_hz,
                    position_filter_.alpha,
                    params_.require_visible ? "YES" : "NO",
                    params_.max_error_threshold);
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "Low-pass Filter is DISABLED. No filtered data will be published.\n"
                    "  Set 'enable_lowpass_filter: true' in config to enable filtering.");
    }
}

void LowpassFilterNode::declare_parameters()
{
    // Topic configuration
    this->declare_parameter("input_topic", "/aurora_data_sensor0");
    this->declare_parameter("output_topic_suffix", "/lowpass_filter");
    this->declare_parameter("enable_lowpass_filter", true);

    // Low-pass filter parameters
    // For 40 Hz sample rate, cutoff of 5 Hz gives good smoothing for slow movements
    this->declare_parameter("cutoff_frequency_hz", 5.0);
    this->declare_parameter("sample_rate_hz", 40.0);

    // Filtering options
    this->declare_parameter("require_visible", true);
    this->declare_parameter("max_error_threshold", 5.0);
}

void LowpassFilterNode::load_parameters()
{
    params_.input_topic = this->get_parameter("input_topic").as_string();
    params_.output_topic_suffix = this->get_parameter("output_topic_suffix").as_string();
    params_.enable_lowpass_filter = this->get_parameter("enable_lowpass_filter").as_bool();

    params_.cutoff_frequency_hz = this->get_parameter("cutoff_frequency_hz").as_double();
    params_.sample_rate_hz = this->get_parameter("sample_rate_hz").as_double();

    params_.require_visible = this->get_parameter("require_visible").as_bool();
    params_.max_error_threshold = this->get_parameter("max_error_threshold").as_double();
}

bool LowpassFilterNode::validate_parameters()
{
    bool valid = true;

    if (params_.cutoff_frequency_hz <= 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "cutoff_frequency_hz must be positive (got %.2f)",
                     params_.cutoff_frequency_hz);
        valid = false;
    }

    if (params_.sample_rate_hz <= 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "sample_rate_hz must be positive (got %.2f)",
                     params_.sample_rate_hz);
        valid = false;
    }

    if (params_.cutoff_frequency_hz >= params_.sample_rate_hz / 2.0) {
        RCLCPP_WARN(this->get_logger(),
                    "cutoff_frequency_hz (%.2f Hz) should be less than Nyquist frequency (%.2f Hz)",
                    params_.cutoff_frequency_hz, params_.sample_rate_hz / 2.0);
    }

    if (params_.max_error_threshold < 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "max_error_threshold must be non-negative (got %.2f)",
                     params_.max_error_threshold);
        valid = false;
    }

    return valid;
}

void LowpassFilterNode::sensor_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg)
{
    // If filtering is disabled, do nothing
    if (!params_.enable_lowpass_filter) {
        return;
    }

    // Apply data quality filters
    if (params_.require_visible && !msg->visible) {
        return;
    }

    if (params_.max_error_threshold > 0.0 && msg->error > params_.max_error_threshold) {
        return;
    }

    // Convert ROS message to Eigen vectors
    Eigen::Vector3d measured_position(msg->position.x, msg->position.y, msg->position.z);
    Eigen::Vector4d measured_orientation(msg->orientation.x, msg->orientation.y,
                                         msg->orientation.z, msg->orientation.w);

    // Initialize filters on first valid measurement
    if (!position_filter_.initialized) {
        initialize_position_filter(measured_position);
    }

    if (!orientation_filter_.initialized) {
        initialize_orientation_filter(measured_orientation);
    }

    // Apply low-pass filtering
    Eigen::Vector3d filtered_position = apply_lowpass_position(measured_position);
    Eigen::Vector4d filtered_orientation = apply_lowpass_orientation(measured_orientation);

    // Publish filtered data with SAME timestamp as input
    auto filtered_msg = std::make_shared<aurora_ndi_ros2_driver::msg::AuroraData>();
    filtered_msg->header = msg->header;  // Keep same timestamp and frame_id
    filtered_msg->position.x = filtered_position(0);
    filtered_msg->position.y = filtered_position(1);
    filtered_msg->position.z = filtered_position(2);
    filtered_msg->orientation.x = filtered_orientation(0);
    filtered_msg->orientation.y = filtered_orientation(1);
    filtered_msg->orientation.z = filtered_orientation(2);
    filtered_msg->orientation.w = filtered_orientation(3);
    filtered_msg->error = msg->error;
    filtered_msg->visible = msg->visible;
    filtered_msg->port_handle = msg->port_handle;

    publisher_->publish(*filtered_msg);
}

void LowpassFilterNode::initialize_position_filter(const Eigen::Vector3d& initial_position)
{
    position_filter_.filtered_value = initial_position;
    position_filter_.initialized = true;

    RCLCPP_INFO(this->get_logger(),
                "Position filter initialized at [%.3f, %.3f, %.3f] mm",
                initial_position(0), initial_position(1), initial_position(2));
}

void LowpassFilterNode::initialize_orientation_filter(const Eigen::Vector4d& initial_orientation)
{
    orientation_filter_.filtered_value = normalize_quaternion(initial_orientation);
    orientation_filter_.initialized = true;

    RCLCPP_INFO(this->get_logger(),
                "Orientation filter initialized at [%.4f, %.4f, %.4f, %.4f]",
                orientation_filter_.filtered_value(0), orientation_filter_.filtered_value(1),
                orientation_filter_.filtered_value(2), orientation_filter_.filtered_value(3));
}

Eigen::Vector3d LowpassFilterNode::apply_lowpass_position(const Eigen::Vector3d& measurement)
{
    // Low-pass filter formula: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    // where:
    //   y[n] = current filtered output
    //   x[n] = current measurement
    //   y[n-1] = previous filtered output
    //   alpha = filter coefficient (higher = less filtering)

    position_filter_.filtered_value =
        position_filter_.alpha * measurement +
        (1.0 - position_filter_.alpha) * position_filter_.filtered_value;

    return position_filter_.filtered_value;
}

Eigen::Vector4d LowpassFilterNode::apply_lowpass_orientation(const Eigen::Vector4d& measurement)
{
    // Normalize measurement quaternion
    Eigen::Vector4d normalized_measurement = normalize_quaternion(measurement);

    // Handle quaternion double cover: q and -q represent same rotation
    // Choose the one closer to current filtered value
    if (orientation_filter_.filtered_value.dot(normalized_measurement) < 0.0) {
        normalized_measurement = -normalized_measurement;
    }

    // Apply low-pass filter
    orientation_filter_.filtered_value =
        orientation_filter_.alpha * normalized_measurement +
        (1.0 - orientation_filter_.alpha) * orientation_filter_.filtered_value;

    // Normalize result to keep it a valid quaternion
    orientation_filter_.filtered_value = normalize_quaternion(orientation_filter_.filtered_value);

    return orientation_filter_.filtered_value;
}

Eigen::Vector4d LowpassFilterNode::normalize_quaternion(const Eigen::Vector4d& q)
{
    double norm = q.norm();
    if (norm < 1e-8) {
        // Return identity quaternion if norm is too small
        return Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
    }
    return q / norm;
}

double LowpassFilterNode::compute_alpha(double cutoff_freq_hz, double sample_rate_hz)
{
    // Compute alpha using the standard first-order low-pass filter formula
    // alpha = dt / (RC + dt)
    // where RC = 1 / (2 * pi * cutoff_freq)
    // and dt = 1 / sample_rate

    double dt = 1.0 / sample_rate_hz;
    double RC = 1.0 / (2.0 * M_PI * cutoff_freq_hz);
    double alpha = dt / (RC + dt);

    return alpha;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<LowpassFilterNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
