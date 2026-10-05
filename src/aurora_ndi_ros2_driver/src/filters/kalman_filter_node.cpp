#include "aurora_ndi_ros2_driver/filters/kalman_filter_node.hpp"

using namespace aurora_ndi_ros2_driver;

KalmanFilterNode::KalmanFilterNode() : Node("kalman_filter_node")
{
    declare_parameters();
    load_parameters();

    if (!validate_parameters()) {
        RCLCPP_FATAL(this->get_logger(), "Parameter validation failed. Shutting down.");
        rclcpp::shutdown();
        return;
    }

    // Initialize Kalman filter noise matrices
    position_filter_.Q = Eigen::Matrix3d::Identity() * params_.process_noise_position;
    position_filter_.R = Eigen::Matrix3d::Identity() * params_.measurement_noise_position;

    orientation_filter_.Q = Eigen::Matrix4d::Identity() * params_.process_noise_orientation;
    orientation_filter_.R = Eigen::Matrix4d::Identity() * params_.measurement_noise_orientation;

    // Create subscriber to input topic
    subscriber_ = this->create_subscription<aurora_ndi_ros2_driver::msg::AuroraData>(
        params_.input_topic,
        10,
        std::bind(&KalmanFilterNode::sensor_callback, this, std::placeholders::_1));

    // Create publisher for filtered data (only if enabled)
    if (params_.enable_kalman_filter) {
        std::string output_topic = params_.input_topic + params_.output_topic_suffix;
        publisher_ = this->create_publisher<aurora_ndi_ros2_driver::msg::AuroraData>(
            output_topic, 10);

        RCLCPP_INFO(this->get_logger(),
                    "Kalman Filter Node initialized\n"
                    "  Input topic:  %s\n"
                    "  Output topic: %s\n"
                    "  Measurement noise - Position: %.6f mm^2\n"
                    "  Measurement noise - Orientation: %.8f\n"
                    "  Process noise - Position: %.6f mm^2\n"
                    "  Process noise - Orientation: %.8f\n"
                    "  Require visible: %s\n"
                    "  Max error threshold: %.2f mm",
                    params_.input_topic.c_str(),
                    output_topic.c_str(),
                    params_.measurement_noise_position,
                    params_.measurement_noise_orientation,
                    params_.process_noise_position,
                    params_.process_noise_orientation,
                    params_.require_visible ? "YES" : "NO",
                    params_.max_error_threshold);
    } else {
        RCLCPP_INFO(this->get_logger(),
                    "Kalman Filter is DISABLED. No filtered data will be published.\n"
                    "  Set 'enable_kalman_filter: true' in config to enable filtering.");
    }
}

void KalmanFilterNode::declare_parameters()
{
    // Topic configuration
    this->declare_parameter("input_topic", "/aurora_data_sensor0");
    this->declare_parameter("output_topic_suffix", "/kalman_filter");
    this->declare_parameter("enable_kalman_filter", true);

    // Kalman filter noise parameters (from calibration)
    this->declare_parameter("measurement_noise_position", 0.00206936);
    this->declare_parameter("measurement_noise_orientation", 7.87569e-07);

    // Process noise (motion model uncertainty)
    // For slow endoscopic movement: position ~0.01 mm^2, orientation ~1e-6
    this->declare_parameter("process_noise_position", 0.01);
    this->declare_parameter("process_noise_orientation", 1.0e-6);

    // Filtering options
    this->declare_parameter("require_visible", true);
    this->declare_parameter("max_error_threshold", 5.0);
}

void KalmanFilterNode::load_parameters()
{
    params_.input_topic = this->get_parameter("input_topic").as_string();
    params_.output_topic_suffix = this->get_parameter("output_topic_suffix").as_string();
    params_.enable_kalman_filter = this->get_parameter("enable_kalman_filter").as_bool();

    params_.measurement_noise_position = this->get_parameter("measurement_noise_position").as_double();
    params_.measurement_noise_orientation = this->get_parameter("measurement_noise_orientation").as_double();
    params_.process_noise_position = this->get_parameter("process_noise_position").as_double();
    params_.process_noise_orientation = this->get_parameter("process_noise_orientation").as_double();

    params_.require_visible = this->get_parameter("require_visible").as_bool();
    params_.max_error_threshold = this->get_parameter("max_error_threshold").as_double();
}

bool KalmanFilterNode::validate_parameters()
{
    bool valid = true;

    if (params_.measurement_noise_position <= 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "measurement_noise_position must be positive (got %.6f)",
                     params_.measurement_noise_position);
        valid = false;
    }

    if (params_.measurement_noise_orientation <= 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "measurement_noise_orientation must be positive (got %.8f)",
                     params_.measurement_noise_orientation);
        valid = false;
    }

    if (params_.process_noise_position <= 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "process_noise_position must be positive (got %.6f)",
                     params_.process_noise_position);
        valid = false;
    }

    if (params_.process_noise_orientation <= 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "process_noise_orientation must be positive (got %.8f)",
                     params_.process_noise_orientation);
        valid = false;
    }

    if (params_.max_error_threshold < 0.0) {
        RCLCPP_ERROR(this->get_logger(),
                     "max_error_threshold must be non-negative (got %.2f)",
                     params_.max_error_threshold);
        valid = false;
    }

    return valid;
}

void KalmanFilterNode::sensor_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg)
{
    // If filtering is disabled, do nothing
    if (!params_.enable_kalman_filter) {
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

    // Apply Kalman filtering
    Eigen::Vector3d filtered_position = predict_and_update_position(measured_position);
    Eigen::Vector4d filtered_orientation = predict_and_update_orientation(measured_orientation);

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

void KalmanFilterNode::initialize_position_filter(const Eigen::Vector3d& initial_position)
{
    position_filter_.x = initial_position;
    position_filter_.P = Eigen::Matrix3d::Identity() * params_.measurement_noise_position;
    position_filter_.initialized = true;

    RCLCPP_INFO(this->get_logger(),
                "Position filter initialized at [%.3f, %.3f, %.3f] mm",
                initial_position(0), initial_position(1), initial_position(2));
}

void KalmanFilterNode::initialize_orientation_filter(const Eigen::Vector4d& initial_orientation)
{
    orientation_filter_.x = normalize_quaternion(initial_orientation);
    orientation_filter_.P = Eigen::Matrix4d::Identity() * params_.measurement_noise_orientation;
    orientation_filter_.initialized = true;

    RCLCPP_INFO(this->get_logger(),
                "Orientation filter initialized at [%.4f, %.4f, %.4f, %.4f]",
                orientation_filter_.x(0), orientation_filter_.x(1),
                orientation_filter_.x(2), orientation_filter_.x(3));
}

Eigen::Vector3d KalmanFilterNode::predict_and_update_position(const Eigen::Vector3d& measurement)
{
    // PREDICT step
    // For stationary/slow-moving model: x_pred = x (no motion model)
    // P_pred = P + Q
    position_filter_.P = position_filter_.P + position_filter_.Q;

    // UPDATE step
    // Kalman gain: K = P_pred * H^T * (H * P_pred * H^T + R)^(-1)
    // For identity observation model (H = I):
    // K = P_pred * (P_pred + R)^(-1)
    Eigen::Matrix3d S = position_filter_.P + position_filter_.R;  // Innovation covariance
    Eigen::Matrix3d K = position_filter_.P * S.inverse();         // Kalman gain

    // State update: x = x_pred + K * (z - H * x_pred)
    // For H = I: x = x_pred + K * (z - x_pred)
    Eigen::Vector3d innovation = measurement - position_filter_.x;
    position_filter_.x = position_filter_.x + K * innovation;

    // Covariance update: P = (I - K * H) * P_pred
    // For H = I: P = (I - K) * P_pred
    position_filter_.P = (Eigen::Matrix3d::Identity() - K) * position_filter_.P;

    return position_filter_.x;
}

Eigen::Vector4d KalmanFilterNode::predict_and_update_orientation(const Eigen::Vector4d& measurement)
{
    // PREDICT step
    // For stationary/slow-moving model: x_pred = x (no motion model)
    // P_pred = P + Q
    orientation_filter_.P = orientation_filter_.P + orientation_filter_.Q;

    // UPDATE step
    // Kalman gain: K = P_pred * (P_pred + R)^(-1)
    Eigen::Matrix4d S = orientation_filter_.P + orientation_filter_.R;  // Innovation covariance
    Eigen::Matrix4d K = orientation_filter_.P * S.inverse();            // Kalman gain

    // Normalize measurement quaternion
    Eigen::Vector4d normalized_measurement = normalize_quaternion(measurement);

    // Handle quaternion double cover: q and -q represent same rotation
    // Choose the one closer to current estimate
    if (orientation_filter_.x.dot(normalized_measurement) < 0.0) {
        normalized_measurement = -normalized_measurement;
    }

    // State update
    Eigen::Vector4d innovation = normalized_measurement - orientation_filter_.x;
    orientation_filter_.x = orientation_filter_.x + K * innovation;

    // Normalize result to keep it a valid quaternion
    orientation_filter_.x = normalize_quaternion(orientation_filter_.x);

    // Covariance update
    orientation_filter_.P = (Eigen::Matrix4d::Identity() - K) * orientation_filter_.P;

    return orientation_filter_.x;
}

Eigen::Vector4d KalmanFilterNode::normalize_quaternion(const Eigen::Vector4d& q)
{
    double norm = q.norm();
    if (norm < 1e-8) {
        // Return identity quaternion if norm is too small
        return Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
    }
    return q / norm;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<KalmanFilterNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Exception in main: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
