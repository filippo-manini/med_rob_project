#ifndef AURORA_NDI_ROS2_DRIVER__KALMAN_FILTER_NODE_HPP_
#define AURORA_NDI_ROS2_DRIVER__KALMAN_FILTER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include "aurora_ndi_ros2_driver/msg/aurora_data.hpp"

namespace aurora_ndi_ros2_driver
{

class KalmanFilterNode : public rclcpp::Node
{
public:
    KalmanFilterNode();
    ~KalmanFilterNode() = default;

private:
    // Configuration parameters
    struct Parameters
    {
        std::string input_topic;
        std::string output_topic_suffix;
        bool enable_kalman_filter;

        // Kalman filter noise parameters
        double measurement_noise_position;      // R matrix - position variance (mm^2)
        double measurement_noise_orientation;   // R matrix - orientation variance
        double process_noise_position;          // Q matrix - position variance (mm^2)
        double process_noise_orientation;       // Q matrix - orientation variance

        // Filtering options
        bool require_visible;
        double max_error_threshold;
    };

    // Kalman filter state for position (3D)
    struct PositionKalmanState
    {
        Eigen::Vector3d x;      // State estimate (position)
        Eigen::Matrix3d P;      // Error covariance
        Eigen::Matrix3d Q;      // Process noise covariance
        Eigen::Matrix3d R;      // Measurement noise covariance
        bool initialized;

        PositionKalmanState() : initialized(false)
        {
            x.setZero();
            P.setIdentity();
            Q.setIdentity();
            R.setIdentity();
        }
    };

    // Kalman filter state for orientation (quaternion - 4D)
    struct OrientationKalmanState
    {
        Eigen::Vector4d x;      // State estimate (quaternion: x, y, z, w)
        Eigen::Matrix4d P;      // Error covariance
        Eigen::Matrix4d Q;      // Process noise covariance
        Eigen::Matrix4d R;      // Measurement noise covariance
        bool initialized;

        OrientationKalmanState() : initialized(false)
        {
            x.setZero();
            x(3) = 1.0;  // Initialize w=1 for identity quaternion
            P.setIdentity();
            Q.setIdentity();
            R.setIdentity();
        }
    };

    // ROS interfaces
    rclcpp::Subscription<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr subscriber_;
    rclcpp::Publisher<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr publisher_;

    // Kalman filter states
    PositionKalmanState position_filter_;
    OrientationKalmanState orientation_filter_;

    // Parameters
    Parameters params_;

    // Methods
    void declare_parameters();
    void load_parameters();
    bool validate_parameters();
    void sensor_callback(const aurora_ndi_ros2_driver::msg::AuroraData::SharedPtr msg);

    // Kalman filter functions
    void initialize_position_filter(const Eigen::Vector3d& initial_position);
    void initialize_orientation_filter(const Eigen::Vector4d& initial_orientation);
    Eigen::Vector3d predict_and_update_position(const Eigen::Vector3d& measurement);
    Eigen::Vector4d predict_and_update_orientation(const Eigen::Vector4d& measurement);
    Eigen::Vector4d normalize_quaternion(const Eigen::Vector4d& q);
};

} // namespace aurora_ndi_ros2_driver

#endif // AURORA_NDI_ROS2_DRIVER__KALMAN_FILTER_NODE_HPP_
