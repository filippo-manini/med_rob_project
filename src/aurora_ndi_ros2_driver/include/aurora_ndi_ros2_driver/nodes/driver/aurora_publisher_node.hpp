#ifndef AURORA_PUBLISHER_NODE_HPP
#define AURORA_PUBLISHER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <aurora_ndi_ros2_driver/msg/aurora_data.hpp>
#include <aurora_ndi_ros2_driver/filters/aurora_data_filter.hpp>

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <optional>

namespace AuroraDriver {
    class ndi_aurora;
    struct ToolPose;
}

namespace aurora_ndi_ros2_driver
{

class AuroraPublisherNode : public rclcpp::Node
{
public:
    AuroraPublisherNode();
    ~AuroraPublisherNode();

private:
    struct Parameters {
        std::string serial_port;
        int baud_rate;
        double serial_timeout_sec;

        int num_sensors;
        std::vector<std::string> tool_rom_files;
        std::vector<std::string> port_handles;
        std::vector<std::string> port_handles_autoconfig;
        std::vector<std::string> topic_names;
        std::vector<std::string> child_frame_names;

        std::string reference_port;
        int track_reply_option;
        std::string measure_reply_option;
        bool status_reply;

        std::string frame_id;
        double publish_rate_hz;
        int queue_size;

        bool enable_data_filtering;
        int filter_window_size;
        double position_scale_factor;
        double orientation_scale_factor;
        double error_scale_factor;

        double command_timeout_sec;
        int max_connection_retries;
        double retry_delay_sec;

        bool log_raw_data;
    };

    struct AuroraData {
        rclcpp::Time ros_timestamp;
        std::string handle;
        int sensor_index{-1};
        bool visible{false};
        std::array<double, 3> position{{0.0, 0.0, 0.0}};
        std::array<double, 4> orientation{{0.0, 0.0, 0.0, 1.0}};
        double error{0.0};

        AuroraData() : ros_timestamp(0, 0, RCL_ROS_TIME) {}
    };

    void declare_parameters();
    void load_parameters();
    bool validate_parameters();

    bool setup_aurora();

    std::vector<std::optional<AuroraData>> parse_aurora_data_multi(
        const std::vector<AuroraDriver::ToolPose>& poses,
        const std::vector<int>& port_indexes,
        const std::vector<int>& visible_tools,
        const rclcpp::Time& measurement_timestamp);

    void read_thread_function();

    void tf_publish_callback();
    void aurora_data_publish_callback();

    void shutdown();

    Parameters params_;

    std::unique_ptr<AuroraDriver::ndi_aurora> aurora_driver_;

    std::vector<rclcpp::Publisher<aurora_ndi_ros2_driver::msg::AuroraData>::SharedPtr> aurora_data_publishers_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr tf_timer_;
    rclcpp::TimerBase::SharedPtr aurora_data_timer_;

    std::thread read_thread_;
    std::atomic<bool> tracking_active_;
    std::mutex data_mutex_;

    std::vector<AuroraData> latest_data_;
    std::vector<std::unique_ptr<AuroraDataFilter>> data_filters_;
};

} // namespace aurora_ndi_ros2_driver

#endif // AURORA_PUBLISHER_NODE_HPP