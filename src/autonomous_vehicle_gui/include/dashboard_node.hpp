#pragma once

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"

class DashboardWindow;

// ============================================================================
// NODE DASHBOARD
//
// Chạy cạnh Qt event loop: rclcpp::spin_some hoặc MultiThreadedExecutor
// kết hợp với QThread. Ở đây dùng std::thread chạy executor.
// ============================================================================
class DashboardNode : public rclcpp::Node {
public:
    explicit DashboardNode(const std::string& name = "autonomous_vehicle_gui_node");
    ~DashboardNode() override;

    void setWindow(DashboardWindow* w);
    void start();
    void stop();

    bool demo() const { return demo_; }

private:
    void onStatus(const std_msgs::msg::String::SharedPtr msg);
    void onRaw(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void onVis(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void onBin(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void onRoi(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    DashboardWindow* win_;
    bool demo_;
    int domainId_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subStatus_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subRaw_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subVis_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subBin_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subRoi_;

    std::unique_ptr<std::thread> spinThread_;
};