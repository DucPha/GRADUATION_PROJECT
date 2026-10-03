#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include <QImage>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"

class DashboardWindow;

// ============================================================================
// NODE DASHBOARD
//
// Chạy cạnh Qt event loop: executor nằm ở std::thread riêng, còn GUI chạy ở
// main thread. Vì vậy callback KHÔNG được chạm trực tiếp vào đối tượng Qt.
//
// Luồng an toàn:
//   1. callback ROS (spin thread): decode JPEG -> QImage (CPU, an toàn đa
//      luồng) -> đẩy vào hàng đợi khóa mutex, chỉ giữ frame MỚI NHẤT.
//   2. timer 30 Hz của DashboardWindow trên GUI thread: kéo hết hàng đợi ra,
//      gọi updateX() -> ImagePanel tạo QPixmap.
//
// Bản cũ decode thẳng ra QPixmap rồi gọi updateRaw() ngay trong callback:
// QPixmap sống trên GPU, chạm từ thread không phải GUI thread là lỗi
// undefined behaviour -> crash ngẫu nhiên khi ảnh về nhanh.
// ============================================================================
class DashboardNode : public rclcpp::Node {
public:
    explicit DashboardNode(const std::string& name = "autonomous_vehicle_gui_node");
    ~DashboardNode() override;

    void setWindow(DashboardWindow* w);
    void start();
    void stop();

    bool demo() const { return demo_; }

    // ==== Hàng đợi trung gian, gọi từ GUI thread trong onTick() ====

    // Trả false nếu không có gì mới.
    bool takeStatus(std::string& out);
    bool takeImage(QImage& out, int channel);

    enum Channel { CH_RAW = 0, CH_VIS = 1, CH_BIN = 2, CH_ROI = 3, CH_COUNT = 4 };

private:
    void onStatus(const std_msgs::msg::String::SharedPtr msg);
    void onRaw(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void onVis(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void onBin(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void onRoi(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    void pushImage(int channel, const QImage& img);

    DashboardWindow* win_;
    bool demo_;
    int domainId_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subStatus_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subRaw_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subVis_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subBin_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subRoi_;

    std::unique_ptr<std::thread> spinThread_;

    // Chỉ giữ frame MỚI NHẤT cho mỗi kênh: ảnh về nhanh hơn GUI vẽ thì hàng
    // đợi không được phép tích luỹ, nếu không bộ nhớ phình theo thời gian và
    // GUI càng lúc càng lag.
    std::mutex q_mtx_;
    std::string q_status_;
    bool q_status_valid_ = false;
    QImage q_img_[CH_COUNT];
    bool q_img_valid_[CH_COUNT] = {false, false, false, false};
};