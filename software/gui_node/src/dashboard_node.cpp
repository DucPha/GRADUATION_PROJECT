#include "dashboard_node.hpp"

#include <QByteArray>
#include <QImage>

#include <thread>
#include <utility>

#include "dashboard_window.hpp"

// Giải mã JPEG thành QImage. QImage là kiểu CPU nên an toàn khi tạo ở
// bất kỳ thread nào - khác với QPixmap.
static QImage decodeJpeg(const sensor_msgs::msg::CompressedImage::SharedPtr& msg) {
    QByteArray bytes(reinterpret_cast<const char*>(msg->data.data()),
                     static_cast<int>(msg->data.size()));
    QImage img;
    if (!img.loadFromData(bytes, "JPEG")) return QImage();
    return img;
}

DashboardNode::DashboardNode(const std::string& name)
    : rclcpp::Node(name),
      win_(nullptr),
      demo_(false),
      domainId_(0) {
    declare_parameter("demo", false);
    declare_parameter("domain_id", 0);
    demo_ = get_parameter("demo").as_bool();
    domainId_ = get_parameter("domain_id").as_int();
}

DashboardNode::~DashboardNode() {
    stop();
}

void DashboardNode::setWindow(DashboardWindow* w) {
    win_ = w;
    if (win_) win_->setDemoMode(demo_);
}

void DashboardNode::start() {
    // Subscribe khi không demo.
    if (!demo_) {
        subStatus_ = create_subscription<std_msgs::msg::String>(
            "/autocar/dbg/status", 10,
            std::bind(&DashboardNode::onStatus, this, std::placeholders::_1));
        subRaw_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/autocar/dbg/cam_raw/compressed", 1,
            std::bind(&DashboardNode::onRaw, this, std::placeholders::_1));
        subVis_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/autocar/dbg/lane_vis/compressed", 1,
            std::bind(&DashboardNode::onVis, this, std::placeholders::_1));
        subBin_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/autocar/dbg/lane_bin/compressed", 1,
            std::bind(&DashboardNode::onBin, this, std::placeholders::_1));
        subRoi_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/autocar/dbg/lane_roi/compressed", 1,
            std::bind(&DashboardNode::onRoi, this, std::placeholders::_1));
    }

    // Spin ở thread riêng để không chặn Qt.
    spinThread_ = std::make_unique<std::thread>([this]() {
        rclcpp::spin(this->get_node_base_interface());
    });
}

void DashboardNode::stop() {
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    if (spinThread_ && spinThread_->joinable()) {
        spinThread_->join();
    }
    spinThread_.reset();
}

// ============================================================================
// PUSH (spin thread) - chỉ đẩy vào hàng đợi, KHÔNG chạm Qt widget
// ============================================================================

void DashboardNode::pushImage(int channel, const QImage& img) {
    if (channel < 0 || channel >= CH_COUNT || img.isNull()) return;
    std::lock_guard<std::mutex> lock(q_mtx_);
    q_img_[channel] = img;
    q_img_valid_[channel] = true;
}

void DashboardNode::onStatus(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(q_mtx_);
    q_status_ = msg->data;
    q_status_valid_ = true;
}

void DashboardNode::onRaw(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    pushImage(CH_RAW, decodeJpeg(msg));
}

void DashboardNode::onVis(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    pushImage(CH_VIS, decodeJpeg(msg));
}

void DashboardNode::onBin(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    pushImage(CH_BIN, decodeJpeg(msg));
}

void DashboardNode::onRoi(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    pushImage(CH_ROI, decodeJpeg(msg));
}

// ============================================================================
// TAKE (GUI thread) - kéo dữ liệu mới nhất ra
// ============================================================================

bool DashboardNode::takeStatus(std::string& out) {
    std::lock_guard<std::mutex> lock(q_mtx_);
    if (!q_status_valid_) return false;
    out = q_status_;
    q_status_valid_ = false;
    return true;
}

bool DashboardNode::takeImage(QImage& out, int channel) {
    if (channel < 0 || channel >= CH_COUNT) return false;
    std::lock_guard<std::mutex> lock(q_mtx_);
    if (!q_img_valid_[channel]) return false;
    out = q_img_[channel];
    q_img_valid_[channel] = false;
    return true;
}