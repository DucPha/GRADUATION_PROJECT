#include "dashboard_node.hpp"

#include <chrono>
#include <thread>

#include "dashboard_window.hpp"

using namespace std::chrono_literals;

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

void DashboardNode::onStatus(const std_msgs::msg::String::SharedPtr msg) {
    if (win_) win_->updateStatus(QString::fromStdString(msg->data));
}

void DashboardNode::onRaw(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    QPixmap pm;
    if (pm.loadFromData(reinterpret_cast<const uchar*>(msg->data.data()),
                        static_cast<uint>(msg->data.size()), "jpg")) {
        if (win_) win_->updateRaw(pm);
    }
}

void DashboardNode::onVis(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    QPixmap pm;
    if (pm.loadFromData(reinterpret_cast<const uchar*>(msg->data.data()),
                        static_cast<uint>(msg->data.size()), "jpg")) {
        if (win_) win_->updateVis(pm);
    }
}

void DashboardNode::onBin(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    QPixmap pm;
    if (pm.loadFromData(reinterpret_cast<const uchar*>(msg->data.data()),
                        static_cast<uint>(msg->data.size()), "jpg")) {
        if (win_) win_->updateBin(pm);
    }
}

void DashboardNode::onRoi(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    QPixmap pm;
    if (pm.loadFromData(reinterpret_cast<const uchar*>(msg->data.data()),
                        static_cast<uint>(msg->data.size()), "jpg")) {
        if (win_) win_->updateRoi(pm);
    }
}