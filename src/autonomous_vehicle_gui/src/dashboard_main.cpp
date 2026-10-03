#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "dashboard_node.hpp"
#include "dashboard_window.hpp"

int main(int argc, char** argv) {
    // Phải init TRƯỚC khi tạo Node: constructor của rclcpp::Node nạp context
    // toàn cục và sẽ ném exception nếu context chưa có. Bản cũ thiếu dòng này
    // nên GUI chết ngay khi khởi động, không hiện cửa sổ.
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);
    app.setApplicationName("autonomous_vehicle_gui");

    auto node = std::make_shared<DashboardNode>();
    auto win = new DashboardWindow(node.get());
    node->setWindow(win);
    node->start();
    win->showMaximized();
    const int ret = app.exec();

    // stop() đã gọi rclcpp::shutdown(); guard bằng rclcpp::ok() cho lần gọi
    // thứ hai từ destructor của node.
    node->stop();
    if (rclcpp::ok()) rclcpp::shutdown();
    return ret;
}