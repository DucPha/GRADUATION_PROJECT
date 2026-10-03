#include <QApplication>

#include "dashboard_node.hpp"
#include "dashboard_window.hpp"

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setApplicationName("autonomous_vehicle_gui");

    auto node = std::make_shared<DashboardNode>();
    auto win = new DashboardWindow(node.get());
    node->setWindow(win);
    node->start();
    win->showMaximized();
    const int ret = app.exec();
    node->stop();
    return ret;
}