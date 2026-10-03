#pragma once

#include <QElapsedTimer>
#include <QImage>
#include <QMainWindow>
#include <QString>
#include <QTimer>
#include <QTime>
#include <QVector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/string.hpp"

#include "widgets/header_bar.hpp"
#include "widgets/lidar_map_widget.hpp"
#include "widgets/image_panel.hpp"
#include "widgets/trend_plot.hpp"
#include "widgets/node_link_table.hpp"
#include "widgets/param_panel.hpp"

class DashboardNode;

// ============================================================================
// CỬA SỔ CHÍNH - LAYOUT ĐÚNG THEO MATLAB
//
// Bố cục: header (5.5%), LiDAR map (34-90%?), 4 panel ảnh, 2 plot, table,
// 6 panel phải. Dùng QGridLayout để giữ tỷ lệ.
// ============================================================================
class DashboardWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit DashboardWindow(DashboardNode* node, QWidget* parent = nullptr);
    ~DashboardWindow() override = default;

    void setDemoMode(bool demo);

    // Chỉ gọi từ GUI thread (onTick). Dữ liệu lấy từ hàng đợi trung gian
    // của DashboardNode, không phải từ callback ROS.
    void updateStatus(const QString& json);
    void updateRaw(const QImage& img);
    void updateVis(const QImage& img);
    void updateBin(const QImage& img);
    void updateRoi(const QImage& img);
    void updateLink(int ok3);

private slots:
    void onTick();
    void onTextTick();
    void onKeyEsc();

private:
    DashboardNode* node_;
    QTimer* uiTimer_;    // 30 Hz
    QTimer* textTimer_;  // 8 Hz
    QTime startTime_;
    bool demo_;

    HeaderBar* header_;
    LidarMapWidget* lidar_;
    ImagePanel* imgRaw_;
    ImagePanel* imgVis_;
    ImagePanel* imgRoi_;
    ImagePanel* imgBin_;
    TrendPlot* pidPlot_;
    TrendPlot* steerPlot_;
    NodeLinkTable* linkTable_;

    ParamPanel* pLidar_;
    ParamPanel* pLane_;
    ParamPanel* pOa_;
    ParamPanel* pEsp_;
    ParamPanel* pAi_;
    ParamPanel* pSys_;

    double tLast_ = 0.0;
    // Giây kể từ lúc bật của mẫu status gần nhất, dùng để tính khoảng cách
    // thời gian cho trend plot. -1 = chưa có mẫu nào.
    double last_sample_s_ = -1.0;
    QElapsedTimer elapsed_;
};