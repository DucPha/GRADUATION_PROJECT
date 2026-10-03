#pragma once

#include <QElapsedTimer>
#include <QMainWindow>
#include <QPixmap>
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
    void updateStatus(const QString& json);
    void updateRaw(const QPixmap& pm);
    void updateVis(const QPixmap& pm);
    void updateBin(const QPixmap& pm);
    void updateRoi(const QPixmap& pm);
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
    QElapsedTimer elapsed_;
};