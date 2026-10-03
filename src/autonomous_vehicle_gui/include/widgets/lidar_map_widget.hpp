#pragma once

#include <QPainter>
#include <QPointF>
#include <QString>
#include <QVector>
#include <QWidget>

// ============================================================================
// BẢN ĐỒ LIDAR
//
// Vẽ lại đúng hệ toạ độ mà LidarModule::update() dùng ở C++:
//   points_px đã là pixel trong bản đồ 600x600, tâm xe ở chính giữa,
//   3.75 px/cm, trục y ngược (y tăng lên là xa hơn phía trước).
//
// Vì vậy widget KHÔNG tự tính lại góc và khoảng cách từ LaserScan mà đọc
// thẳng points_px. Nếu tự tính lại, một sai lệch quy ước (ví dụ chiều cao
// frame thay đổi) sẽ làm bản đồ lệch mà không ai nhận ra, trong khi bản đồ
// C++ vẫn đúng.
// ============================================================================
class LidarMapWidget : public QWidget {
    Q_OBJECT

public:
    explicit LidarMapWidget(QWidget* parent = nullptr);

    // Bán kính hiển thị [cm]. Mặc định 100 khớp bán kính vẽ trong C++.
    void setRangeCm(int cm);

    // Ngưỡng đổi màu: dưới ngưỡng này là vật cản gần [cm].
    void setNearCm(int cm);

    // Cập nhật điểm. near dùng để tô đỏ, far dùng nền.
    void setPoints(const QVector<QPointF>& far, const QVector<QPointF>& near);

    // Khoảng cách tối thiểu theo 4 vùng [cm]. Dùng -1 để hiển thị "---".
    void setZoneDistances(double front, double rear, double left, double right);

    void setHasData(bool has);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    // cm -> pixel widget. Tâm xe luôn ở giữa widget, giống bản đồ 600x600.
    QPointF toWidget(const QPointF& px) const;

    double cmToPx() const;

    void drawGrid(QPainter& p) const;
    void drawZones(QPainter& p) const;
    void drawPoints(QPainter& p) const;
    void drawNoData(QPainter& p) const;

    int range_cm_ = 100;
    int near_cm_ = 30;

    QVector<QPointF> far_;
    QVector<QPointF> near_;

    double d_front_ = -1.0;
    double d_rear_ = -1.0;
    double d_left_ = -1.0;
    double d_right_ = -1.0;

    bool has_data_ = false;
};