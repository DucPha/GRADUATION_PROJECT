#include "widgets/lidar_map_widget.hpp"

#include <QPainter>
#include <QPainterPath>

#include "theme.hpp"

namespace {
// Hằng số phải khớp LidarModule (src/autonomous_vehicle/include/lidar_module.hpp).
// Sao chép thay vì #include vì package GUI không nên phụ thuộc vào header
// của package robot (kéo theo cả OpenCV/serial vào GUI).
constexpr double kPxPerCm = 3.75;
constexpr int kMapW = 600;
constexpr int kMapH = 600;

constexpr int kSectorAngles[] = {60, 120, 240, 300};
}  // namespace

LidarMapWidget::LidarMapWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(240, 240);
    // Nền trắng, khung mảnh: giống panel ảnh bên cạnh.
    setAutoFillBackground(false);
}

void LidarMapWidget::setRangeCm(int cm) {
    range_cm_ = cm > 10 ? cm : 10;
    update();
}

void LidarMapWidget::setNearCm(int cm) {
    near_cm_ = cm > 0 ? cm : 1;
    update();
}

void LidarMapWidget::setPoints(const QVector<QPointF>& far,
                               const QVector<QPointF>& near) {
    far_ = far;
    near_ = near;
    update();
}

void LidarMapWidget::setZoneDistances(double front, double rear,
                                      double left, double right) {
    d_front_ = front;
    d_rear_ = rear;
    d_left_ = left;
    d_right_ = right;
    update();
}

void LidarMapWidget::setHasData(bool has) {
    has_data_ = has;
    update();
}

double LidarMapWidget::cmToPx() const {
    // Bản đồ vuông, lấy cạnh nhỏ hơn. -2 để vòng ngoài cùng không bị cắt.
    const double side = std::min(width(), height()) - 2.0;
    return side / (2.0 * range_cm_);
}

QPointF LidarMapWidget::toWidget(const QPointF& px) const {
    // Bản đồ C++ 600x600 có tâm ở (300,300) và y xuống dưới. Widget dùng
    // cùng quy ước y xuống (không lật) nên chỉ cần tỉ lệ và tâm.
    const double s = cmToPx() / kPxPerCm;
    const QPointF ctr(width() * 0.5, height() * 0.5);
    return QPointF(ctr.x() + (px.x() - kMapW * 0.5) * s,
                   ctr.y() + (px.y() - kMapH * 0.5) * s);
}

void LidarMapWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    p.fillRect(rect(), Theme::plotBg());
    drawGrid(p);
    drawZones(p);
    drawPoints(p);

    // Khung.
    p.setPen(QPen(Theme::borderLine(), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRect(rect().adjusted(0, 0, -1, -1));

    if (!has_data_) drawNoData(p);
}

void LidarMapWidget::drawGrid(QPainter& p) const {
    const QPointF ctr(width() * 0.5, height() * 0.5);
    const double s = cmToPx();

    // Vòng tròn 20cm một bước. Bản C++ vẽ 20/40/60/80cm trong bản đồ 600px
    // tương ứng MAP_H/2/80cm, ở đây vẽ theo bán kính hiển thị.
    const int step = range_cm_ <= 60 ? 10 : (range_cm_ <= 120 ? 20 : 50);
    for (int r = step; r <= range_cm_; r += step) {
        const double rad = r * s;
        p.setPen(QPen(Theme::gridLine(), 1));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(ctr, rad, rad);

        // Nhãn khoảng cách đặt trên đường chéo 45 độ.
        p.setPen(QPen(Theme::textFaint(), 1));
        p.drawText(QPointF(ctr.x() + rad * 0.7071 + 2,
                           ctr.y() - rad * 0.7071 - 2),
                   QString::number(r));
    }

    // Trục chính.
    p.setPen(QPen(Theme::gridLine(), 1));
    p.drawLine(QPointF(ctr.x(), 0), QPointF(ctr.x(), height()));
    p.drawLine(QPointF(0, ctr.y()), QPointF(width(), ctr.y()));

    // Đường phân vùng 60/120/240/300 độ, kéo dài tới mép bản đồ.
    p.setPen(QPen(Theme::axisLine(), 1, Qt::DashLine));
    for (int a : kSectorAngles) {
        const double rad = a * M_PI / 180.0;
        const QPointF end(ctr.x() + range_cm_ * s * std::cos(rad),
                          ctr.y() - range_cm_ * s * std::sin(rad));
        p.drawLine(ctr, end);
    }

    // Nhãn hướng. -90 độ = phía trước (trên bản đồ), 0 = phải.
    p.setPen(QPen(Theme::textMuted(), 1));
    const double m = range_cm_ * s + 12.0;
    struct { const char* t; double deg; } marks[] = {
        {"FRONT", 90}, {"LEFT", 180}, {"REAR", 270}, {"RIGHT", 0}};
    for (const auto& m2 : marks) {
        const double rad = m2.deg * M_PI / 180.0;
        p.drawText(QPointF(ctr.x() + m * std::cos(rad) - 12,
                           ctr.y() - m * std::sin(rad) + 4),
                   m2.t);
    }

    // Mũi xe: tam giác nhỏ ở tâm, chỉ phía trước.
    const double car = std::max(6.0, s * 6.0);
    QPainterPath tri;
    tri.moveTo(ctr.x(), ctr.y() - car);
    tri.lineTo(ctr.x() - car * 0.6, ctr.y() + car * 0.6);
    tri.lineTo(ctr.x() + car * 0.6, ctr.y() + car * 0.6);
    tri.closeSubpath();
    p.setPen(Qt::NoPen);
    p.setBrush(Theme::green());
    p.drawPath(tri);
}

void LidarMapWidget::drawZones(QPainter& p) const {
    if (!has_data_) return;

    const QPointF ctr(width() * 0.5, height() * 0.5);
    const double s = cmToPx();

    // Khoảng cách gần nhất trong 4 vùng, vẽ thành vòng tròn toả sáng quanh
    // xe để thấy ngay "xe đang cách vật cản bao nhiêu" mà không phải đọc số.
    struct Zone { double d; const char* label; };
    const Zone zones[] = {{d_front_, "F"}, {d_rear_, "B"},
                          {d_left_, "L"}, {d_right_, "R"}};

    for (const auto& z : zones) {
        if (z.d < 0) continue;

        // <= near thì đỏ, <= 60 thì hổ phách, xa hơn thì xanh.
        QColor c = Theme::green();
        if (z.d <= near_cm_) c = Theme::red();
        else if (z.d <= 60.0) c = Theme::amber();

        p.setPen(QPen(c, 2));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(ctr, z.d * s, z.d * s);

        // Chữ nhãn ở đầu hướng tương ứng.
        double deg = 90;
        if (QString(z.label) == "R") deg = 0;
        else if (QString(z.label) == "B") deg = 270;
        else if (QString(z.label) == "L") deg = 180;
        const double rad = deg * M_PI / 180.0;
        const double lr = z.d * s;
        const QPointF tp(ctr.x() + lr * std::cos(rad),
                         ctr.y() - lr * std::sin(rad));

        const QString txt = z.d >= 199.5
            ? QStringLiteral(">200")
            : QString::number(static_cast<int>(std::lround(z.d)));
        const QRectF box(tp.x() - 24, tp.y() - 10, 48, 20);
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(255, 255, 255, 235));
        p.drawRoundedRect(box, 3, 3);
        p.setPen(c);
        p.drawText(box, Qt::AlignCenter, QString(z.label) + "  " + txt);
    }
}

void LidarMapWidget::drawPoints(QPainter& p) const {
    if (!has_data_) return;

    // Vẽ trùng lên nhau phải chồng điểm, không dùng drawPoint từng cái vì
    // chậm hơn nhiều khi có vài nghìn điểm.
    p.setPen(Qt::NoPen);

    if (!far_.isEmpty()) {
        QPainterPath path;
        path.setFillRule(Qt::WindingFill);
        for (const QPointF& pt : far_) {
            const QPointF w = toWidget(pt);
            path.addEllipse(w, 2.0, 2.0);
        }
        p.setBrush(Theme::accent());
        p.drawPath(path);
    }

    if (!near_.isEmpty()) {
        QPainterPath path;
        for (const QPointF& pt : near_) {
            const QPointF w = toWidget(pt);
            path.addEllipse(w, 2.8, 2.8);
        }
        p.setBrush(Theme::red());
        p.drawPath(path);
    }
}

void LidarMapWidget::drawNoData(QPainter& p) const {
    const QRectF box(0, height() * 0.5 - 20, width(), 40);
    p.setPen(QPen(Theme::red(), 2));
    p.setBrush(Qt::NoBrush);
    p.drawText(box, Qt::AlignCenter, QStringLiteral("NO LIDAR DATA"));
}