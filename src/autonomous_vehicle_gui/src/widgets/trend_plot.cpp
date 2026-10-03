#include "widgets/trend_plot.hpp"

#include <QPainter>
#include <algorithm>

#include "theme.hpp"

namespace {
constexpr int kLeft = 42;
constexpr int kRight = 16;
constexpr int kTop = 24;
constexpr int kBottom = 20;
}  // namespace

TrendPlot::TrendPlot(const QString& title,
                     const QString& yLabel,
                     double yMin, double yMax,
                     bool centerZero,
                     QWidget* parent)
    : QWidget(parent),
      title_(title),
      yLabel_(yLabel),
      yMin_(yMin),
      yMax_(yMax),
      centerZero_(centerZero),
      l1_("L1"), l2_("L2"),
      yMinAuto_(yMin), yMaxAuto_(yMax) {
    setMinimumSize(200, 80);
}

void TrendPlot::addPoint(double tRel, double v1, double v2) {
    t_.push_back(tRel);
    v1_.push_back(v1);
    v2_.push_back(v2);

    // Giữ buffer (MATLAB dùng 300). Nếu tăng lên quá lớn trên NUC cũng
    // không sao, nhưng 300 điểm mỗi frame chỉ repaint, không rebuild.
    const int maxn = 500;
    if (t_.size() > maxn) {
        t_.remove(0, t_.size() - maxn);
        v1_.remove(0, v1_.size() - maxn);
        v2_.remove(0, v2_.size() - maxn);
    }

    // Autoscale. Thêm 10% đệm để đường không dính mép.
    if (!v1_.isEmpty() && !std::isnan(v1_.last()) && !std::isinf(v1_.last())) {
        const double lv1 = v1_.last();
        yMinAuto_ = std::min(yMinAuto_, lv1);
        yMaxAuto_ = std::max(yMaxAuto_, lv1);
    }
    if (!v2_.isEmpty() && !std::isnan(v2_.last()) && !std::isinf(v2_.last())) {
        const double lv2 = v2_.last();
        yMinAuto_ = std::min(yMinAuto_, lv2);
        yMaxAuto_ = std::max(yMaxAuto_, lv2);
    }
    const double range = std::max(1e-3, yMaxAuto_ - yMinAuto_);
    yMinAuto_ -= range * 0.1;
    yMaxAuto_ += range * 0.1;

    update();
}

void TrendPlot::clear() {
    t_.clear();
    v1_.clear();
    v2_.clear();
    yMinAuto_ = yMin_;
    yMaxAuto_ = yMax_;
    update();
}

void TrendPlot::setYRange(double yMin, double yMax) {
    yMin_ = yMin; yMax_ = yMax;
    yMinAuto_ = yMin; yMaxAuto_ = yMax;
    update();
}

void TrendPlot::setLegend(const QString& l1, const QString& l2) {
    l1_ = l1; l2_ = l2;
    update();
}

void TrendPlot::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.fillRect(rect(), Theme::panelBg());
    drawTitle(p);
    drawGrid(p);
    drawSeries(p);
}

void TrendPlot::drawTitle(QPainter& p) const {
    p.setPen(Theme::textPrimary());
    QFont f;
    f.setBold(true);
    f.setPointSize(9);
    p.setFont(f);
    p.drawText(QRect(4, 2, width()-8, kTop-4),
               Qt::AlignVCenter, title_);
}

void TrendPlot::drawGrid(QPainter& p) const {
    const QRect plot(kLeft, kTop, width()-kLeft-kRight, height()-kTop-kBottom);
    const double y0 = centerZero_ ? 0.0 : yMinAuto_;
    const double y1 = centerZero_ ? std::max(std::abs(yMinAuto_), std::abs(yMaxAuto_)) : yMaxAuto_;
    const double yr = std::max(1e-3, y1 - y0);
    const int nY = 5;

    // Vạch ngang và nhãn Y.
    p.setPen(QPen(Theme::gridLine(), 1));
    QFont f;
    f.setPointSize(8);
    p.setFont(f);
    for (int i=0; i<=nY; ++i) {
        const double vy = i == 0 ? y0 : (centerZero_ && i > nY/2 ? y1 * (2.0*i/nY - 1.0) : yMinAuto_ + yr*i/nY);
        const double dy = i == 0 ? y0 : (centerZero_ ? (i <= nY/2 ? -y1 + 2*y1*i/nY : y1 - 2*y1*(nY-i)/nY) : (yMinAuto_ + yr*i/nY));
        const int y = plot.bottom() - (i * plot.height() / (double)nY);
        p.drawLine(QPoint(plot.left(), y), QPoint(plot.right(), y));
        if (centerZero_) {
            const double vv = std::abs(dy) < 1e-6 ? 0.0 : (i <= nY/2 ? - (y1 - 2*y1*i/nY) : (y1 - 2*y1*(nY-i)/nY));
            if (i == nY/2) vv = 0.0;
            p.setPen(Theme::textMuted());
            p.drawText(QRect(2, y-9, kLeft-4, 18), Qt::AlignRight, QString::number(std::lround(vv)));
        } else {
            p.setPen(Theme::textMuted());
            p.drawText(QRect(2, y-9, kLeft-4, 18), Qt::AlignRight, QString::number(std::lround(dy)));
        }
        p.setPen(QPen(Theme::gridLine(), 1));
    }

    // Trục X (2 nhãn).
    p.setPen(Theme::axisLine());
    for (int i=0; i<=2; ++i) {
        const int x = plot.left() + i*plot.width()/2;
        p.drawLine(QPoint(x, plot.bottom()), QPoint(x, plot.bottom()+3));
        const double trel = -10.0 + 10.0*i;  // giả định trend 10s
        p.setPen(Theme::textMuted());
        p.drawText(QRect(x-30, plot.bottom()+4, 60, 14), Qt::AlignHCenter, QString::number(trel) + "s");
    }

    // Nắp khung.
    p.setPen(Theme::borderLine());
    p.drawRect(plot.adjusted(0,0,-1,-1));
}

void TrendPlot::drawSeries(QPainter& p) const {
    if (t_.isEmpty()) return;

    const QRect plot(kLeft, kTop, width()-kLeft-kRight, height()-kTop-kBottom);
    const double trelMin = -10.0;  // khớp MATLAB trendSec 10s
    const double trelMax = 0.0;
    const double y0 = centerZero_ ? -std::max(std::abs(yMinAuto_), std::abs(yMaxAuto_)) : yMinAuto_;
    const double yr = std::max(1e-3, (centerZero_ ? 2*std::max(std::abs(yMinAuto_), std::abs(yMaxAuto_)) : yMaxAuto_ - yMinAuto_));

    auto mapx = [&](double tr) -> int {
        return plot.left() + (int)((tr - trelMin) / (trelMax - trelMin) * plot.width());
    };
    auto mapy = [&](double v) -> int {
        return plot.bottom() - (int)((v - y0) / yr * plot.height());
    };

    p.setPen(QPen(Theme::gray(), 2));
    for (int i=0; i<t_.size()-1; ++i) {
        const int x1 = mapx(t_[i]), x2 = mapx(t_[i+1]);
        if (x2 < plot.left()) continue;
        if (x1 > plot.right()) break;
        double vv = v1_[i];
        if (std::isfinite(vv))
            p.drawLine(QPoint(x1, mapy(vv)), QPoint(x2, mapy(v1_[i+1])));
    }
    p.setPen(QPen(Theme::accent(), 2));
    for (int i=0; i<t_.size()-1; ++i) {
        const int x1 = mapx(t_[i]), x2 = mapx(t_[i+1]);
        if (x2 < plot.left()) continue;
        if (x1 > plot.right()) break;
        double vv = v2_[i];
        if (std::isfinite(vv))
            p.drawLine(QPoint(x1, mapy(vv)), QPoint(x2, mapy(v2_[i+1])));
    }
}