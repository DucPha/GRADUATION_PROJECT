#pragma once

#include <QVector>
#include <QWidget>

// ============================================================================
// TREND PLOT
//
// Vẽ 2 series (lane + final, hoặc SP + PV) với đường kẻ trục và autoscale
// theo trục dọc. Không dùng Qwt — vẽ tay bằng QPainter để nhanh và không
// thêm dependency.
// ============================================================================
class TrendPlot : public QWidget {
    Q_OBJECT

public:
    TrendPlot(const QString& title,
              const QString& yLabel,
              double yMin,
              double yMax,
              bool centerZero,
              QWidget* parent = nullptr);

    // Thêm điểm mới (thời gian relatif -trendSec..0)
    void addPoint(double tRel, double v1, double v2);
    void clear();

    void setYRange(double yMin, double yMax);
    void setLegend(const QString& l1, const QString& l2);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawBackground(QPainter& p) const;
    void drawGrid(QPainter& p) const;
    void drawSeries(QPainter& p) const;
    void drawTitle(QPainter& p) const;

    QString title_;
    QString yLabel_;
    double yMin_;
    double yMax_;
    bool centerZero_;

    QString l1_;
    QString l2_;

    // Số điểm giữ trong buffer = uiHz * trendSec. MATLAB dùng N = round(30*10).
    QVector<double> t_;
    QVector<double> v1_;
    QVector<double> v2_;

    // Tự động mở rộng trục Y nếu điểm vượt ra ngoài.
    double yMinAuto_;
    double yMaxAuto_;
};