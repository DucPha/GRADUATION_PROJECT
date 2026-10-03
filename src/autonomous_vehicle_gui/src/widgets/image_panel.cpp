#include "widgets/image_panel.hpp"

#include <QPainter>

#include "theme.hpp"

ImagePanel::ImagePanel(const QString& title, QWidget* parent)
    : QWidget(parent), title_(title), accent_(Theme::borderLine()) {
    setMinimumSize(200, 140);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void ImagePanel::setPixmap(const QPixmap& pm) {
    pm_ = pm;
    update();
}

void ImagePanel::clearImage() {
    pm_ = QPixmap();
    update();
}

void ImagePanel::setAccentColor(const QColor& c) {
    accent_ = c;
    update();
}

void ImagePanel::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::SmoothPixmapTransform, true);

    p.fillRect(rect(), Theme::emptyBg());

    const int title_h = 20;
    const QRect inner(1, title_h, width() - 2, height() - title_h - 1);

    if (pm_.isNull()) {
        p.setPen(QPen(Theme::emptyText(), 1));
        p.drawText(inner, Qt::AlignCenter, QStringLiteral("NO SIGNAL"));
    } else {
        // Vừa trong khung, giữ tỉ lệ gốc.
        const QSize scaled = pm_.size().scaled(inner.size(), Qt::KeepAspectRatio);
        const QRect target(inner.x() + (inner.width() - scaled.width()) / 2,
                           inner.y() + (inner.height() - scaled.height()) / 2,
                           scaled.width(), scaled.height());
        p.drawPixmap(target, pm_);
    }

    // Tiêu đề trên nền trắng.
    p.fillRect(QRect(0, 0, width(), title_h), Theme::panelBg());
    p.setPen(accent_.darker() > 60 ? accent_.darker(140) : Theme::textPrimary());
    p.drawText(QRect(6, 0, width() - 12, title_h), Qt::AlignVCenter | Qt::AlignLeft,
               title_);

    // Khung ngoài, dùng màu nhấn mạnh để phân biệt các panel.
    p.setPen(QPen(accent_, 1));
    p.setBrush(Qt::NoBrush);
    p.drawRect(rect().adjusted(0, 0, -1, -1));
}