#pragma once

#include <QColor>
#include <QImage>
#include <QString>
#include <QWidget>

// ============================================================================
// PANEL HIỂN THỊ ẢNH
//
// Vẽ bằng QPainter để canh khung bằng tay thay vì để Qt tự co giãn: giữ đúng
// tỉ lệ, không méo, không nhảy layout khi ảnh đổi kích thước giữa các frame.
//
// Luồng an toàn: callback ROS (thread riêng) đẩy QImage vào hàng đợi ->
// timer GUI thread 30 Hz kéo ra -> tạo QPixmap tại đây (chỉ GUI thread được
// chạm QPixmap) -> paintEvent chỉ đọc QPixmap.
// ============================================================================
class ImagePanel : public QWidget {
    Q_OBJECT

public:
    explicit ImagePanel(const QString& title, QWidget* parent = nullptr);

    // Đặt ảnh mới. Pixmap phải đã được decode sẵn (xem DecodeWorker).
    // Nhận QImage, không nhận QPixmap: QPixmap sống trên GPU và chỉ được chạm
    // từ GUI thread. Callback ROS nằm ở thread khác nên phải decode ra QImage
    // (CPU, an toàn đa luồng) rồi chuyển sang QPixmap trong paintEvent.
    void setImage(const QImage& img);

    // Xoá ảnh, hiện "NO SIGNAL".
    void clearImage();

    // Đường viền màu để phân biệt trạng thái (ví dụ ảnh ROI dùng viền vàng).
    void setAccentColor(const QColor& c);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QString title_;
    QPixmap pm_;
    QColor accent_;
};