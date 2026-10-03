#pragma once

#include <QPixmap>
#include <QWidget>

// ============================================================================
// PANEL HIỂN THỊ ẢNH
//
// Một QLabel + QPixmap, vẽ bằng drawPixman() để canh khung bằng tay thay vì
// để Qt tự co giãn: giữ đúng tỉ lệ, không méo, không nhảy layout khi ảnh đổi
// kích thước giữa các frame.
// ============================================================================
class ImagePanel : public QWidget {
    Q_OBJECT

public:
    explicit ImagePanel(const QString& title, QWidget* parent = nullptr);

    // Đặt ảnh mới. Pixmap phải đã được decode sẵn (xem DecodeWorker).
    void setPixmap(const QPixmap& pm);

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