#pragma once

#include <QWidget>
#include <QLabel>

// ============================================================================
// HEADER BAR
// Hiển thị tiêu đề, LINK, CLOCK, ESC hint.
// ============================================================================
class HeaderBar : public QWidget {
    Q_OBJECT

public:
    explicit HeaderBar(QWidget* parent = nullptr);
    void setLink(const QString& txt, const QColor& c);
    void setClock(const QString& txt);

private:
    QLabel* link_;
    QLabel* clock_;
};