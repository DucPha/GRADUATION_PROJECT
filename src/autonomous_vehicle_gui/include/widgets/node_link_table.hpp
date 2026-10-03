#pragma once

#include <QColor>
#include <QString>
#include <QVector>
#include <QWidget>

// ============================================================================
// NODE LINK TABLE
//
// Hiển thị 5 dòng giống MATLAB: LIDAR, CAMERA, ESP32, TRAFFIC LIGHT, TURN DETECTOR
// với cột RATE, STATUS, DOT màu.
// ============================================================================
class NodeLinkTable : public QWidget {
    Q_OBJECT

public:
    struct Row {
        QString name;
        QString rate;
        QString status;
        QColor dot;
    };

    explicit NodeLinkTable(QWidget* parent = nullptr);
    void setRows(const QVector<Row>& rows);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QVector<Row> rows_;
};