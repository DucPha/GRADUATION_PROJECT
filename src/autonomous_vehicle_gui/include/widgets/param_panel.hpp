#pragma once

#include <QGroupBox>
#include <QMap>
#include <QString>
#include <QVector>
#include <QWidget>

// ============================================================================
// PARAM PANEL
//
// Hiển thị 6 nhóm giống MATLAB: lidar, lane, oa, esp, ai, sys.
// Mỗi dòng có nhãn trái và giá trị phải. Màu viền GroupBox dùng accent.
// ============================================================================
class ParamPanel : public QGroupBox {
    Q_OBJECT

public:
    struct Line {
        QString key;
        QString label;
    };

    explicit ParamPanel(const QString& title,
                        const QColor& accent,
                        const QVector<Line>& lines,
                        QWidget* parent = nullptr);

    void setValue(const QString& key, const QString& val, const QColor& color = QColor());

private:
    struct Cell {
        QLabel* label;
        QLabel* val;
    };
    QMap<QString, Cell> map_;
};