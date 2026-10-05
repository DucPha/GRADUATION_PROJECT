#include "widgets/node_link_table.hpp"

#include <QFont>
#include <QPainter>

#include <algorithm>

#include "theme.hpp"

NodeLinkTable::NodeLinkTable(QWidget* parent) : QWidget(parent) {
    setMinimumSize(180, 120);
}

void NodeLinkTable::setRows(const QVector<Row>& rows) {
    rows_ = rows;
    update();
}

void NodeLinkTable::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    const QRect full = rect();
    p.fillRect(full, Theme::panelBg());
    p.setPen(QPen(Theme::borderLine(), 1));
    p.drawRect(full.adjusted(0,0,-1,-1));

    const int headerH = 18;
    p.fillRect(QRect(0,0,width(),headerH), Theme::headerBg());
    p.setPen(Theme::textOnDark());
    QFont f;
    f.setBold(true);
    f.setPointSize(8);
    p.setFont(f);
    p.drawText(QRect(4,0,width()-4,headerH), Qt::AlignVCenter, "NODE LINK");

    f.setBold(false);
    f.setPointSize(8);
    p.setFont(f);
    p.setPen(QColor(210,210,210));
    p.drawText(QRect(width()-90,0,80,headerH), Qt::AlignRight, "RATE  STATUS");

    const int rowH = (height() - headerH) / std::max(1, rows_.size());
    f.setPointSize(9);
    for (int i = 0; i < rows_.size(); ++i) {
        const int y = headerH + i*rowH;
        if (i % 2 == 1) {
            p.fillRect(QRect(0,y,width(),rowH), QColor(248,248,248));
        }
        const Row& r = rows_[i];
        p.setBrush(r.dot);
        p.setPen(Qt::NoPen);
        p.drawEllipse(QPoint(8, y + rowH/2), 4, 4);
        p.setPen(Theme::textPrimary());
        p.drawText(QRect(16, y, width()-120, rowH), Qt::AlignVCenter, r.name);
        p.setPen(Theme::textMuted());
        p.drawText(QRect(width()-100, y, 50, rowH), Qt::AlignRight, r.rate);
        p.setPen(r.dot);
        f.setBold(true);
        p.setFont(f);
        p.drawText(QRect(width()-50, y, 46, rowH), Qt::AlignLeft, r.status);
        f.setBold(false);
    }
}