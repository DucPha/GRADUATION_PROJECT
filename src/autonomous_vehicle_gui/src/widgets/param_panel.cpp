#include "widgets/param_panel.hpp"

#include <QLabel>
#include <QVBoxLayout>
#include <QGridLayout>

#include "theme.hpp"

ParamPanel::ParamPanel(const QString& title,
                       const QColor& accent,
                       const QVector<Line>& lines,
                       QWidget* parent)
    : QGroupBox(title, parent) {
    setStyleSheet(QString("QGroupBox { font-weight: bold; border: 1px solid %1; border-radius: 4px; margin-top: 1ex; } QGroupBox::title { subcontrol-origin: margin; left: 6px; }")
                  .arg(accent.name()));
    QVBoxLayout* v = new QVBoxLayout(this);
    QGridLayout* g = new QGridLayout();
    g->setColumnStretch(0, 1);
    g->setColumnStretch(1, 1);
    g->setHorizontalSpacing(4);
    g->setVerticalSpacing(1);

    for (int i = 0; i < lines.size(); ++i) {
        QLabel* l = new QLabel(lines[i].label);
        l->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        l->setStyleSheet("color: #606060; font-size: 9pt;");
        QLabel* vlab = new QLabel("---");
        vlab->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        vlab->setStyleSheet("color: #505050; font-size: 10pt; font-weight: 600;");
        g->addWidget(l, i, 0);
        g->addWidget(vlab, i, 1);
        map_[lines[i].key] = {l, vlab};
    }
    v->addLayout(g);
    v->addStretch(1);
}

void ParamPanel::setValue(const QString& key, const QString& val, const QColor& color) {
    if (!map_.contains(key)) return;
    auto c = map_[key];
    c.val->setText(val);
    if (color.isValid()) {
        c.val->setStyleSheet(QString("color: %1; font-size: 10pt; font-weight: 600;").arg(color.name()));
    }
}