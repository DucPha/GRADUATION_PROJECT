#include "widgets/header_bar.hpp"

#include <QHBoxLayout>
#include <QLabel>

#include "theme.hpp"

HeaderBar::HeaderBar(QWidget* parent) : QWidget(parent) {
    setFixedHeight(40);
    setAutoFillBackground(true);
    QPalette p = palette();
    p.setColor(QPalette::Background, Theme::headerBg());
    setPalette(p);

    QHBoxLayout* h = new QHBoxLayout(this);
    h->setContentsMargins(12, 4, 12, 4);

    QLabel* title = new QLabel("AUTOCAR  |  FUSION MONITOR");
    title->setStyleSheet("color: #FFFFFF; font-size: 12pt; font-weight: 600;");
    h->addWidget(title);

    h->addStretch(1);

    link_ = new QLabel("LINK  0/3");
    link_->setStyleSheet("color: #FFFFFF; font-size: 10pt; font-weight: 600;");
    h->addWidget(link_);

    h->addSpacing(24);

    clock_ = new QLabel("00:00:00");
    clock_->setStyleSheet("color: #FFFFFF; font-size: 10pt; font-weight: 600;");
    h->addWidget(clock_);

    h->addSpacing(24);
    QLabel* esc = new QLabel("ESC : EXIT");
    esc->setStyleSheet("color: #C0C0C0; font-size: 9pt;");
    h->addWidget(esc);
}

void HeaderBar::setLink(const QString& txt, const QColor& c) {
    link_->setText(txt);
    link_->setStyleSheet(QString("color: %1; font-size: 10pt; font-weight: 700;").arg(c.name()));
}

void HeaderBar::setClock(const QString& txt) {
    clock_->setText(txt);
}