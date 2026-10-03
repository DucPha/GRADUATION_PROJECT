#include "dashboard_window.hpp"

#include <QApplication>
#include <QDateTime>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QImage>
#include <QKeyEvent>
#include <QPainter>
#include <QSplitter>
#include <QTime>
#include <QVariant>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <json/json.h>

#include "dashboard_node.hpp"
#include "theme.hpp"

// Bọc tìm trường JSON đơn giản, tránh crash khi thiếu field.
static QVariant getj(const Json::Value& v, const std::string& path) {
    const char* sep = ".";
    Json::Value cur = v;
    std::istringstream ss(path);
    std::string p;
    while (std::getline(ss, p, *sep)) {
        if (!cur.isMember(p)) return QVariant();
        cur = cur[p];
    }
    if (cur.isInt()) return QVariant(cur.asInt());
    if (cur.isUInt64()) return QVariant(qlonglong(cur.asUInt64()));
    if (cur.isDouble()) return QVariant(cur.asDouble());
    if (cur.isBool()) return QVariant(cur.asBool());
    if (cur.isString()) return QVariant(QString::fromStdString(cur.asString()));
    return QVariant();
}
static double getd(const Json::Value& v, const std::string& path, double d) {
    QVariant q = getj(v, path); return q.isValid() ? q.toDouble() : d;
}
static QString gets(const Json::Value& v, const std::string& path, const QString& d) {
    QVariant q = getj(v, path); return q.isValid() ? q.toString() : d;
}
static bool getb(const Json::Value& v, const std::string& path, bool d) {
    QVariant q = getj(v, path); return q.isValid() ? q.toBool() : d;
}
static bool cameraStale(const Json::Value& v) { return getb(v, "lane.stale", true); }
static QColor stateColor(const QString& s) {
    QString u = s.toUpper();
    if (u == "NORMAL") return Theme::green();
    if (u == "SLOW_DOWN") return Theme::amber();
    if (u == "DETECT_BYPASS") return QColor(240,115,0);
    if (u == "SWERVE_LEFT"||u=="SWERVE_RIGHT") return Theme::purple();
    if (u == "BYPASS_LEFT"||u=="BYPASS_RIGHT") return Theme::accent();
    if (u == "RETURN_LEFT"||u=="RETURN_RIGHT") return Theme::teal();
    if (u == "EMERGENCY") return Theme::red();
    return Theme::gray();
}
static QColor lightColor(const QString& s) {
    QString u = s.toUpper();
    if (u=="RED"||u=="STOP") return Theme::red();
    if (u=="YELLOW"||u=="20") return Theme::amber();
    if (u=="GREEN") return Theme::green();
    return Theme::gray();
}

DashboardWindow::DashboardWindow(DashboardNode* node, QWidget* parent)
    : QMainWindow(parent), node_(node), demo_(false) {
    elapsed_.start();
    startTime_ = QTime::currentTime();

    uiTimer_ = new QTimer(this);
    uiTimer_->setInterval(33);
    connect(uiTimer_, &QTimer::timeout, this, &DashboardWindow::onTick);
    uiTimer_->start();

    textTimer_ = new QTimer(this);
    textTimer_->setInterval(125);
    connect(textTimer_, &QTimer::timeout, this, &DashboardWindow::onTextTick);
    textTimer_->start();

    QWidget* cw = new QWidget(this);
    setCentralWidget(cw);
    QVBoxLayout* root = new QVBoxLayout(cw);
    root->setContentsMargins(6,4,6,4);
    root->setSpacing(4);

    header_ = new HeaderBar();
    root->addWidget(header_);

    QSplitter* top = new QSplitter(Qt::Horizontal);
    top->setHandleWidth(4);
    root->addWidget(top);

    lidar_ = new LidarMapWidget();
    top->addWidget(lidar_);

    QWidget* imgGrid = new QWidget();
    QGridLayout* ig = new QGridLayout(imgGrid);
    ig->setSpacing(4);
    ig->setContentsMargins(2,2,2,2);
    imgRaw_ = new ImagePanel("CAMERA  (RAW)");
    imgVis_ = new ImagePanel("LANE OVERLAY");
    imgRoi_ = new ImagePanel("CAMERA  ROI");
    imgBin_ = new ImagePanel("BINARY IMAGE");
    ig->addWidget(imgRaw_,0,0);
    ig->addWidget(imgVis_,0,1);
    ig->addWidget(imgRoi_,1,0);
    ig->addWidget(imgBin_,1,1);
    top->addWidget(imgGrid);
    top->setSizes({320,800});

    QWidget* bot = new QWidget();
    QSplitter* bh = new QSplitter(bot);
    bh->setStyleSheet("QSplitter::handle { width: 4px; }"); // hoặc chỉnh khoảng cách thông qua layout bên trong splitter nếu có
    bh->setContentsMargins(0,0,0,0);

    pidPlot_ = new TrendPlot("PID SPEED","km/h",0,20,false);
    pidPlot_->setLegend("SP","PV");
    bh->addWidget(pidPlot_);
    steerPlot_ = new TrendPlot("STEERING","px",-160,160,true);
    steerPlot_->setLegend("Lane","Final");
    bh->addWidget(steerPlot_);

    linkTable_ = new NodeLinkTable();
    QVector<NodeLinkTable::Row> r(5);
    r[0]={"LIDAR","--","WAITING",Theme::gray()};
    r[1]={"CAMERA","--","WAITING",Theme::gray()};
    r[2]={"ESP32","--","N/A",Theme::gray()};
    r[3]={"TRAFFIC LIGHT","--","WAITING",Theme::gray()};
    r[4]={"TURN DETECTOR","--","WAITING",Theme::gray()};
    linkTable_->setRows(r);
    bh->addWidget(linkTable_);

    QWidget* right = new QWidget();
    QVBoxLayout* rv = new QVBoxLayout(right);
    rv->setSpacing(2);
    rv->setContentsMargins(0,0,0,0);

    auto mk = [&](const QString& t,const QColor& a,const QVector<ParamPanel::Line>& l){ auto *p=new ParamPanel(t,a,l); rv->addWidget(p); return p; };
    pLidar_ = mk("LIDAR",Theme::accent(),{{"ok","Status"},{"fps","Scan rate"},{"front","Front"},{"rear","Rear"},{"left","Left"},{"right","Right"}});
    pLane_  = mk("LANE DETECTION",Theme::green(),{{"valid","Status"},{"cmd","Direction"},{"dev","Lane deviation"},{"mode","Detector"},{"curve","Curve angle"},{"dual","Lane type"},{"fps","Camera FPS"}});
    pOa_    = mk("OBSTACLE AVOIDANCE",Theme::amber(),{{"state","State"},{"dev","Dev final"},{"spd","Speed control"},{"estop","Emergency stop"}});
    pEsp_   = mk("ESP32",Theme::purple(),{{"ser","Serial"},{"v","Velocity"}});
    pAi_    = mk("AI DETECTION",Theme::red(),{{"light","Traffic light"},{"turn","Turn sign"}});
    pSys_   = mk("SYSTEM",Theme::headerBg(),{{"fps","Dashboard FPS"},{"rt","Render time"},{"age","Status age"},{"rate","Total rate"},{"up","Uptime"}});
    bh->addWidget(right);
    bh->setSizes({220,220,180,280});
    root->addWidget(bot);
    root->setStretchFactor(top,5);
    root->setStretchFactor(bot,2);
}

void DashboardWindow::setDemoMode(bool demo){ demo_=demo; }

void DashboardWindow::updateStatus(const QString& json){
    Json::Value v;
    Json::CharReaderBuilder b; std::string e; std::istringstream s(json.toStdString());
    if (!Json::parseFromStream(b,s,&v,&e)) return;
    const double t = elapsed_.elapsed() / 1000.0;
    (void)t;  // giữ symbol nếu sau này dùng; tLast_ vẫn cập nhật
    tLast_ = t;

    // LiDAR
    bool lok = getb(v,"lidar.ok",false);
    pLidar_->setValue("ok", lok?"OK":"NO DATA", lok?Theme::green():Theme::red());
    pLidar_->setValue("fps", QString::number(getd(v,"lidar.fps",0),'f',0)+" Hz", Theme::textPrimary());
    auto cmv=[](double x)->QString{ if(x<0)return "---"; if(x>=199.5)return ">200 cm"; return QString::number((int)std::round(x))+" cm"; };
    pLidar_->setValue("front", cmv(getd(v,"lidar.front",-1)), getd(v,"lidar.front",-1)<=30?Theme::red():Theme::textPrimary());
    pLidar_->setValue("rear", cmv(getd(v,"lidar.rear",-1)));
    pLidar_->setValue("left", cmv(getd(v,"lidar.left",-1)));
    pLidar_->setValue("right", cmv(getd(v,"lidar.right",-1)));

    bool llive = lok;
    lidar_->setHasData(llive);
    // zones tạm thời (chưa parse đầy đủ) — MATLAB dùng front/rear/left/right
    lidar_->setZoneDistances(getd(v,"lidar.front",-1), getd(v,"lidar.rear",-1), getd(v,"lidar.left",-1), getd(v,"lidar.right",-1));
    lidar_->setRangeCm(100); lidar_->setNearCm(30);

    // lane
    bool lv = getb(v,"lane.valid",false);
    pLane_->setValue("valid", lv?"TRACKING":"LOST", lv?Theme::green():Theme::red());
    pLane_->setValue("cmd", gets(v,"lane.cmd","---"));
    pLane_->setValue("dev", QString::number(getd(v,"lane.dev",0),'f',0)+" px");
    int lmode = (int)getd(v,"lane.mode",0); pLane_->setValue("mode", lmode==1?"IPM":"SCAN");
    pLane_->setValue("curve", QString::number(getd(v,"lane.curve",0),'f',1)+" deg");
    bool ldual = getb(v,"lane.dual",false); pLane_->setValue("dual", ldual?"DUAL":"SINGLE");
    pLane_->setValue("fps", QString::number(getd(v,"lane.fps",0),'f',1));

    // oa
    QString sst = gets(v,"oa.state","UNKNOWN");
    pOa_->setValue("state", sst, stateColor(sst));
    pOa_->setValue("dev", QString::number(getd(v,"oa.dev",0),'f',0)+" px");
    pOa_->setValue("spd", QString::number(getd(v,"oa.speed",0),'f',1));
    bool aest = getb(v,"oa.estop",false); pOa_->setValue("estop", aest?"ACTIVE":"OFF", aest?Theme::red():Theme::green());

    // esp
    bool eok = getb(v,"esp.ok",false);
    pEsp_->setValue("ser", eok?"CONNECTED":"FAIL", eok?Theme::green():Theme::red());
    if (getb(v,"esp.valid",false)) pEsp_->setValue("v", QString::number(getd(v,"esp.v",0),'f',1)+" km/h");
    else pEsp_->setValue("v","---", Theme::gray());

    // ai
    QString ltl = gets(v,"ai.light","NONE"); pAi_->setValue("light", ltl, lightColor(ltl));
    QString ttn = gets(v,"ai.turn","NONE");  pAi_->setValue("turn",  ttn,  ttn=="NONE"?Theme::gray():Theme::accent());

    // Trend plot. Trục X là "giây trước thời điểm hiện tại" trong cửa sổ
    // TREND_SEC, nên mỗi mẫu mang tRel = -(thời gian đã trôi kể từ mẫu trước).
    //
    // Bản cũ truyền `tlast - 10` với tlast = số giây kể từ lúc bật -> giá trị
    // tăng đơn điệu 0,1,2,3... Trong khi drawSeries() lại map trục X theo
    // khoảng CỐ ĐỊNH [-10, 0]. Mọi điểm vì thế rơi ngoài mép phải ngay từ
    // giây thứ 10 và đường biểu đồ biến mất hoàn toàn.
    const double now_s = elapsed_.elapsed() / 1000.0;
    const double dt = (last_sample_s_ >= 0.0)
        ? std::max(0.0, now_s - last_sample_s_)
        : 0.0;
    last_sample_s_ = now_s;

    pidPlot_->addPoint(-dt, getd(v,"pid.sp",getd(v,"esp.v",0)),
                       getd(v,"pid.pv",getd(v,"esp.v",0)));
    steerPlot_->addPoint(-dt, getd(v,"lane.dev",0), getd(v,"oa.dev",0));

    // link
    QVector<NodeLinkTable::Row> rows(5);
    rows[0]={"LIDAR", llive?QString::number(getd(v,"lidar.fps",0),'f',0)+" Hz":"--", llive?"ONLINE":"LOST", llive?Theme::green():(getd(v,"lidar.age_ms",1e9)<500?Theme::amber():Theme::red())};
    rows[1]={"CAMERA", lv?QString::number(getd(v,"lane.fps",0),'f',1):"--", lv?"ONLINE":(cameraStale(v)?"STALE":"LOST"), lv?Theme::green():(cameraStale(v)?Theme::amber():Theme::red())};
    rows[2]={"ESP32", eok?QString::number(getd(v,"esp.v",0),'f',1)+" km/h":"--", eok?"CONNECTED":"FAIL", eok?Theme::green():Theme::red()};
    rows[3]={"TRAFFIC LIGHT","--", gets(v,"ai.light","NONE")=="NONE"?"IDLE":"ACTIVE", gets(v,"ai.light","NONE")=="NONE"?Theme::gray():Theme::green()};
    rows[4]={"TURN DETECTOR","--", gets(v,"ai.turn","NONE")=="NONE"?"IDLE":"ACTIVE", gets(v,"ai.turn","NONE")=="NONE"?Theme::gray():Theme::green()};
    linkTable_->setRows(rows);
    int ok3 = (rows[0].dot==Theme::green()?1:0)+(rows[1].dot==Theme::green()?1:0)+(rows[2].dot==Theme::green()?1:0);
    updateLink(ok3);

    // sys
    pSys_->setValue("age", QString::number(getd(v,"lidar.age_ms",getd(v,"lane.age_ms",0)),'f',0)+" ms");
    pSys_->setValue("up", QTime(0,0,0).addMSecs(elapsed_.elapsed()).toString("hh:mm:ss"));
}
void DashboardWindow::updateLink(int ok3){ header_->setLink(QString("LINK  %1/3").arg(ok3), ok3==3?Theme::greenLight():ok3==0?Theme::redLight():Theme::amberLight()); }
void DashboardWindow::updateRaw(const QImage& img){ imgRaw_->setImage(img); }
void DashboardWindow::updateVis(const QImage& img){ imgVis_->setImage(img); }
void DashboardWindow::updateBin(const QImage& img){ imgBin_->setImage(img); }
void DashboardWindow::updateRoi(const QImage& img){ imgRoi_->setImage(img); }

// ============================================================================
// TICK 30 Hz - điểm duy nhất chạm đối tượng Qt với dữ liệu từ node.
//
// Bản cũ onTick() rỗng và toàn bộ cập nhật nằm trong callback ROS chạy ở
// spin thread: vừa chạm Qt sai thread, vừa không giữ được nhịp 30 Hz khi
// ảnh về chậm hơn (mọi ô số liệu đứng yên).
// ============================================================================
void DashboardWindow::onTick(){
    if (!node_) return;

    std::string json;
    if (node_->takeStatus(json)) updateStatus(QString::fromStdString(json));

    QImage img;
    if (node_->takeImage(img, DashboardNode::CH_RAW))  updateRaw(img);
    if (node_->takeImage(img, DashboardNode::CH_VIS))  updateVis(img);
    if (node_->takeImage(img, DashboardNode::CH_BIN))  updateBin(img);
    if (node_->takeImage(img, DashboardNode::CH_ROI))  updateRoi(img);

    if (demo_) return;
}
void DashboardWindow::onTextTick(){ QTime t=QTime::currentTime(); header_->setClock(t.toString("HH:mm:ss")); }
void DashboardWindow::onKeyEsc(){ QApplication::quit(); }
