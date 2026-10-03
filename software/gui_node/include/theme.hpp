#pragma once

#include <QColor>
#include <QString>

// Bảng màu dùng chung cho toàn giao diện. Tách riêng khỏi widget để đổi
// chủ đề (sáng/tối) ở một chỗ thay vì sửa trong từng paintEvent.
namespace Theme {

// Nền
inline QColor panelBg()      { return QColor(255, 255, 255); }
inline QColor windowBg()     { return QColor(248, 248, 250); }
inline QColor headerBg()     { return QColor(18, 18, 20); }
inline QColor plotBg()       { return QColor(255, 255, 255); }
inline QColor gridLine()     { return QColor(226, 226, 230); }
inline QColor axisLine()     { return QColor(140, 140, 146); }
inline QColor borderLine()   { return QColor(206, 206, 212); }

// Chữ
inline QColor textPrimary()  { return QColor(20, 20, 22); }
inline QColor textMuted()    { return QColor(120, 120, 128); }
inline QColor textOnDark()   { return QColor(255, 255, 255); }
inline QColor textFaint()    { return QColor(168, 168, 174); }

// Nhấn mạnh
inline QColor accent()       { return QColor(0, 114, 189); }   // xanh dữ liệu
inline QColor green()        { return QColor(33, 153, 71); }
inline QColor amber()        { return QColor(237, 153, 13); }
inline QColor red()          { return QColor(214, 31, 31); }
inline QColor purple()       { return QColor(140, 51, 166); }
inline QColor teal()         { return QColor(0, 153, 153); }
inline QColor gray()         { return QColor(128, 128, 128); }
inline QColor idle()         { return QColor(89, 115, 153); }

// Bản sáng hơn, dùng cho chữ trên nền tối và cho trạng thái tốt
inline QColor greenLight()   { return QColor(77, 217, 115); }
inline QColor amberLight()   { return QColor(255, 191, 51); }
inline QColor redLight()     { return QColor(255, 102, 102); }

// Vùng không có dữ liệu
inline QColor emptyBg()      { return QColor(240, 240, 242); }
inline QColor emptyText()    { return QColor(140, 140, 140); }

// Font chữ. Chỉ dùng font có sẵn trên mọi bản Ubuntu để không phụ thuộc
// font cài thêm: sans-serif cho nhãn, monospace cho số liệu cần thẳng cột.
inline QString fontFamily()     { return QStringLiteral("Ubuntu"); }
inline QString monoFamily()     { return QStringLiteral("Ubuntu Mono"); }

}  // namespace Theme