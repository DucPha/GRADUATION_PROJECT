# -*- coding: utf-8 -*-
from common import *


def render():
    tp = doc.add_paragraph()
    tp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = tp.add_run("PHÂN TÍCH MODULE CAMERA LANE")
    r.font.name = "Calibri"
    r.font.size = Pt(28)
    r.font.bold = True
    r.font.color.rgb = RGBColor.from_string("1F3050")

    sp = doc.add_paragraph()
    sp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = sp.add_run("src/autonomous_vehicle/include/camera_lane.hpp\nsrc/autonomous_vehicle/src/camera_lane.cpp")
    r.font.name = "Consolas"
    r.font.size = Pt(12)
    r.font.color.rgb = RGBColor.from_string("555555")

    ip = doc.add_paragraph()
    ip.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = ip.add_run("Dự án Xe Tự Hành – Graduation Project  |  Camera USB + ESP32-S3 + ROS 2  |  Phân tích mã nguồn")
    r.font.size = Pt(11)
    r.font.italic = True
    r.font.color.rgb = RGBColor.from_string("777777")

    p(after=120)

    mk_table(
        ["Mục tiêu tài liệu", "Nội dung"],
        [
            ["Mục tiêu",
             "Giải thích **toàn bộ** `camera_lane.hpp` (238 dòng) và `camera_lane.cpp` (672 dòng): ý nghĩa từng dòng, "
             "từng hàm, từng hằng số, thuật toán và cơ chế đa luồng, đủ để hiểu đúng bản chất module."],
            ["Phạm vi",
             "2 file trên. Tài liệu liên quan tới `fusion_viz_node.cpp` (nơi khởi tạo và tiêu thụ kết quả), "
             "`obstacle_avoidance.hpp` (nơi dùng độ lệch) và `serial_esp32.cpp` (nơi gửi lệnh xuống ESP32)."],
            ["Phương pháp",
             "Đọc từng khối lệnh, dẫn số liệu thật (khung 640×400 và các hằng số trong header) để kiểm chứng từng khẳng định. "
             "Mọi trích dẫn code lấy nguyên văn từ file nguồn kèm số dòng."],
            ["Kết luận nhanh",
             "Đây **không phải** module dùng Hough Transform. Nó là bộ dò quét dòng (scanline) hai vạch, chạy trên hai luồng "
             "riêng theo chính sách latest-frame (bỏ khung cũ, không tích luỹ độ trễ), kèm bộ lập tốc độ ba trạng thái có "
             "hysteresis và bù đường cong."],
            ["Tình trạng mã nguồn",
             "Phát hiện **2 lỗi biên dịch** tại `fusion_viz_node.cpp` dòng 98 và 440 (đọc `dev_px`, trong khi `LaneOutput` "
             "chỉ có `dev_final_px`) và **5 hằng số không dùng** trong header. Chi tiết ở mục 11."],
        ],
        fracs=[0.20, 0.80],
    )

    h1("MỤC LỤC")
    toc = [
        "1. Tổng quan: module CameraLane làm gì",
        "   1.1. Bản chất thuật toán: dò quét scanline, không dùng Hough",
        "   1.2. Quy trình xử lý 9 bước",
        "   1.3. Vì sao tách hai luồng (capture / process)",
        "2. Vị trí trong kiến trúc hệ thống và luồng dữ liệu",
        "   2.1. Chuỗi khởi tạo module",
        "   2.2. Chuỗi tiêu thụ kết quả mỗi vòng",
        "   2.3. Bảng ánh xạ trường LaneOutput → nơi sử dụng",
        "   2.4. Sơ đồ luồng dữ liệu tổng thể",
        "3. File 1 – camera_lane.hpp: cấu trúc khai báo",
        "   3.1. Include và include guard",
        "   3.2. struct LaneOutput – hợp đồng đầu ra",
        "   3.3. Hằng số khung hình, đệm và cờ bật/tắt",
        "   3.4. Hằng số ROI và ngưỡng số điểm",
        "   3.5. Hằng số bộ lập tốc độ và hysteresis",
        "   3.6. enum class SpeedState",
        "   3.7. Hàm thành viên công khai",
        "   3.8. Hàm thành viên riêng",
        "   3.9. Cấu trúc FrameSlot và bộ đệm khung hình",
        "   3.10. Toàn bộ biến thành viên",
        "4. File 2 – camera_lane.cpp: cấu trúc cài đặt",
        "   4.1. Constructor",
        "   4.2. Destructor",
        "   4.3. start() – mở camera và khởi động hai luồng",
        "   4.4. stop() – dừng an toàn",
        "   4.5. get_latest() và buffered_frames()",
        "   4.6. capture_thread() – luồng đọc camera",
        "   4.7. process_thread() – luồng xử lý ảnh",
        "   4.8. Nhóm UTILITIES – toán học và lọc nhiễu",
        "5. detect_lanes(): giải thích chi tiết từng dòng",
        "   5.1. Đặt lại toàn bộ trạng thái đầu ra (dòng 415–427)",
        "   5.2. Chặn ảnh rỗng và ảnh quá nhỏ (dòng 429–447)",
        "   5.3. ROI động theo tốc độ (dòng 449–466)",
        "   5.4. Chuyển ảnh xám, làm mờ, Canny, phép đóng (dòng 468–479)",
        "   5.5. Vòng quét dòng – trái tim của module (dòng 481–516)",
        "   5.6. Quyết định làn kép hay không (dòng 518–532)",
        "   5.7. Cập nhật bề rộng làn và chuẩn hoá trục quét (dòng 534–551)",
        "   5.8. Cắt lọc điểm, làm trơn trung vị, tính slope (dòng 553–579)",
        "   5.9. Nhánh làn kép: dựng centerline và tính độ lệch (dòng 581–608)",
        "   5.10. Nhánh một vạch: suy luận trung tâm từ bề rộng làn (dòng 609–626)",
        "   5.11. Ra lệnh và bộ lập tốc độ (dòng 628–657)",
        "   5.12. Xuất ảnh gốc và ảnh visualize (dòng 659–671)",
        "6. Hệ toạ độ và cơ chế bám làn theo trục quét",
        "   6.1. Toạ độ pixel chuẩn của OpenCV",
        "   6.2. Hai trục giữa: xmid_default và xmid_scan_",
        "   6.3. Bám làn bằng cách giới hạn tốc độ dịch chuyển",
        "   6.4. Chi phí của cách quét dòng",
        "7. Cách module chống nhiễu – năm lớp bảo vệ",
        "   7.1. Nguồn nhiễu và lớp xử lý tương ứng",
        "   7.2. Trung vị 3 điểm – loại nhiễu đơn lẻ",
        "   7.3. Trung vị 5 điểm gần xe – vì sao lấy ở cuối",
        "   7.4. Đánh đổi giữa độ trễ và độ nhiễu",
        "8. Máy trạng thái bật ba mức tốc độ",
        "   8.1. Vì sao cần máy trạng thái thay vì so sánh trực tiếp",
        "   8.2. Bốn ngưỡng và điều kiện vào/ra",
        "   8.3. Bảng chuyển trạng thái (rút gọn từ dòng 385–400)",
        "   8.4. Trạng thái quy ra tốc độ mục tiêu và bộ lọc mềm",
        "   8.5. Cộng bù tốc độ từ phía LiDAR",
        "9. Từ độ lệch đo được đến lệnh lái gửi xuống ESP32",
        "   9.1. Chuỗi dữ liệu đầy đủ",
        "   9.2. Ba lệnh rời rạc và mối quan hệ với giá trị điều khiển",
        "   9.3. Vì sao nhánh một vạch không cộng bù đường cong",
        "   9.4. Bù sớm đường cong – chi tiết hàm calculate_slope_offset",
        "   9.5. Chuỗi giá trị điển hình trong một khung hình",
        "10. Bảng tham số cấu hình đầy đủ",
        "   10.1. Tham số camera",
        "   10.2. Tham số phát hiện biên và quét",
        "   10.3. Tham số ngưỡng nhận dạng làn",
        "   10.4. Tham số bộ lập tốc độ",
        "   10.5. Bảng tra nhanh: kích thước và ngân sách",
        "11. Các vấn đề đã phát hiện và đề xuất cải tiến",
        "   11.1. Lỗi biên dịch: hai chỗ đọc trường dev_px không tồn tại",
        "   11.2. median_value làm thay đổi thứ tự vector đầu vào",
        "   11.3. median_tail_x có bộ đệm cố định 8 phần tử",
        "   11.4. Hai trường tốc độ và speed_margin chưa được dùng",
        "   11.5. Năm hằng số khai báo nhưng không dùng",
        "   11.6. Ảnh gốc luôn được sao chép kể cả khi tắt visualize",
        "   11.7. calculate_slope yêu cầu 10 điểm trong khi bước kiểm tra chỉ đòi 5",
        "   11.8. stop() có thể treo nếu driver camera kẹt",
        "   11.9. clamp_float vô tình chặn luôn NaN – hành vi tốt nhưng chưa được ghi",
        "   11.10. Những điểm đã thiết kế tốt – không nên sửa",
        "   11.11. Hướng mở rộng tự nhiên",
        "   11.12. Thứ tự nên xử lý",
        "12. Kết luận",
        "   12.1. Module này làm gì",
        "   12.2. Những quyết định thiết kế đáng ghi nhận",
        "   12.3. Hạn chế cần biết",
        "   12.4. Danh sách kiểm tra khi chạy thử trên xe",
        "   12.5. Tóm tắt một câu",
    ]
    for t in toc:
        pp = doc.add_paragraph()
        pp.paragraph_format.space_after = Pt(1)
        add_runs(pp, t, size=19)
    p(after=60)
    pagebreak()

    # ================================================================ 1
    h1("1. Tổng quan: module CameraLane làm gì")

    p("`CameraLane` là lớp xử lý ảnh camera của xe. Nhiệm vụ của nó là **tìm hai vạch làn trên mặt đường, đo độ lệch của "
      "xe so với trục làn, rồi biến độ lệch đó thành lệnh lái và thành tốc độ chạy**.")

    p("Module này **không phải là một ROS 2 node**. Nó là một lớp C++ thuần, không dùng rclcpp, không publish topic, không "
      "phụ thuộc cv_bridge. Node `fusion_viz_node` giữ nó qua `std::unique_ptr<CameraLane>` và gọi hàm `get_latest()` mỗi vòng "
      "để lấy kết quả mới nhất. Việc tách module ra khỏi node cho phép phần thị giác được kiểm thử độc lập, không cần dựng ROS graph.")

    h2("1.1. Bản chất thuật toán: dò quét scanline, không dùng Hough")

    p("Điểm quan trọng nhất khi đọc module: **đây không phải pipeline OpenCV kinh điển**. Không có `HoughLinesP`, không có "
      "`fitLine`, không có ma trận hiệu chỉnh phối cảnh (homography). Thuật toán thật sự là **quét từng dòng ảnh nhị phân** (scanline):")

    bullets([
        "Cắt một vùng chữ nhật ở **nửa dưới** ảnh – nơi chỉ chứa mặt đường.",
        "Chuyển vùng đó thành ảnh xám, làm mờ, chạy Canny, rồi đóng lỗ hổng bằng phép toán hình thái.",
        "Quét **mỗi 4 hàng một lần**; trên mỗi hàng lấy **pixel biên đầu tiên** khi đi từ giữa ảnh ra trái, và **pixel biên "
        "đầu tiên** khi đi từ giữa ảnh ra phải.",
        "Ghép các điểm tìm được thành đường vạch trái và đường vạch phải, mỗi điểm là một cặp `(x, y)`.",
        "Từ hai đường này suy ra **đường trung tâm của làn**, rồi đo độ lệch so với trục giữa ảnh.",
    ])

    p("Vì sao chọn cách này? Vì vạch làn trong ảnh là **vật thể dạng cột dọc gần thẳng đứng**, và ảnh không cần hiệu chỉnh phối "
      "cảnh (camera nhắc thẳng về phía trước, gần như nhìn từ trên xe xuống). Quét theo dòng có tốc độ rất cao: toàn bộ chi phí "
      "là một vòng lặp đọc con trỏ hàng `edges_.ptr<uchar>(row)`, không tạo ảnh trung gian nào. Hough Transform phải duyệt toàn bộ "
      "không gian tham số (rho, theta) và chậm hơn nhiều so với nhu cầu thời gian thực.")

    h2("1.2. Quy trình xử lý 9 bước")

    mk_table(
        ["Bước", "Nội dung", "Nơi thực hiện"],
        [
            ["1", "Đặt lại toàn bộ trường của `LaneOutput` về trạng thái an toàn (STOP, lệch bằng 0)", "414–427"],
            ["2", "Chặn ảnh rỗng và ảnh quá nhỏ; chuẩn hoá tốc độ về khoảng 0–20 km/h", "429–447"],
            ["3", "Tính vùng ROI động: xe chạy càng nhanh thì vùng quan tâm càng rộng ra phía trên", "449–466"],
            ["4", "ROI → xám → GaussianBlur 5×5 → Canny(30, 120) → morphologyEx CLOSE 3×3", "468–479"],
            ["5", "Vòng quét dòng (bước 4 hàng, dò từ `xmid` ra hai bên): thu điểm vạch trái/phải và bề rộng làn", "493–516"],
            ["6", "Kết luận làn kép hay không; cập nhật bề rộng làn bằng lọc EMA; dịch `xmid` về trung tâm thực", "518–551"],
            ["7", "Cắt lọc theo `y0_used`, làm trơn trung vị, tính slope của từng vạch", "553–579"],
            ["8", "Tính `centerline` và độ lệch (nhánh hai vạch) hoặc suy luận trung tâm (nhánh một vạch)", "581–626"],
            ["9", "Sinh `camera_cmd`, chạy bộ lập tốc độ ba trạng thái, lọc mềm, tạo ảnh visualize", "628–671"],
        ],
        fracs=[0.07, 0.72, 0.21],
    )

    h2("1.3. Vì sao tách hai luồng (capture / process)")

    p("Module chạy **hai std::thread**: một luồng chỉ đọc camera, một luồng chỉ xử lý. Vấn đề gốc mà thiết kế này giải quyết là "
      "**độ trễ tích luỹ**. Nếu một luồng vừa đọc vừa xử lý, khi camera nhanh hơn máy tính thì hàng đợi ảnh sẽ dài ra, và lệnh "
      "lái mà xe nhận được là lệnh tính từ khung hình cũ. Với xe chạy 8,5 km/h = 2,36 m/s, chậm 200 ms cũng là 47 cm sai lệch – "
      "vượt cả bề rộng làn.")

    p("Chính sách áp dụng là **latest-frame policy**: bộ đệm chỉ có `FRAME_BUFFER_COUNT = 2` ô. Nếu luồng xử lý chưa xong mà "
      "camera đã có khung mới thì khung cũ bị **loại bỏ ngay**, không hề xếp hàng. Ưu tiên là dữ liệu mới nhất, không phải dữ liệu "
      "đầy đủ. Chi tiết ở mục 4.6 và 4.7.")

    p("Giao diện ra bên ngoài vì vậy rất đơn giản và luôn trả dữ liệu mới nhất: `get_latest()` chỉ chép kết quả khung vừa xong, "
      "không bao giờ chặn và không bao giờ xếp hàng.")

    pagebreak()

    # ================================================================ 2
    h1("2. Vị trí trong kiến trúc hệ thống và luồng dữ liệu")

    h2("2.1. Chuỗi khởi tạo module")
    p("Trong constructor của node `fusion_viz_node.cpp`, dòng 269–270:")
    code(rng(FUS, 269, 270, numbered=False), caption="fusion_viz_node.cpp:269–270")
    p("Chỉ cần hai dòng: tạo module với chỉ số camera đọc từ tham số ROS `cam_index` và số FPS đọc từ `camera_fps`, rồi gọi "
      "`start()`. Tham số thứ ba là `true` để bật backend V4L2. Nếu `start()` trả false, node chỉ in cảnh báo và tiếp tục chạy – "
      "thiết kế không chặn cả hệ thống chỉ vì một camera lỗi.")

    h2("2.2. Chuỗi tiêu thụ kết quả mỗi vòng")
    code(rng(FUS, 422, 434, numbered=False),
         caption="fusion_viz_node.cpp:422–434 – lấy tốc độ thật từ ESP32 rồi đưa ngược vào CameraLane")
    code(rng(FUS, 440, 449, numbered=False),
         caption="fusion_viz_node.cpp:440–449 – đưa độ lệch vào ObstacleAvoidance rồi xuống ESP32")

    p("Có một chi tiết kiến trúc đáng chú ý ở dòng 423–425: **tốc độ thật lấy từ ESP32 (vòng quay đo được), rồi truyền ngược "
      "vào `get_latest()`**. Nhờ vậy CameraLane biết xe đang chạy bao nhiêu để chọn vùng ROI, mà không cần thêm cảm biến tốc độ "
      "trên ROS. `current_speed_kmh` cũng là đầu vào của bộ lập tốc độ.")

    p("Dòng 442–443 cho thấy thứ tự dữ liệu vòng: camera đo được độ lệch làn, độ cong đường, cờ làn kép và trạng thái "
      "đèn giao thông → đưa vào `ObstacleAvoidance` → nơi đó quyết định có né hay không, có hãm tốc độ không → `dev_final_px` "
      "cuối cùng mới được gửi xuống ESP32. Nói cách khác **camera không bao giờ lái xe trực tiếp**, nó chỉ đề xuất; "
      "`ObstacleAvoidance` là nơi quyết định cuối cùng, vì chỉ khi đó mới có đủ thông tin cả LiDAR.")

    h2("2.3. Bảng ánh xạ trường LaneOutput → nơi sử dụng")

    mk_table(
        ["Trường `LaneOutput`", "Nơi dùng", "Mục đích"],
        [
            ["`left`, `right`", "fusion_viz_node.cpp:441",
             "Kiểm tra `is_dual_lane` (có thấy cả hai vạch không) → truyền cho ObstacleAvoidance để quyết định có cần "
             "bám sát vạch bên phải hơn khi né vật cản"],
            ["`vis`", "fusion_viz_node.cpp:436", "Vẽ lên panel camera trong ảnh telemetry"],
            ["`raw`", "fusion_viz_node.cpp:467–470", "Publish ảnh gốc qua topic riêng, dùng để xem lại khung hình khi debug"],
            ["`curve_angle_deg`", "fusion_viz_node.cpp:442",
             "Chuyển lại thành `dominant_slope` (chia 57,3) rồi truyền cho ObstacleAvoidance"],
            ["`dev_final_px`", "**không dùng ở fusion_viz_node**",
             "Lẽ ra phải truyền vào `ObstacleAvoidance` – xem mục 11.1"],
            ["`valid`", "fusion_viz_node.cpp:93, 270", "Quyết định hiển thị STOP hay lệnh thật trên thanh tiêu đề"],
            ["`camera_cmd`", "fusion_viz_node.cpp:93", "Hiển thị lệnh FWD / LEFT / RIGHT / STOP lên lớp phủ"],
            ["`center`", "không dùng", "Chỉ dùng nội bộ để tính độ lệch và vẽ"],
            ["`target_speed_x10`, `speed_factor`", "không dùng",
             "Chưa được nối vào bộ lập tốc độ của node – xem mục 11.4"],
            ["`pixels_used`, `frame_id`, `processing_ms`, `curvature`", "không dùng",
             "Dành cho chẩn đoán và giám sát hiệu năng"],
            ["`lane_color_bgr`, `lane_color_name`", "không dùng",
             "Dự kiến cho nhiệm vụ phát hiện màu vạch làn, phục vụ nhận dạng đèn giao thông"],
        ],
        fracs=[0.24, 0.19, 0.57],
    )

    note([
        "Có HAI chỗ trong fusion_viz_node.cpp đọc trường không tồn tại:",
        "  dòng  98:  lane ? lane->dev_px : 0      (dòng chữ 'Dev: xxx px' trên telemetry)",
        "  dòng 440:  has_cam ? lo.dev_px : 0      (độ lệch truyền vào ObstacleAvoidance)",
        "Nhưng struct LaneOutput (camera_lane.hpp dòng 25) khai báo trường là dev_final_px, hoàn toàn không có dev_px.",
        "→ Đây là lỗi BIÊN DỊCH, không phải lỗi logic: chương trình dừng ở bước biên dịch, chưa chạy được tới đây.",
        "Hệ quả nếu để nguyên: toàn bộ package không build được, không tạo ra node nào chạy được.",
        "Cách sửa: đổi cả hai thành dev_final_px. Sau khi sửa, giá trị này mới thực sự tới được ObstacleAvoidance –",
        "và lúc đó toàn bộ nhánh né vật cản dựa trên độ lệch làn mới hoạt động (xem mục 11.1).",
    ], title="Điểm chặn build đã phát hiện")

    h2("2.4. Sơ đồ luồng dữ liệu tổng thể")

    code([
        "  USB Camera (640x400, MJPG, buffer=1)",
        "        |",
        "        v",
        "  [capture_thread]   cap_.read(frame)  --->  frame_pool_[slot]   (2 ô đệm)",
        "        |   gán frame_id, thay thế khung chờ cũ  ->  frame_ready_cv_",
        "        v",
        "  [process_thread]   detect_lanes(frame, current_speed, work_output_)",
        "        |   ghi vào gray_, blur_, edges_, leftPts_, rightPts_, center",
        "        v",
        "  latest_output_ = work_output_        (dưới output_mtx_)",
        "        ^",
        "        |   get_latest(lo, current_speed)   <- gọi bởi fusion_viz_node",
        "        |",
        "  ObstacleAvoidance::update(lidar, deviation, slope, is_dual_lane, ...)",
        "        |",
        "        v",
        "  SerialESP32::send_command(dev_final_px, speed_control, emergency_stop)",
        "        |",
        "        v",
        "  ESP32-S3   -->   servo lái  +  động cơ DC",
    ], caption="Luồng dữ liệu từ camera tới động cơ")

    pagebreak()
