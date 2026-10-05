# -*- coding: utf-8 -*-
from common import *


def render():
    h1("6. Hệ toạ độ và cơ chế bám làn theo trục quét")

    h2("6.1. Toạ độ pixel chuẩn của OpenCV")
    p("Mọi phép tính trong module dùng **toạ độ pixel ảnh**, với gốc `(0,0)` ở góc trên bên trái, `x` tăng sang phải, `y` tăng "
      "xuống dưới. Hệ này do OpenCV quy định, không phải lựa chọn riêng. Hệ quả trực tiếp: **điểm càng gần xe (y càng lớn) "
      "càng quan trọng**, và vùng quan tâm phải nằm ở nửa dưới ảnh.")
    mk_table(
        ["Khái niệm", "Ý nghĩa vật lý"],
        [
            ["`y` nhỏ (0 → 220)", "Xa xe, gần chân trời. Ảnh ở vùng này bị bóp nhỏ mạnh, vạch mờ, góc nhìn cắt vạch."],
            ["`y` lớn (≈ 396)", "Sát xe. Vạch rõ nét, sai số pixel nhỏ nhất khi quy đổi ra centimet – **vì vậy phải lấy điểm ở đây**."],
            ["`y` tăng dần trong vòng quét", "Quét từ trên xuống dưới nên các điểm thu được đã **sắp xếp sẵn theo y tăng dần**. "
                                             "Đây là điều kiện tiên quyết để `median_tail_x()` lấy được “5 điểm cuối” mà không cần sắp xếp lại."],
            ["Toạ độ ROI", "Mọi tra cứu biên đều dùng `y − y0_roi`, tức toạ độ **trong ROI**, không phải toạ độ ảnh gốc."],
        ],
        fracs=[0.28, 0.72],
    )

    h2("6.2. Hai trục giữa: xmid_default và xmid_scan_")
    p("Đây là khái niệm quan trọng nhất để hiểu module, và là điểm dễ hiểu sai nhất.")
    code([
        "  xmid_default = W / 2            cố định theo khung hình, dùng làm MỐC ĐO lệch",
        "                               và làm trục quét ở vòng đầu tiên",
        "",
        "  xmid_scan_                     biến thành viên, NHỚ QUA CÁC KHUNG",
        "                               dùng làm vị trí bắt đầu quét mỗi hàng",
    ], caption="Hai trục giữa, hai vai trò hoàn toàn khác nhau")
    mk_table(
        ["", "`xmid_default`", "`xmid_scan_`"],
        [
            ["Nguồn", "`W / 2` – hằng số theo khung hình", "Thành viên `mutable`, khởi tạo `W/2`, rồi được cập nhật mỗi khung"],
            ["Ý nghĩa", "**Trục lý tưởng**: đường xe nên đi", "**Trục thực tế**: vị trí đang bám theo làn"],
            ["Dùng để", "Tính `raw_dev`; khởi tạo `xmid_candidate` khi chưa có dữ liệu", "Bắt đầu quét từ đây ra trái và ra phải ở mỗi dòng"],
            ["Cập nhật", "Không", "Dòng 534–551: về trung tâm đo được (nếu làn kép), về giữa ảnh (nếu một vạch)"],
            ["Vẽ trên ảnh", "Đường trắng", "Đường vàng"],
        ],
        fracs=[0.15, 0.40, 0.45],
    )
    note([
        "Vì sao cần hai trục: nếu chỉ dùng xmid_default để quét thì khi xe lệch sang một bên, vị trí quét không nằm giữa làn",
        "nên sẽ không tìm thấy mép trong của vạch ở phía xa xe. Nếu chỉ dùng xmid_scan_ để đo lệch thì module sẽ tự đặt mục tiêu",
        "bằng chính vị trí hiện tại, tức không bao giờ nhận ra xe đang lệch.",
        "Tách hai khái niệm này ra là điều kiện tiên quyết để vừa bám được làn vừa đo được độ lệch.",
    ], fill="E8F1DE")

    h2("6.3. Bám làn bằng cách giới hạn tốc độ dịch chuyển")
    p("Khi có cả hai vạch, trung tâm làn được đo bằng trung vị của các trung tâm theo dòng. Nhưng trục quét **không được nhảy thẳng** "
      "tới đó:")
    code([
        "  if |measured_mid − xmid_scan_| > 20 :  dịch tối đa 20 px mỗi khung",
        "  ngược lại                             :  chuyển thẳng tới measured_mid",
        "",
        "  Xe chạy ~2,4 m/s; ở 30 FPS thì 20 px/khung ≈ 0,67 px mỗi khung ≈ 3,6 px/giây.",
        "  Nếu bị đẩy lệch 150 px, trục quét cần ~2,5 sạp lại – đủ chậm để không sốc, đủ nhanh để kịp bám.",
    ], caption="Giới hạn dịch chuyển của trục quét")
    p("Khi chỉ thấy một vạch, mục tiêu không phải bám vạch mà là **trả trục quét về giữa hình học ảnh**, và giới hạn dịch chuyển "
      "chỉ còn 5 px mỗi khung – chặt hơn nhiều, vì lúc này tín hiệu kém tin cậy nên cần thận trọng hơn.")

    h2("6.4. Chi phí của cách quét dòng")
    mk_table(
        ["Cách duyệt", "Số phép so sánh mỗi dòng", "Ghi chú"],
        [
            ["Quét dòng từ trục ra hai bên", "≈ bề rộng làn ≈ 230 px", "Chỉ quét phần thực sự cần; trung tâm là `edges_.ptr<uchar>(row)`, "
                                                                  "không tạo ảnh trung gian"],
            ["Quét toàn bộ chiều rộng", "640 px", "Gần gấp 3 lần, mà phần ngoài bề rộng làn vốn không dùng"],
            ["Hough Transform", "duyệt không gian tham số (rho, theta)", "Chậm hơn nhiều lần, không cần thiết vì vạch gần thẳng đứng"],
        ],
        fracs=[0.30, 0.26, 0.44],
    )

    pagebreak()

    # ================================================================ 7
    h1("7. Cách module chống nhiễu – năm lớp bảo vệ")
    p("Ảnh camera ngoài đường nhiễu ở nhiều nguồn: rung máy, vệt sáng, bóng đổ, mờ chuyển động, phản xạ trên sơn vạch. Mỗi nguồn "
      "được xử lý bằng một lớp riêng, đúng nguyên tắc xử lý nhiễu ở đúng nơi nó phát sinh.")

    h2("7.1. Nguồn nhiễu và lớp xử lý tương ứng")
    mk_table(
        ["Nguồn nhiễu", "Biểu hiện", "Lớp xử lý"],
        [
            ["Nhiễu hạt cảm biến", "Biên Canny nhiều điểm rời rạc", "GaussianBlur 5×5 trước Canny (dòng 477)"],
            ["Vạch đứt khúc", "Chỉ tìm thấy vạch ở vài dòng", "`morphologyEx` CLOSE 3×3 bịt lỗ, nối đoạn (dòng 479)"],
            ["Điểm nhiễu rơi vào đường vạch", "Một điểm lệch hẳn, làm slope vô lý", "Trung vị 3 điểm (dòng 564)"],
            ["Điểm ở cuối đường vạch bị nhiễu", "Độ lệch nhảy theo", "Lấy **trung vị 5 điểm gần xe** thay vì trung bình (dòng 596, 614)"],
            ["Bề rộng làn dao động", "Suy luận trung tâm nhánh một vạch bị lệch", "EMA hệ số 0,2 cho `lane_width_est_px_` (dòng 531)"],
            ["Trục quét bị kéo lung tung", "Tìm nhầm vạch", "Giới hạn dịch 20 px/khung (dòng 538)"],
            ["Trạng thái tốc độ chập chờn", "Xe phanh giật giật", "Ngưỡng vào/ra tách biệt (hysteresis), dòng 378–383"],
            ["Máy tính chậm hơn camera", "Lệnh lái dựa trên khung cũ, sai lệch hàng chục cm", "Bỏ khung cũ ngay (latest-frame, dòng 241–243)"],
        ],
        fracs=[0.24, 0.33, 0.43],
    )

    h2("7.2. Trung vị 3 điểm – loại nhiễu đơn lẻ")
    p("Bộ lọc `median_smooth` lấy trung vị của ba điểm liên tiếp theo chiều sâu và gán vào **điểm giữa**. Ba điểm là mức tối thiểu "
      "để trung vị khác giá trị trung bình: nếu một điểm sai lệch, trung vị bỏ qua nó, còn trung bình thì bị kéo theo.")
    code([
        "  Điểm:      (x1, y1)   (x2, y2)   (x3, y3)",
        "             lệch      đúng       đúng        ->  trung vị = x2, giữ nguyên",
        "             đúng       lệch lớn   đúng        ->  trung vị = x2, sửa được lỗi",
        "             đúng       đúng       lệch        ->  trung vị = x2, giữ nguyên",
    ], caption="Ba tình huống khi lọc trung vị 3 điểm")
    note([
        "Đặc tính: điểm đầu và điểm cuối không bao giờ bị sửa (vòng lặp chạy từ i = 1 đến size − 2).",
        "Điều này là cố ý: điểm sát xe là điểm quyết định độ lệch, nên giữ nguyên giá trị quan sát được từ ảnh.",
        "Đánh đổi: nếu chính điểm cuối bị nhiễu thì nó không được sửa – đó là lý do 5.8 còn dùng thêm trung vị 5 điểm ở bước đo lệch.",
    ])

    h2("7.3. Trung vị 5 điểm gần xe – vì sao lấy ở cuối")
    bullets([
        "Các điểm được thu theo y tăng dần, nên **5 điểm cuối là 5 điểm gần xe nhất**.",
        "Vạt gần xe cho tương phản cao nhất và ít nhiễu phơi sáng nhất, nên tín hiệu ở đây đáng tin nhất.",
        "Trung vị trong 5 giá trị loại được tối đa 2 giá trị sai, tức chịu được nhiễu kiểu đối lập.",
        "Chỉ số `pixels_used` được đưa ra ngoài để khi chạy thật có thể quan sát độ dài đường vạch thực tế.",
    ])

    h2("7.4. Đánh đổi giữa độ trễ và độ nhiễu")
    p("Các bộ lọc trên đều **làm tăng độ trễ** (trung vị 5 điểm, EMA bề rộng làn, giới hạn dịch trục quét). Với xe chạy 8,5 km/h, "
      "mỗi 100 ms độ trễ là 24 cm sai lệch. Đây là điểm cần cân nhắc khi hiệu chỉnh:")
    mk_table(
        ["Cấu hình", "Nhiễu", "Độ trễ", "Phù hợp khi"],
        [
            ["Bộ lọc hiện tại (trung vị 3 + trung vị 5 + EMA 0,2 + giới hạn 20 px)", "Thấp", "Vừa phải", "Đường thẳng vạch rõ, camera cố định"],
            ["Giảm `MEDIAN_TAIL_POINTS` xuống 3", "Cao hơn", "Thấp hơn", "Tốc độ cao, cần phản hồi nhanh"],
            ["Tăng `MEDIAN_TAIL_POINTS` lên 7–9", "Thấp hơn nhiều", "Cao hơn", "Đường cong, vạch mờ, lá che"],
            ["Tăng `ALPHA_LANE_WIDTH` lên 0,4", "Bề rộng làn bám nhanh hơn", "Thấp hơn", "Làn đổi độ rộng thường xuyên"],
            ["Giảm giới hạn dịch `xmid_scan_` xuống 10 px", "Trục quét rung hơn", "Bám nhanh hơn", "Xe bị đẩy lệch lớn"],
        ],
        fracs=[0.34, 0.11, 0.15, 0.40],
    )

    pagebreak()

    # ================================================================ 8
    h1("8. Máy trạng thái bật ba mức tốc độ")

    h2("8.1. Vì sao cần máy trạng thái thay vì so sánh trực tiếp")
    p("Giả sửn chỉ so sánh `|slope| > 0,65` để giảm tốc. Khi xe đi đúng mép ngưỡng, nhiễu ảnh làm giá trị nhảy qua lại quanh 0,65, "
      "xe sẽ **phanh và tăng ga liên tục**. Máy trạng thái khắc phục đúng lỗi này bằng **hysteresis**: ngưỡng vào và ngưỡng ra khác nhau.")

    h2("8.2. Bốn ngưỡng và điều kiện vào/ra")
    mk_table(
        ["Điều kiện", "Vào", "Thoát", "Ý nghĩa"],
        [
            ["**Cua SHARP**", "`slope ≥ 0,85` hoặc `|dev| ≥ 80`", "`slope < 0,75` và `|dev| < 65`", "Đường cong gắt hoặc lệch lớn – phải giảm tốc mạnh"],
            ["**Cua CURVE**", "`slope ≥ 0,65` hoặc `|dev| ≥ 50`", "`slope < 0,55` và `|dev| < 40`", "Đường cong vừa"],
            ["**Vùng đệm giữa hai ngưỡng**", "`slope` nằm trong `[0,75, 0,85]`", "`|dev|` nằm trong `[65, 80]`", "Không thoát trạng thái – chống chập chờn"],
        ],
        fracs=[0.20, 0.25, 0.25, 0.30],
    )
    note([
        "Điểm thiết kế quan trọng: điều kiện vào là phép HOẶC (slope lớn HOẶC lệch lớn), còn điều kiện thoát là phép VÀ.",
        "Nghĩa là chỉ cần một trong hai tín hiệu là nặng thì đã giảm tốc, nhưng phải cả hai đều nhẹ mới tăng tốc lại.",
        "Hệ quả thực tế: xe vào chế độ chậm rất dễ, ra khỏi chế độ chậm khó hơn – đây là hướng an toàn đúng cho xe tự hành.",
    ])

    h2("8.3. Bảng chuyển trạng thái (rút gọn từ dòng 385–400)")
    mk_table(
        ["Trạng thái hiện tại", "Điều kiện ưu tiên theo thứ tự kiểm tra", "Trạng thái mới"],
        [
            ["STRAIGHT", "sharp vào", "SHARP"],
            ["STRAIGHT", "curve vào", "CURVE"],
            ["STRAIGHT", "không điều kiện nào", "giữ STRAIGHT"],
            ["CURVE", "sharp vào", "SHARP"],
            ["CURVE", "curve thoát", "STRAIGHT"],
            ["CURVE", "còn lại", "giữ CURVE"],
            ["SHARP", "sharp thoát", "CURVE"],
            ["SHARP", "curve thoát", "STRAIGHT"],
            ["SHARP", "còn lại", "giữ SHARP"],
        ],
        fracs=[0.24, 0.48, 0.28],
    )

    h2("8.4. Trạng thái quy ra tốc độ mục tiêu và bộ lọc mềm")
    mk_table(
        ["Trạng thái", "Tốc độ mục tiêu (×10)", "Tương đương km/h", "Ý nghĩa"],
        [
            ["STRAIGHT", "85", "8,5 km/h", "Tốc độ hành trình"],
            ["CURVE", "60", "6,0 km/h", "Giảm vừa đủ để cắt cung vừa phải"],
            ["SHARP", "45", "4,5 km/h", "Giảm mạnh trước cua gắt"],
        ],
        fracs=[0.18, 0.22, 0.20, 0.40],
    )
    p("Sau máy trạng thái còn hai bước nữa:")
    code([
        "  frame đầu tiên:   target = yêu cầu (không lọc)      -> tránh giảm tốc oan lúc khởi động",
        "  các frame sau:   new = 0,7 × yêu_cầu + 0,3 × target  -> lọc mềm, đáp ứng nhanh nhưng không giật",
        "",
        "  clamp 0…255 → làm tròn → uint8_t",
        "  speed_factor = 100 × target / 85  → 85:100, 60:71, 45:53",
    ], caption="Bộ lọc mềm và tính speed_factor")
    p("Nhánh không hợp lệ (dòng 649–657) là điểm an toàn quan trọng: khi ảnh hỏng hoặc không thấy làn, module **xoá cờ khởi tạo** để lần "
      "sau có dữ liệu trở lại thì bộ lọc khởi động lại từ giá trị yêu cầu, không phải từ giá trị 0 bị kẹp trước đó.")

    h2("8.5. Cộng bù tốc độ từ phía LiDAR")
    p("`ObstacleAvoidance` giữ `speed_margin_cm_` và có `get_speed_margin_cm()` với chú thích *“Lấy speed margin để truyền sang "
      "camera”*. Đây là hướng tích hợp hợp lý: khi LiDAR phát hiện vật cản gần, margin giảm, CameraLane nên hạ tốc thêm. "
      "Hiện tại module chưa nhận tham số này (xem mục 11.4).")

    pagebreak()

    # ================================================================ 9
    h1("9. Từ độ lệch đo được đến lệnh lái gửi xuống ESP32")

    h2("9.1. Chuỗi dữ liệu đầy đủ")
    code(rng(FUS, 440, 448, numbered=False), caption="fusion_viz_node.cpp:440–448 – chuỗi đưa độ lệch tới ESP32")
    mk_table(
        ["Bước", "Nơi thực hiện", "Nội dung"],
        [
            ["1", "camera_lane.cpp:606, 622", "`dev_final_px` = độ lệch đã cộng bù đường cong (nhánh hai vạch) hoặc độ lệch suy luận "
                                           "(nhánh một vạch)"],
            ["2", "fusion_viz_node.cpp:440", "Đọc ra biến `deviation` – **đây là chỗ đang lỗi biên dịch, xem mục 11.1**"],
            ["3", "fusion_viz_node.cpp:441", "`is_dual_lane` được **tính lại** ở node bằng điều kiện `!lo.left.empty() && !lo.right.empty()`, "
                                           "không dùng cờ `is_dual_lane` do module tự tính"],
            ["4", "fusion_viz_node.cpp:442", "Khôi phục lại `dominant_slope = curve_angle_deg / 57,3`, vì module chỉ công bố góc"],
            ["5", "obstacle_avoidance.cpp:83, 85, 107–135", "Trong `handle_normal()`: giữ nguyên `dev_px`; nếu phát hiện cần tránh thì "
                                                            "giảm 30 px (tránh phải), hoặc cộng 30 px (né sang phải)"],
            ["6", "fusion_viz_node.cpp:445–448", "`BypassCommand::dev_final_px` được gán vào `SerialCommand::dev_final_px` rồi gửi xuống ESP32"],
        ],
        fracs=[0.06, 0.28, 0.66],
    )
    note([
        "Bước 5 là minh hoạ điểm mạnh của việc để quyết định cuối ở ObstacleAvoidance: camera đo được độ lệch thuần túy,",
        "còn chỗ dành cho né vật cản do LiDAR quyết định – mà quyết định đó có thể cộng hoặc trừ thêm 30…80 px vào độ lệch gốc.",
        "Camera không bao giờ tự ý né; nó chỉ cung cấp độ lệch đúng với đường vạch.",
    ], fill="E8F1DE")

    h2("9.2. Ba lệnh rời rạc và mối quan hệ với giá trị điều khiển")
    mk_table(
        ["Điều kiện trên `dev_final_px`", "`camera_cmd`", "Giá trị điều khiển thật"],
        [
            ["`valid = false`", "STOP", "Không có – node chỉ hiển thị"],
            ["`|dev| < 15`", "FWD", "`dev ≈ 0`, ESP32 giữ servo gần vị trí trung tính"],
            ["`dev < 0`", "LEFT", "`dev` âm, ESP32 lái trái theo độ lớn"],
            ["`dev ≥ 15`", "RIGHT", "`dev` dương, ESP32 lái phải theo độ lớn"],
        ],
        fracs=[0.30, 0.22, 0.48],
    )
    p("Điểm cần nhấn mạnh: `camera_cmd` là **chuỗi ký hiệu để telemetry**, không phải đầu vào điều khiển. Giá trị điều khiển thật là "
      "`dev_final_px` với dấu và độ lớn liên tục. Nếu ai đó chỉ dùng `camera_cmd` để điều khiển thì hệ thống sẽ chỉ có ba mức lái "
      "rời rạc và mất hoàn toàn khả năng bám làn mềm.")

    h2("9.3. Vì sao nhánh một vạch không cộng bù đường cong")
    mk_table(
        ["", "Nhánh hai vạch (dòng 581–608)", "Nhánh một vạch (dòng 609–626)"],
        [
            ["Nguồn độ lệch", "Đo trực tiếp trung tâm làn từ hai mép vạch", "Suy luận: `toà độ vạch ± nửa bề rộng làn`"],
            ["Độ tin cậy", "Cao – hai vạch độc lập xác nhận nhau", "Thấp – phụ thuộc `lane_width_est_px_` ước lượng bằng EMA"],
            ["Cộng `slope_offset`?", "**Có** (dòng 606)", "**Không** – chỉ dùng giá trị thô"],
            ["Lý do", "Đường cong đo đủ tin cậy để báo động sớm", "Ước lượng trung tâm đã có sai số, cộng thêm bù chỉ làm sai nặng thêm"],
        ],
        fracs=[0.20, 0.40, 0.40],
    )

    h2("9.4. Bù sớm đường cong – chi tiết hàm calculate_slope_offset")
    p("Đây là một trong những ý tưởng đáng chú ý nhất của module. Vấn đề: khi xe đang đi trên đường cong, **những centimet cuối cùng "
      "trước xe trông gần như thẳng**, dù đường phía trước đang cua. Nếu chỉ dùng độ lệch thô, xe sẽ giữ thẳng và lao ra khỏi làn.")
    code([
        "  magnitude = |slope|          (slope = dx/dy, đơn vị px trên 1 px chiều sâu)",
        "",
        "  |slope| ≤ 0,80   ->   offset = 0 px            đường thẳng, không bù",
        "  |slope| ≥ 1,00   ->   offset = 120 px          đường cong mạnh, bù tối đa",
        "  0,80…1,00        ->   offset nội suy 50 → 120 px",
        "",
        "  Dấu của offset luôn NGƯỢC dấu slope:",
        "      slope > 0  ->  trả về −offset",
        "      slope < 0  ->  trả về +offset",
    ], caption="Quy tắc của calculate_slope_offset")
    mk_table(
        ["Tình huống", "`slope`", "`slope_offset`", "Tác dụng"],
        [
            ["Đường thẳng", "0,00", "0", "Không can thiệp"],
            ["Cua nhẹ", "0,90", "khoảng −65 px", "Báo động sớm vừa đủ"],
            ["Cua vừa", "0,95", "khoảng −87 px", "Báo động sớm rõ rệt"],
            ["Cua gắt", "1,20", "−120 px", "Bù tối đa"],
        ],
        fracs=[0.22, 0.16, 0.24, 0.38],
    )
    note([
        "Điểm phải hiểu đúng: đây KHÔNG phải bù sai lệch, mà là cộng thêm một lệch có chủ ý để lái sớm.",
        "Do đó lệnh phát ra không còn mô tả đúng vị trí xe so với tâm làn tại thời điểm đó.",
        "Đổi lại, xe vào cua trước và thoát cua mềm hơn – đánh đổi lấy ổn định hành trình.",
        "Vì vậy bước dòng 601–604 rất quan trọng: nếu lệch thô đã vượt 95 px thì bù bị xoá, vì lúc đó xe đã lệch xa,",
        "cần quay về làn bằng phản hồi thực chứ không phải bằng bù suông.",
    ])

    h2("9.5. Chuỗi giá trị điển hình trong một khung hình")
    code([
        "  Vạch trái ở x = 150, vạch phải ở x = 380, ảnh rộng 640",
        "      center_x = (150 + 380) / 2 = 265",
        "      xmid_default = 320",
        "      raw_dev = 265 − 320 = −55 px",
        "      slope của vạch phải = +0,95  (đường cong)",
        "      slope_offset ≈ −87 px",
        "      dev_final_px = −55 + (−87) = −142 px   ->  LEFT",
        "      |dev| = 142 ≥ 80   ->  máy trạng thái chuyển sang SHARP",
        "      target = 45 (×10)  ->  speed_factor ≈ 53",
    ], caption="Ví dụ tính tay một khung hình")

    pagebreak()