# -*- coding: utf-8 -*-
from common import *


def render():
    h1("10. Bảng tham số cấu hình đầy đủ")
    p("Toàn bộ tham số nằm trong class `CameraLane` dưới dạng hằng số `static constexpr`, biên dịch một lần. Người dùng muốn đổi "
      "hành vi phải sửa header rồi build lại – ưu điểm là không có đường ghi đè lúc chạy (dễ lỗi), nhược điểm là không hiệu chỉnh "
      "nhanh được trên xe.")

    h2("10.1. Tham số camera")
    code(rng(HPP, 52, 62))
    mk_table(
        ["Tham số", "Giá trị", "Ý nghĩa"],
        [
            ["`FRAME_W`, `FRAME_H`", "640 × 400", "Kích thước khung yêu cầu. Tỉ lệ 8:5 là ảnh rộng, hợp với camera trước xe."],
            ["`ENABLE_VISUALIZATION`", "true", "Bật ảnh vẽ đè. Là `if constexpr` nên bật/tắt bằng cách sửa hằng số, không tốn thời gian chạy."],
        ],
        fracs=[0.26, 0.14, 0.60],
    )

    h2("10.2. Tham số phát hiện biên và quét")
    code(rng(HPP, 65, 75))
    mk_table(
        ["Tham số", "Giá trị", "Ý nghĩa"],
        [
            ["`CANNY_LOW`, `CANNY_HIGH`", "30, 120", "Ngưỡng Canny. Tỉ lệ 1:4 là cân bằng cho vạch đen trên nền sáng."],
            ["`BLUR_KERNEL`", "5", "Cửa sổ làm mờ. Lớn hơn giảm nhiễu nhưng mất chi tiết vạch mảnh."],
            ["`SCAN_STEP`", "4", "Bước nhảy quét theo chiều sâu. Càng nhỏ càng chính xác nhưng chậm và nhiễu hơn."],
            ["`MEDIAN_TAIL_POINTS`", "5", "Số điểm gần xe dùng để đo độ lệch."],
            ["`ROI_FACTOR_SLOW`", "0,70", "Tỉ lệ bắt đầu ROI khi xe chạy chậm (≤ 4 km/h)."],
            ["`ROI_FACTOR_FAST_MIN`", "0,55", "Tỉ lệ ROI nhỏ nhất, đạt khi xe chạy nhanh nhất. ROI mở rộng lên phía xa."],
            ["`ROI_SPEED_START`", "4,0", "Trên ngưỡng này ROI bắt đầu co lại theo tốc độ."],
            ["`ROI_SPEED_GAIN`", "0,02", "Độ dốc của việc co ROI: mỗi 1 km/h giảm hệ số 0,02."],
            ["`SINGLE_ROI_FACTOR`", "0,85", "ROI khi chỉ thấy một vạch – hẹp hơn, tập trung sát xe."],
        ],
        fracs=[0.30, 0.13, 0.57],
    )
    note([
        "Kiểm tra tương tác: từ 4 km/h đến 20 km/h cần mất (0,70 − 0,55)/0,02 = 7,5 km/h mới đạt hệ số nhỏ nhất.",
        "Với giá trị tốc độ tối đa khai báo là 20 km/h thì ROI chạm đúng đáy của khoảng – không có phần thừa.",
        "Số dòng quét thực tế: 30 dòng khi chậm (y từ 280) và 45 dòng khi nhanh (y từ 220), bước 4.",
    ])

    h2("10.3. Tham số ngưỡng nhận dạng làn")
    code(rng(HPP, 77, 91))
    mk_table(
        ["Tham số", "Giá trị", "Ý nghĩa"],
        [
            ["`MIN_PRELIM_POINTS`", "15", "Số dòng tối thiểu cho mỗi vạch ở bước sơ bộ, trước khi cắt lọc."],
            ["`MIN_FINAL_POINTS`", "5", "Số điểm tối thiểu sau cắt lọc và làm trơn để vạch được coi là hữu dụng."],
            ["`MIN_COMMON_POINTS`", "10", "Số dòng có **cả hai vạch** cùng hợp lệ, điều kiện để kết luận làn kép."],
            ["`MIN_LANE_WIDTH_PX`", "200", "Bề rộng làn nhỏ nhất chấp nhận được, tính trên ảnh rộng 640."],
            ["`MAX_LANE_WIDTH_PX`", "500", "Bề rộng làn lớn nhất – loại trường hợp dò tìm nhầm hai cạnh xa nhau."],
            ["`DEFAULT_LANE_WIDTH_PX`", "230", "Bề rộng làn giả định lúc khởi động, được nhân theo tỉ lệ kích thước ảnh thật."],
            ["`ALPHA_LANE_WIDTH`", "0,20", "Trọng số cho bề rộng mới đo trong công thức EMA."],
            ["`FWD_THRESHOLD_PX`", "15", "Lệch tuyệt đối dưới ngưỡng này thì phát lệnh FWD."],
            ["`SLOPE_OFFSET_START`", "0,80", "Độ cong bắt đầu được bù."],
            ["`SLOPE_OFFSET_END`", "1,00", "Độ cong đạt mức bù tối đa."],
            ["`SLOPE_OFFSET_MIN_PX`", "50", "Lượng bù tại ngưỡng bắt đầu."],
            ["`SLOPE_OFFSET_MAX_PX`", "120", "Lượng bù tối đa."],
        ],
        fracs=[0.32, 0.13, 0.55],
    )
    p("Ba ngưỡng `MIN_*` tạo thành hình chóp: cần nhiều điểm để kết luận làn kép (15 và 10), nhưng chỉ cần vài điểm để coi là có dữ liệu "
      "lệch (5). Nhờ vậy khi xe đang vào cua, vạch bị cắt mất phần xa nhưng phần gần xe vẫn đủ dùng để lái.")

    h2("10.4. Tham số bộ lập tốc độ")
    code(rng(HPP, 99, 116))
    mk_table(
        ["Tham số", "Giá trị", "Ý nghĩa"],
        [
            ["`SPEED_STRAIGHT_X10`", "85", "Tốc độ thẳng = 8,5 km/h."],
            ["`SPEED_CURVE_X10`", "60", "Tốc độ cua vừa = 6,0 km/h."],
            ["`SPEED_SHARP_X10`", "45", "Tốc độ cua gắt = 4,5 km/h."],
            ["`ALPHA_TARGET_SPEED`", "0,70", "Trọng số yêu cầu trong bộ lọc mềm tốc độ."],
            ["`CURVE_ENTER_SLOPE` / `CURVE_EXIT_SLOPE`", "0,65 / 0,55", "Ngưỡng vào và ra của trạng thái CURVE theo độ cong."],
            ["`SHARP_ENTER_SLOPE` / `SHARP_EXIT_SLOPE`", "0,85 / 0,75", "Ngưỡng vào và ra của trạng thái SHARP theo độ cong."],
            ["`CURVE_ENTER_DEV_PX` / `CURVE_EXIT_DEV_PX`", "50 / 40", "Ngưỡng vào và ra của CURVE theo độ lệch."],
            ["`SHARP_ENTER_DEV_PX` / `SHARP_EXIT_DEV_PX`", "80 / 65", "Ngưỡng vào và ra của SHARP theo độ lệch."],
        ],
        fracs=[0.36, 0.17, 0.47],
    )

    h2("10.5. Bảng tra nhanh: kích thước và ngân sách")
    mk_table(
        ["Đại lượng", "Giá trị"],
        [
            ["Kích thước bộ đệm", "`std::array<FrameSlot, 2>` – 2 ô `cv::Mat` 640×400×3 ≈ 0,73 MB mỗi ô"],
            ["Cấp phát trong vòng xử lý", "1 clone ảnh gốc + 1 clone ảnh vẽ (khi bật visualize)"],
            ["Vector dự trữ sẵn", "9 vector × 128 phần tử `cv::Point`"],
            ["Số dòng quét", "30 (chậm) → 45 (nhanh), bước 4"],
            ["Chi phí mỗi dòng quét", "≈ 230 phép so sánh, không cấp phát"],
            ["Độ trễ thuần túy của thuật toán", "≈ 1,5 khung hình"],
        ],
        fracs=[0.36, 0.64],
    )

    pagebreak()

    # ================================================================ 11
    h1("11. Các vấn đề đã phát hiện và đề xuất cải tiến")

    h2("11.1. Lỗi biên dịch: hai chỗ đọc trường dev_px không tồn tại")
    code(rng(FUS, 98, 98, numbered=False), caption="fusion_viz_node.cpp:98 – chữ Dev trên telemetry")
    code(rng(FUS, 440, 440, numbered=False), caption="fusion_viz_node.cpp:440 – độ lệch truyền vào ObstacleAvoidance")
    p("`LaneOutput` khai báo `dev_final_px` (`camera_lane.hpp` dòng 25) và không có `dev_px`. Cả hai dòng trên đều không biên dịch được. "
      "Điểm đáng lưu ý: `BypassCommand` và `SerialCommand` **có** trường `dev_final_px`, nên có thể tên `dev_px` là sót lại từ một phiên "
      "bản trước khi đổi tên trường.")
    mk_table(
        ["Dòng", "Cách sửa", "Hậu quả nếu sửa sai"],
        [
            ["98", "`lane ? lane->dev_final_px : 0`", "Chỉ ảnh hưởng hiển thị telemetry"],
            ["440", "`has_cam ? lo.dev_final_px : 0`", "Sai ở đây là **toàn bộ tính năng dựa trên độ lệch làn ngừng hoạt động**"],
        ],
        fracs=[0.10, 0.44, 0.46],
    )
    p("Kèm theo, dòng 441 của node tự tính lại `is_dual_lane` bằng `!lo.left.empty() && !lo.right.empty()` thay vì dùng cờ "
      "`is_dual_lane` mà module đã tính sẵn. Điều kiện này **yếu hơn**: hai vector còn sót điểm cũ cũng được coi là làn kép. Nên ưu tiên "
      "`lo.is_dual_lane`.")

    h2("11.2. median_value làm thay đổi thứ tự vector đầu vào")
    code(rng(CPP, 341, 346), caption="cpp:341–346 – hàm có tham số tham chiếu không const")
    p("`std::nth_element` **hoán đổi các phần tử trong dãy** để đặt phần tử thứ `mid` vào đúng chỗ. Vì tham số là "
      "`std::vector<int>&` chứ không phải `const&`, lời gọi `median_value(prelim_centers_)` ở dòng 535 sẽ làm mất thứ tự "
      "sắp xếp của `prelim_centers_`.")
    bullets([
        "**Hiện tại chưa gây lỗi**: `prelim_centers_` được xoá lại ở đầu mỗi khung (dòng 483) và chỉ dùng để lấy trung vị, "
        "không dựa vào thứ tự – nên đây là lỗi **tiềm ẩn**, chưa lộ ra.",
        "Cách sửa: đổi tham số thành `const std::vector<int>&`. Thư viện chuẩn cung cấp phiên bản `nth_element` nhận "
        "`const` iterator từ C++11, nên không tốn bộ nhớ sao chép.",
        "Hoặc rẻ hơn: đổi `prelim_centers_` thành `std::vector<int>` truyền bằng giá trị – vector vài chục phần tử, gần như "
        "không tốn gì.",
    ])

    h2("11.3. median_tail_x có bộ đệm cố định 8 phần tử")
    code(rng(CPP, 328, 339), caption="cpp:328–339 – bộ đệm std::array<int, 8> đặt cứng")
    p("`std::array<int, 8>` giới hạn số điểm lấy ở **8 phần tử**, trong khi tham số `tail_count` lại nhận giá trị bất kỳ. "
      "Với `MEDIAN_TAIL_POINTS = 5` hiện tại thì vô hại, nhưng nếu ai đó tăng hằng số lên 10 để giảm nhiễu thì hàm âm thầm chỉ lấy "
      "8 điểm mà không báo gì. Sửa bằng `static_assert(tail_count <= 8)` để trình biên dịch bắt lỗi sớm.")

    h2("11.4. Hai trường tốc độ và speed_margin chưa được dùng")
    mk_table(
        ["Trường", "Nơi sinh ra", "Trạng thái"],
        [
            ["`target_speed_x10`", "dòng 644–645, sau bộ lọc mềm", "Node chưa đọc; ESP32 nhận `speed_control` từ `ObstacleAvoidance`, không phải từ đây"],
            ["`speed_factor`", "dòng 647", "Cùng lý do – đang là giá trị tính nhưng chưa ai dùng"],
            ["`get_speed_margin_cm()`", "`obstacle_avoidance.hpp:58` – chú thích ghi rõ *“để truyền sang camera”*", "Ý định thiết kế có, nhưng CameraLane chưa có tham số để nhận"],
        ],
        fracs=[0.24, 0.36, 0.40],
    )
    p("Cách nối đúng, không làm thay đổi giao thức UART: thêm tham số `speed_margin_cm` vào hàm khởi tạo hoặc `get_latest()`, rồi "
      "trong bộ lập tốc độ giảm một mức khi margin nhỏ. Như vậy CameraLane **đề xuất** tốc độ, còn ESP32 và ObstacleAvoidance vẫn là "
      "nơi quyết định cuối.")

    h2("11.5. Năm hằng số khai báo nhưng không dùng")
    mk_table(
        ["Hằng số", "Giá trị", "Nhận xét"],
        [
            ["`ROI_W`", "320", "Thừa: ROI tính động theo tỉ lệ theo chiều cao ảnh, không dùng chiều rộng cố định."],
            ["`ROI_H`", "200", "Thừa vì lý do trên."],
            ["`ENABLE_DEBUG_LOG`", "false", "Chưa dùng ở đâu; đáng giữ lại nếu sau này thêm log chẩn đoán."],
            ["`SPEED_CURVE_SLOPE`", "0,65", "Trùng giá trị với `CURVE_ENTER_SLOPE` – có thể gộp làm một."],
            ["`SPEED_SHARP_SLOPE`", "0,85", "Trùng giá trị với `SHARP_ENTER_SLOPE` – có thể gộp làm một."],
        ],
        fracs=[0.26, 0.14, 0.60],
    )
    p("Hai hằng số trùng lặp là điểm nên xử lý trước: hiện tại `calculate_target_speed` dùng bốn hằng số `CURVE_ENTER_*`/`SHARP_ENTER_*`, "
      "còn `SPEED_CURVE_SLOPE` và `SPEED_SHARP_SLOPE` không được dùng ở đâu. Rủi ro khi để lại: sau này sửa một bên, người đọc tưởng "
      "không có tác dụng và sửa nhầm bên kia.")

    h2("11.6. Ảnh gốc luôn được sao chép kể cả khi tắt visualize")
    code(rng(CPP, 659, 662), caption="cpp:659–662 – clone ảnh gốc không nằm trong if constexpr")
    p("Dòng 659 nằm **ngoài** khối `if constexpr (ENABLE_VISUALIZATION)`, nên `out.raw` luôn được `clone()` mỗi khung – một lần cấp phát "
      "và sao chép toàn bộ 640×400×3 byte. Với `ENABLE_VISUALIZATION = false`, node không dùng `raw` thì toàn bộ chi phí này là lãng phí.")
    bullets([
        "Cách sửa nhẹ nhất: thêm một hằng số `ENABLE_RAW_IMAGE` và bọc dòng 659 trong `if constexpr`, mặc định giữ `true` để không "
        "làm hỏng topic ảnh gốc đang chạy.",
        "Lưu ý: `raw` là dữ liệu ảnh được publish ra ngoài. Không được thu hẹp `cv::Mat` để tránh sao chép – "
        "dùng `copyTo` vào một `cv::Mat` tái sử dụng sẽ vỡ vì người gọi giữ con trỏ tới vùng nhớ cũ.",
    ])

    h2("11.7. calculate_slope yêu cầu 10 điểm trong khi bước kiểm tra chỉ đòi 5")
    p("`MIN_FINAL_POINTS = 5` nên một vạch còn 5–9 điểm vẫn được coi là hữu dụng và độ lệch vẫn được tính. Nhưng "
      "`calculate_slope` yêu cầu tối thiểu 10 điểm nên với các vạch đó `left_slope` và `right_slope` **luôn bằng 0**. Hệ quả: "
      "`dominant_slope = 0` nên máy trạng thái tốc độ không bao giờ thấy đường cong, dù đường vạch có thật.")
    p("Không phải lỗi nghiêm trọng vì khi thiếu vạch, nhánh một vạch đã loại bỏ bù đường cong; nhưng nên ghi rõ điều kiện này bằng "
      "chú thích trong header, hoặc hạ ngưỡng của `calculate_slope` xuống 5 để khớp với `MIN_FINAL_POINTS`.")

    h2("11.8. stop() có thể treo nếu driver camera kẹt")
    p("`stop()` dừng hai luồng bằng cờ và biến điều kiện, nhưng không có cách ngắt cuộc gọi `cap_.read()` đang chặn. Với V4L2 trên Linux "
      "thì read trở lại sau tối đa một chu kỳ khung, chấp nhận được; nhưng nếu driver hoặc thiết bị USB kẹt, `join()` sẽ treo vô hạn và "
      "node không tắt được.")
    p("Hướng khắc phục phổ biến là chạm vào thiết bị hoặc bật lại chế độ chốt băng của driver khi gỡ camera, hoặc dùng "
      "`select`/`poll` với độ trễ trên descriptor để đóng camera từ bên ngoài. Đây là hạn chế của thư viện `cv::VideoCapture`, "
      "không phải lỗi của logic module.")

    h2("11.9. clamp_float vô tình chặn luôn NaN – hành vi tốt nhưng chưa được ghi")
    code(rng(CPP, 308, 310), caption="cpp:308–310 – clamp viết bằng min/max")
    p("Với `value = NaN`: `std::min(NaN, max)` trả về `NaN` vì phép so sánh `<` với NaN luôn cho false; rồi `std::max(min, NaN)` "
      "lại trả về `min` – tức **NaN biến thành giá trị dưới cùng (0)**. Giá trị âm vô cùng cũng thành 0, dương vô cùng thành 20.")
    note([
        "Đây là hành vi đúng và mong muốn, nhưng là hệ quả kéo theo chứ không phải chủ ý viết ra.",
        "Nó là lớp phòng thủ thứ ba chống NaN, bên cạnh hai lần kiểm tra thủ công ở dòng 178–181 và 446–447.",
        "Nên thêm một dòng chú thích trong header để lập trình viên sau không vô tình viết lại clamp theo cách khác và làm mất lớp bảo vệ này.",
    ], fill="E8F1DE")

    h2("11.10. Những điểm đã thiết kế tốt – không nên sửa")
    bullets([
        "Đặt lại toàn bộ trạng thái đầu ra ở đầu hàm (dòng 415–427) trước mọi nhánh thoát sớm.",
        "`raw` và `vis` được `clone()` mỗi khung nên chia sẻ con trỏ giữa hai luồng là an toàn.",
        "Điều kiện `wait` có vẻ đánh giá lại, nên không bao giờ thức dậy oan.",
        "Join luồng đọc trước rồi mới đánh thức và join luồng xử lý – tránh bế tắc chéo.",
        "Giữ cờ `running_` là biến nguyên tử, và chuẩn hoá NaN ở cả phía nhận lẫn phía xử lý.",
        "Nhánh không hợp lệ xoá cờ khởi tạo tốc độ để không bị kẹp ở giá trị 0 khi có dữ liệu trở lại.",
    ])

    h2("11.11. Hướng mở rộng tự nhiên")
    mk_table(
        ["Hướng", "Cơ sở đã có trong code", "Công việc cần làm"],
        [
            ["Bám làn khi không có vạch", "Nhánh một vạch đã hoạt động",
             "Thay bề rộng ước lượng bằng bộ quan sát theo thời gian để không phụ thuộc `lane_width_est_px_`"],
            ["Nhận dạng màu vạch làn", "`lane_color_bgr`, `lane_color_name` đã khai báo trong `LaneOutput`",
             "Lấy mẫu màu tại các điểm đã tìm được rồi so sánh trong không gian HSV, phục vụ nhận dạng đèn giao thông"],
            ["Ưu tiên vạch trong khi né", "`is_dual_lane` đã được truyền sang `ObstacleAvoidance`",
             "Dùng `is_dual_lane` để chọn mép trong làm chuẩn bám khi lệch sang phải"],
            ["Hồi quy bậc hai cho cua mạnh", "Đã có điểm vạch theo y",
             "Thay hồi quy tuyến tính bằng bậc hai khi `|slope|` vượt ngưỡng; cần bảo vệ trước hiện tượng Runge"],
            ["Lọc theo thời gian Kalman", "Đã có `xmid_scan_` làm biến trạng thái",
             "Thay giới hạn dịch cứng bằng bộ lọc Kalman 1D cho trục quét và độ lệch"],
        ],
        fracs=[0.22, 0.34, 0.44],
    )

    h2("11.12. Thứ tự nên xử lý")
    mk_table(
        ["Ưu tiên", "Việc", "Rủi ro nếu bỏ qua"],
        [
            ["1 – Bắt buộc", "Sửa hai chỗ `dev_px` ở dòng 98 và 440", "Package không build được"],
            ["2 – Cao", "Đổi `median_value` nhận `const&`", "Lỗi tiềm ẩn, dễ nổ lên khi tái sử dụng hàm ở nơi khác"],
            ["3 – Cao", "Dùng `lo.is_dual_lane` thay vì tính lại ở dòng 441", "Quyết định né vật cản dựa trên cờ làn kép không chính xác"],
            ["4 – Trung bình", "Thêm `static_assert` cho `median_tail_x`, gộp hai hằng số trùng", "Nguy cơ sửa sai khi hiệu chỉnh sau này"],
            ["5 – Thấp", "Bọc `out.raw` trong `if constexpr`, giảm `calculate_slope` xuống 5 điểm", "Tốn băng thông; độ cong bị bỏ sót ở vạch ngắn"],
            ["6 – Thấp", "Xử lý treo khi đóng camera", "Node không tắt được, phải rút nguồn"],
        ],
        fracs=[0.16, 0.44, 0.40],
    )

    pagebreak()

    # ================================================================ 12
    h1("12. Kết luận")

    h2("12.1. Module này làm gì")
    p("`CameraLane` là lớp C++/OpenCV thuần, độc lập ROS, nhận ảnh từ `cv::VideoCapture` và biến mỗi khung thành ba thứ: **độ lệch so với "
      "tâm làn**, **một trong ba lệnh FWD/LEFT/RIGHT**, và **mức tốc độ đề xuất**. Thiết kế hai luồng với bộ đệm hai ô và chính sách "
      "latest-frame bảo đảm lệnh luôn dựa trên khung hình mới nhất, không bao giờ dồn hàng đợi.")

    h2("12.2. Những quyết định thiết kế đáng ghi nhận")
    mk_table(
        ["Quyết định", "Vì sao đúng"],
        [
            ["Quét dòng thay vì Hough", "Vạch làn là vật thể gần thẳng đứng; quét dòng nhanh hơn nhiều và không tạo ảnh trung gian."],
            ["Tách trục đo và trục quét", "Vừa bám được làn khi xe lệch, vừa đo được độ lệch thật."],
            ["Bù đường cong có dấu ngược", "Báo động trước khi vào cua, đánh đổi bằng việc giá trị lệch không còn mô tả đúng vị trí xe."],
            ["Hysteresis hai ngưỡng", "Vào trạng thái chậm dễ, ra khỏi trạng thái chậm khó – hướng an toàn."],
            ["Trung vị thay vì trung bình", "Chịu được nhiễu điểm lẻ, giữ được giá trị thật của đường vạch."],
            ["Đặt lại trạng thái ở đầu hàm", "Mọi nhánh thoát sớm đều cho kết quả hợp lệ."],
            ["Không dùng Hough, không ROS trong module", "Kiểm thử được độc lập, ghép được vào bất kỳ node nào."],
        ],
        fracs=[0.32, 0.68],
    )

    h2("12.3. Hạn chế cần biết")
    bullets([
        "Chưa có biên dịch thành công do lỗi `dev_px` ở `fusion_viz_node.cpp` dòng 98 và 440 – phải sửa trước khi chạy thử.",
        "Chưa có kiểm thử tự động: nên thêm bộ dữ liệu ảnh đã ghi sẵn và đo độ lớn độ lệch để kiểm tra đường cong và nhiễu.",
        "Tham số cứng trong header, không hiệu chỉnh được khi chạy – sẽ mất nhiều thời gian mỗi lần đổi.",
        "Giả định vạch làn **trắng hoặc sáng trên nền tối**. Vạch vàng trên nền sáng hoặc vạch mờ vào chiều sáng thấp sẽ làm Canny "
        "cho ít điểm biên.",
        "Một cặp vạch mỗi khung. Giao cắt làn hoặc làn rẽ sẽ làm `is_dual_lane` mất ổn định trong vài khung.",
    ])

    h2("12.4. Danh sách kiểm tra khi chạy thử trên xe")
    bullets([
        "Đặt `ENABLE_VISUALIZATION = true`, kiểm tra trên ảnh `vis`: đường trắng ở giữa ảnh, đường vàng bám làn, vạch đỏ, trung tâm xanh lá.",
        "Kiểm tra chữ `Dev: xxx px` trên telemetry đổi dấu đúng khi đưa xe sang trái rồi sang phải.",
        "Đo thời gian xử lý qua `processing_ms`; nếu lớn hơn thời gian một khung hình thì camera sẽ liên tục bỏ khung.",
        "Đo cân bằng trục trắng và trục vàng: lệch quá nhiều nghĩa là tham số bám làn cần hiệu chỉnh.",
        "Thử vào cua gấp và quan sát tốc độ có giảm đúng ba mức 8,5 → 6,0 → 4,5 km/h.",
        "Lấp tạm camera để kiểm tra nhánh không hợp lệ: lệnh phải về STOP, ROI phải không vẽ vạch nào, và khi bỏ tay ra phải tự "
        "trở lại FWD.",
        "Kiểm tra phản hồi khi vạch bị che một phần: `is_dual_lane` về false, trung tâm được suy luận từ vạch còn lại, và lệnh không "
        "giật.",
    ])

    h2("12.5. Tóm tắt một câu")
    note([
        "CameraLane là một module dòng làn nhỏ gọn nhưng đúng bài: quét dòng hai phía trục giữa, dùng trục quét có trí nhớ để bám làn,",
        "đo độ lệch bằng trung vị 5 điểm gần xe, cộng bù đường cong để báo động sớm, và đưa ra đề xuất tốc độ ba mức qua một máy trạng",
        "thái có hysteresis. Điểm cần sửa gấp là hai chỗ đọc trường dev_px không tồn tại trong fusion_viz_node.cpp.",
    ], fill="E8F1DE")

    p()
    p("— Hết tài liệu —", align="c", color="808080")