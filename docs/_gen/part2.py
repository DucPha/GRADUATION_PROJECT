# -*- coding: utf-8 -*-
from common import *


def render():
    h1("3. File 1 – camera_lane.hpp: cấu trúc khai báo")
    p("Header gồm 238 dòng, khai báo toàn bộ giao diện công khai của module: struct `LaneOutput` (hợp đồng đầu ra) và class "
      "`CameraLane` cùng toàn bộ hằng số cấu hình. Không có logic xử lý nào.")

    h2("3.1. Include và include guard")
    code(rng(HPP, 1, 12))
    mk_table(
        ["Dòng", "Nội dung", "Giải thích"],
        [
            ["1", "`#pragma once`",
             "Chống include trùng. Cách hiện đại hơn `#ifndef / #define / #endif` (2 dòng thay vì 3). Hạn chế còn lại: "
             "`#pragma once` không theo chuẩn ISO, một số compiler rất cũ không hỗ trợ."],
            ["3", "`#include <opencv2/opencv.hpp>`",
             "Nạp toàn bộ OpenCV, không chọn lọc submodule. Cũ xử lý nhưng rất tiện, đánh đổi bằng thời gian biên dịch. "
             "Không có `cv_bridge` hay `rclcpp` vì module này cố ý tách rời ROS."],
            ["5", "`#include <array>`", "Dùng cho `std::array<FrameSlot, 2>` (bộ đệm khung) và `std::array<int, 8>` trong hàm trung vị."],
            ["6", "`#include <atomic>`",
             "Dùng cho `std::atomic<bool> running_` và `std::atomic<float> current_speed_kmh_` – hai biến được hai luồng "
             "truy cập đồng thời."],
            ["7", "`#include <condition_variable>`", "Hai biến điều kiện `frame_ready_cv_` và `frame_free_cv_` để hai luồng ngủ và thức."],
            ["8", "`#include <cstdint>`", "Các kiểu cố định bề rộng: `int16_t`, `uint8_t`, `uint64_t`, `size_t`."],
            ["9", "`#include <mutex>`", "`frame_mtx_` bảo vệ bộ đệm khung; `output_mtx_` bảo vệ kết quả đầu ra."],
            ["10", "`#include <string>`", "Kiểu của `camera_cmd` (STOP, FWD, LEFT, RIGHT) và `lane_color_name`."],
            ["11", "`#include <thread>`", "Hai std::thread: luồng đọc camera và luồng xử lý."],
            ["12", "`#include <vector>`", "Các vector điểm `cv::Point` chứa vạch trái, vạch phải và trung tâm làn."],
        ],
        fracs=[0.06, 0.26, 0.68],
    )

    h2("3.2. struct LaneOutput – hợp đồng đầu ra")
    code(rng(HPP, 14, 42))
    p("Đây là toàn bộ dữ liệu mà module công bố ra ngoài. Nó chia làm bốn nhóm: đường vạch, lệnh, tốc độ, và nhóm chẩn đoán.")
    mk_table(
        ["Nhóm", "Dòng", "Trường", "Kiểu", "Ý nghĩa"],
        [
            ["Đường vạch", "19", "`left`", "`vector<cv::Point>`",
             "Các điểm (x, y) của vạch trái, theo thứ tự y tăng dần. Dùng để vẽ và để biết có nhìn thấy vạch trái hay không."],
            ["Đường vạch", "20", "`right`", "`vector<cv::Point>`", "Tương tự, cho vạch phải."],
            ["Đường vạch", "21", "`center`", "`vector<cv::Point>`",
             "Trung tâm làn, chỉ được tạo ở nhánh làn kép. Đây là hình học mà cả hệ thống thực sự cần."],
            ["Lệnh", "23", "`camera_cmd`", "`string` = \"STOP\"",
             "Lệnh dạng chữ cho telemetry và cho người đọc: STOP, FWD, LEFT, RIGHT. Mặc định là STOP – an toàn."],
            ["Lệnh", "25", "`dev_final_px`", "`int16_t` = 0",
             "**Trường quan trọng nhất.** Độ lệch ngang của xe so với trung tâm làn, tính bằng pixel. Dương là lệch phải, "
             "âm là lệch trái. Chính số này đi vào gói UART gửi xuống ESP32."],
            ["Tốc độ", "26", "`target_speed_x10`", "`uint8_t` = 0",
             "Tốc độ mục tiêu nhân 10, đơn vị km/h. 85 là 8,5 km/h (đường thẳng), 60 là 6 km/h (cua vừa), 45 là 4,5 km/h (cua gắt)."],
            ["Tốc độ", "38", "`speed_factor`", "`uint8_t` = 0",
             "Tốc độ chuẩn hoá 0–100 so với tốc độ tối đa 85×10. Dùng cho giao diện và để ghép tốc độ của nhiều nguồn."],
            ["Cờ", "27", "`valid`", "`bool` = false",
             "Cờ quan trọng nhất về mặt an toàn: chỉ khi true mới là có kết quả đáng tin. Khi false thì mọi số khác phải "
             "bị xem là vô nghĩa – và code đã bảo đảm điều đó."],
            ["Chẩn đoán", "32", "`pixels_used`", "`int` = 0",
             "Tổng số điểm của cả hai vạch. Chỉ số trực tiếp độ tin cậy: quá ít điểm là đang đo trên nhiễu."],
            ["Chẩn đoán", "36", "`curve_angle_deg`", "`float` = 0",
             "Góc của đường so với trục dọc, đơn vị độ, tính bằng `atan(slope)·180/π`. Là đại lượng dùng để bù đường cong."],
            ["Chẩn đoán", "37", "`curvature`", "`float` = 0", "Giá trị tuyệt đối của slope, tức độ cong không có dấu."],
            ["Chẩn đoán", "40", "`frame_id`", "`uint64_t` = 0",
             "Số thứ tự khung hình, đếm bằng `++next_frame_id_`. Dùng để phát hiện khung bị bỏ qua do latest-frame policy."],
            ["Chẩn đoán", "41", "`processing_ms`", "`float` = 0",
             "Thời gian xử lý một khung, đo bằng `chrono::steady_clock`. Con số để cân bằng độ phân giải với tốc độ."],
            ["Ảnh", "33", "`vis`", "`cv::Mat`",
             "Ảnh BGR đã vẽ đè: trục giữa ảnh (trắng), trục quét (vàng), vạch trái và phải (đỏ), trung tâm (xanh lá). "
             "Chỉ phục vụ telemetry."],
            ["Ảnh", "34", "`raw`", "`cv::Mat`", "Bản sao ảnh gốc, publish qua topic riêng để lưu hoặc xem lại."],
            ["Màu làn", "29", "`lane_color_bgr`", "`cv::Scalar`", "Màu BGR quan sát được của vạch, dự kiến cho nhận dạng đèn giao thông."],
            ["Màu làn", "30", "`lane_color_name`", "`string` = \"black\"",
             "Tên màu dạng chữ để hiển thị. Mặc định `black` khớp với thiết kế làm việc trên **vạch làn màu đen** trên nền sáng."],
        ],
        fracs=[0.09, 0.05, 0.15, 0.15, 0.56],
    )

    note([
        "Tất cả trường đều có giá trị khởi tạo mặc định tại khai báo: bằng 0, bằng false, hoặc \"STOP\".",
        "Điều này quan trọng vì detect_lanes() ghi đè lại toàn bộ ở đầu mỗi khung: nếu hàm thoát sớm ở giữa chừng, người gọi",
        "vẫn nhận được một struct hợp lệ (STOP, độ lệch bằng 0) chứ không phải dữ liệu rác.",
        "Lưu ý nhược điểm của việc đặt cv::Mat trong struct: khi chép struct, OpenCV chỉ tăng bộ đếm tham chiếu chứ KHÔNG copy",
        "dữ liệu ảnh. Ở module này điều đó lại đúng ý muốn – xem giải thích an toàn ở mục 4.7.",
    ])

    h2("3.3. Hằng số khung hình, đệm và cờ bật/tắt")
    code(rng(HPP, 52, 69))
    mk_table(
        ["Dòng", "Hằng số", "Giá trị", "Công dụng"],
        [
            ["52–53", "`FRAME_W`, `FRAME_H`", "640 × 400",
             "Kích thước khung hình **mong muốn**. Dùng làm giá trị yêu cầu cho camera và làm chuẩn quy đổi tỉ lệ. "
             "Chú ý: module **luôn** lấy kích thước thật từ `cap_.get()` chứ không giả định 640×400. Nếu camera trả về kích "
             "thước khác thì mọi ngưỡng tính bằng pixel đều được nhân với `width_scale = W / 640` để không bị lệch."],
            ["55–56", "`ROI_W`, `ROI_H`", "320 × 200",
             "**Không dùng.** Bản thân code quyết định ROI theo tỉ lệ với chiều cao ảnh, không dùng kích thước cố định. Có thể xoá."],
            ["59", "`FRAME_BUFFER_COUNT`", "2",
             "Số ô trong bộ đệm khung. 2 là số tối thiểu để luồng đọc và luồng xử lý chạy song song **không khoá nhau**: một ô "
             "đang xử lý, một ô đang chờ hoặc đang được ghi."],
            ["61", "`ENABLE_VISUALIZATION`", "true",
             "Cờ bật ảnh visualize. Dùng trong `if constexpr` nên khi đặt false thì **trình biên dịch loại hẳn khối code vẽ**, "
             "không tốn thời gian chạy. Bỏ trong bản chạy trên xe sẽ tiết kiệm một lần sao chép ảnh."],
            ["62", "`ENABLE_DEBUG_LOG`", "false", "**Không dùng.** Không có chỗ nào đọc cờ này."],
            ["65–66", "`CANNY_LOW`, `CANNY_HIGH`", "30 / 120",
             "Ngưỡng Canny, thấp hơn hẳn so với mặc định 50/150. Lý do: **vạch làn màu đen trên nền sáng tạo biên tương phản cao**, "
             "nên hạ ngưỡng để không bỏ sót vạch mờ; đổi lại dễ bị nhiễu hơn – nên bắt buộc có bước đóng lỗ hổng bằng hình thái ở ngay sau."],
            ["67", "`BLUR_KERNEL`", "5",
             "Kích thước kernel GaussianBlur 5×5, truyền sigma bằng 0 để OpenCV tự tính. Số lẻ là bắt buộc. "
             "Loại nhiễu cảm biến trước khi chạy Canny."],
            ["68", "`SCAN_STEP`", "4",
             "Quét mỗi 4 hàng một lần. Giảm 4 lần số lần đọc so với quét từng hàng nhưng vẫn giữ được đường vạch mượt. "
             "Với khung 640×400 và ROI bắt đầu ở y = 280 thì số dòng quét là (399 − 280)/4 + 1 = 30 dòng."],
            ["69", "`MEDIAN_TAIL_POINTS`", "5",
             "Số điểm cuối cùng, tức gần xe nhất, dùng để tính trung vị. Vị trí này quan trọng nhất vì phản ánh trạng thái "
             "ngay dưới bánh xe."],
        ],
        fracs=[0.07, 0.19, 0.10, 0.64],
    )

    h2("3.4. Hằng số ROI và ngưỡng số điểm")
    code(rng(HPP, 71, 91))
    mk_table(
        ["Dòng", "Hằng số", "Giá trị", "Công dụng"],
        [
            ["71", "`ROI_FACTOR_SLOW`", "0.70",
             "Tỉ lệ chiều cao xác định đường trên của ROI khi xe **chạy chậm**: 0,70 × 400 = y 280, vùng xử lý cao 120 px."],
            ["72", "`ROI_FACTOR_FAST_MIN`", "0.55",
             "Tỉ lệ nhỏ nhất khi tốc độ cao nhất, ROI cao nhất = 180 px. Nếu không giới hạn dưới thì khi chạy nhanh con số sẽ âm và ROI vô nghĩa."],
            ["73", "`ROI_SPEED_START`", "4,0 km/h",
             "Ngưỡng bắt đầu mở rộng ROI. Dưới ngưỡng này coi như tốc độ thấp, giữ ROI hẹp để tăng độ chính xác gần xe."],
            ["74", "`ROI_SPEED_GAIN`", "0.02",
             "Độ dốc của phép nội suy ROI theo tốc độ. Mỗi 1 km/h vượt ngưỡng thì ROI cao thêm 0,02 × 400 = 8 px."],
            ["75", "`SINGLE_ROI_FACTOR`", "0.85",
             "ROI khi chỉ thấy một vạch: 0,85 × 400 = y 340, cao 60 px. Hẹp hơn nhiều và sát xe hơn, vì lúc này trung tâm làn "
             "phải **suy luận** từ bề rộng làn chứ không quan sát trực tiếp, nên cần vùng gần xe – nơi bề rộng ổn định nhất."],
            ["77", "`MIN_PRELIM_POINTS`", "15",
"Ngưỡng cho khâu **sơ bộ**, tức số điểm thu được khi quét toàn vùng ROI kép. Với 30 dòng quét, ngưỡng này "
              "đòi hỏi vạch hiện diện ở ít nhất một nửa số dòng."],
            ["78", "`MIN_FINAL_POINTS`", "5",
             "Ngưỡng cho khâu **cuối cùng**, sau khi cắt theo `y0_used`. Thấp hơn có chủ ý: ở nhánh một vạch, ROI chỉ còn "
             "15 dòng quét."],
            ["79", "`MIN_COMMON_POINTS`", "10",
             "Số dòng phải có **cả hai vạch** đồng thời hợp lệ về bề rộng. Đây là tiêu chí mạnh nhất để tuyên bố đây là làn kép. "
             "Nếu chỉ thấy hai vạch ở những dòng rời rạc thì đó có thể là nhiễu, không phải làn."],
            ["81", "`MIN_LANE_WIDTH_PX`", "200",
             "Bề rộng làn tối thiểu, quy đổi theo `width_scale`. Nếu thấy hai vạch cách nhau 40 px thì rất có thể đó là **hai mép "
             "của cùng một vạch** do Canny sinh cạnh đôi, hoặc là một vạch làn hẹp trong ảnh phối cảnh."],
            ["82", "`MAX_LANE_WIDTH_PX`", "500", "Bề rộng làn tối đa – chặn việc khóa nhầm vào hai vạch ở rất xa."],
            ["83", "`DEFAULT_LANE_WIDTH_PX`", "230",
             "Bề rộng giả định ban đầu, cũng là giá trị khởi tạo của bộ lọc. Được hiệu chỉnh lại ở dòng 128 của .cpp theo tỉ lệ "
             "kích thước ảnh thật."],
            ["84", "`ALPHA_LANE_WIDTH`", "0.20",
             "Hệ số EMA cho bề rộng làn: mới = 0,2 × đo được + 0,8 × giá trị cũ. Hệ số nhỏ vì bề rộng làn thay đổi chậm, "
             "chọn nhỏ để ổn định."],
            ["86", "`FWD_THRESHOLD_PX`", "15",
             "Độ lệch tuyệt đối dưới 15 px thì coi như **đi thẳng**. Chừa vùng chết khoảng 15 px để tránh xe liên tục đánh lái "
             "trái–phải khi đang bám làn tốt (hiện tượng chattering)."],
            ["88–91", "`SLOPE_OFFSET_START/END/MIN_PX/MAX_PX`", "0,80 / 1,00 / 50 / 120",
             "Tham số bù đường cong. Dưới slope 0,80 thì bằng 0; slope từ 1,00 trở lên thì bù tối đa 120 px; giữa đoạn nội suy tuyến tính. "
             "Chi tiết ở mục 9.4."],
        ],
        fracs=[0.07, 0.20, 0.11, 0.62],
    )

    h2("3.5. Hằng số bộ lập tốc độ và hysteresis")
    code(rng(HPP, 93, 116))
    mk_table(
        ["Dòng", "Hằng số", "Giá trị", "Công dụng"],
        [
            ["99", "`SPEED_STRAIGHT_X10`", "85", "Đường thẳng: **8,5 km/h**. Đồng thời là mẫu chuẩn để tính `speed_factor` (100 là 85×10)."],
            ["100", "`SPEED_CURVE_X10`", "60", "Cua vừa: **6,0 km/h**."],
            ["101", "`SPEED_SHARP_X10`", "45",
             "Cua gắt: **4,5 km/h**. Tốc độ giảm theo độ cong – nguyên tắc hãm tốc trước khi vào cua."],
            ["103–104", "`SPEED_CURVE_SLOPE`, `SPEED_SHARP_SLOPE`", "0,65 / 0,85",
             "**Không dùng.** Bị thay bằng cặp ngưỡng vào và ra của hysteresis ở dòng 108–116."],
            ["105", "`ALPHA_TARGET_SPEED`", "0.70",
             "Hệ số lọc mềm tốc độ: mới = 0,7 × giá trị yêu cầu + 0,3 × giá trị hiện tại. Hệ số **lớn** (0,7), khác với "
             "bộ lọc bề rộng làn (0,2), vì tốc độ phải phản ứng nhanh để kịp hãm trước cua."],
            ["108–111", "`CURVE_ENTER_SLOPE`, `CURVE_EXIT_SLOPE`, `SHARP_ENTER_SLOPE`, `SHARP_EXIT_SLOPE`",
             "0,65 / 0,55 / 0,85 / 0,75",
             "Ngưỡng slope để **vào** và **thoát** trạng thái. Chênh lệch giữa cặp vào và ra chính là vùng chống chập chờn: "
             "mỗi cặp cách nhau 0,10, tương đương một vùng đệm ngưỡng."],
            ["113–116", "`CURVE_ENTER_DEV_PX`, `CURVE_EXIT_DEV_PX`, `SHARP_ENTER_DEV_PX`, `SHARP_EXIT_DEV_PX`",
             "50 / 40 / 80 / 65",
             "Ngưỡng độ lệch (pixel) cho cùng mục đích. Nhờ điều kiện là **hoặc** (||), khi xe đang lệch lớn thì sẽ hạ tốc độ "
             "**ngay cả khi đường thẳng** – đây là cơ chế an toàn: mất cân bằng thì cũng phải chậm lại."],
        ],
        fracs=[0.09, 0.24, 0.13, 0.54],
    )

    p("Sự khác biệt số học giữa cặp ngưỡng là điểm cốt lõi của hysteresis. Nếu cả ngưỡng vào và ngưỡng ra đều là 0,65 thì khi xe "
      "bán kính cua dao động quanh ngưỡng, trạng thái sẽ bật/tắt liên tục mỗi khung hình và tốc độ sẽ nhảy 85 ↔ 60 liên tục. "
      "Bằng cách đặt ngưỡng ra thấp hơn ngưỡng vào (0,55 < 0,65), trạng thái phải đi xa hơn một chút mới thoát – hành vi ổn định hơn nhiều.")

    h2("3.6. enum class SpeedState")
    code(rng(HPP, 118, 122))
    p("Kiểu enum có phạm vi (scoped enum) với kiểu nền `uint8_t` nên chiếm đúng **một byte**. Ba giá trị tương ứng ba mức tốc độ. "
      "Dùng `enum class` thay vì `enum` để không rò tên hằng ra phạm vi toàn cục và bắt buộc ghi rõ `SpeedState::` khi dùng – "
      "giúp tránh nhầm với các enum khác trong project.")

    h2("3.7. Hàm thành viên công khai")
    code(rng(HPP, 124, 140))
    mk_table(
        ["Dòng", "Hàm", "Ý nghĩa"],
        [
            ["124–128", "`explicit CameraLane(int device_index = 0, int fps = 120, bool use_v4l2 = true)`",
             "Constructor. Tất cả tham số có giá trị mặc định nên có thể gọi `CameraLane()` rỗng. Từ khoá `explicit` cấm chuyển "
             "đổi ngầm từ số nguyên – ngăn `CameraLane c = 0;` là lỗi ý nghĩa. Lưu ý mặc định fps = 120: đây là fps **yêu cầu**, "
             "driver thường tự hạ xuống mức hợp lý."],
            ["130", "`~CameraLane()`", "Destructor, chỉ gọi `stop()`. Bảo đảm không còn luồng nào chạy khi đối tượng bị huỷ."],
            ["132", "`bool start()`",
             "Mở camera, cấp phát bộ đệm, đặt lại toàn bộ trạng thái, khởi động hai luồng. **Idempotent**: gọi lần hai khi đang "
             "chạy sẽ trả về true ngay. Trả false nếu mở camera thất bại."],
            ["133", "`void stop()`",
             "Dừng an toàn: hạ cờ `running_`, đánh thức cả hai biến điều kiện để luồng ngủ thoát, `join()` cả hai luồng, đóng camera. "
             "Cũng xử lý được tình huống gọi `stop()` khi chưa từng `start()`."],
            ["135–138", "`bool get_latest(LaneOutput& out, float current_speed_kmh = 0.0f)`",
             "Hàm chính để node tiêu thụ. Ghi tốc độ hiện tại vào biến nguyên tử rồi trả về một bản sao kết quả khung vừa xử lý xong. "
             "Trả false nếu chưa có khung nào xong. **Không bao giờ chặn** – đây là điểm mấu chốt của thiết kế hai luồng."],
            ["140", "`size_t buffered_frames() const`",
             "Số khung đang chờ xử lý, tức 0 hoặc 1. Dùng để chẩn đoán: giá trị 1 liên tục nghĩa là máy tính chậm hơn camera "
             "và đang phải bỏ khung liên tục."],
        ],
        fracs=[0.08, 0.30, 0.62],
    )

    h2("3.8. Hàm thành viên riêng")
    code(rng(HPP, 144, 187))
    mk_table(
        ["Dòng", "Hàm", "Ý nghĩa"],
        [
            ["144", "`void capture_thread()`", "Thân luồng đọc camera. Vòng lặp: chờ ô trống, `cap_.read()`, đánh số khung, báo có khung mới."],
            ["145", "`void process_thread()`", "Thân luồng xử lý. Vòng lặp: chờ khung, `detect_lanes()`, ghi kết quả, trả ô về."],
            ["147–151", "`void detect_lanes(const cv::Mat& bgr, float current_speed_kmh, LaneOutput& out)`",
             "Hàm cốt lõi, chiếm gần 260 dòng (414–672). Nhận ảnh **tham chiếu** nên không copy, nhận tốc độ, và ghi kết quả ra `out`."],
            ["153–155", "`static float calculate_slope(const vector<cv::Point>&)`",
             "Hồi quy tuyến tính theo phương y để suy ra hệ số **dx/dy**. Khai báo `static` nên không cần đối tượng."],
            ["157–160", "`static int median_tail_x(const vector<cv::Point>&, int tail_count)`",
             "Trung vị toạ độ x của `tail_count` điểm cuối cùng, tức gần xe nhất. Tham số là `const&` nên không sửa vector đầu vào."],
            ["162–164", "`static int median_value(std::vector<int>& values)`",
             "Trung vị bằng `nth_element`, phức tạp trung bình O(n). **Tham số không const** vì `nth_element` sắp xếp lại dãy một phần."],
            ["166–168", "`static void median_smooth(std::vector<cv::Point>& pts)`",
             "Lọc trung vị 3 điểm **tại chỗ** trên toạ độ x, loại bỏ nhiễu điểm lẻ trong khi giữ nguyên thứ tự các điểm."],
            ["170–172", "`static int16_t clamp_int16(int value)`", "Chặn giá trị về đúng miền của int16_t trước khi ép kiểu, tránh tràn đổ dấu."],
            ["174–178", "`static float clamp_float(float, float, float)`", "Chặn một số thực trong khoảng [min, max]. Dùng rất nhiều trong module."],
            ["180–182", "`static int calculate_slope_offset(float dominant_slope)`",
             "Biến độ cong thành lệch bù cộng thêm, trong khoảng −120…+120 px. Cơ chế **bù sớm đường cong**."],
            ["184–187", "`uint8_t calculate_target_speed(float dominant_slope, int abs_deviation)`",
             "Máy trạng thái ba mức: đọc trạng thái hiện tại, quyết định trạng thái kế tiếp theo ngưỡng vào và ra, trả tốc độ ứng với "
             "trạng thái mới. Không phải `static` vì nó **sửa trạng thái** `speed_state_`."],
        ],
        fracs=[0.08, 0.30, 0.62],
    )

    h2("3.9. Cấu trúc FrameSlot và bộ đệm khung hình")
    code(rng(HPP, 199, 215))
    p("Mỗi ô đệm là một cặp `(cv::Mat image, uint64_t frame_id)`. Mảng `frame_pool_` là `std::array` cố định hai phần tử – "
      "không cấp phát động, không có con trỏ hỏng. Việc đánh số khung đi kèm ảnh giúp module biết chính xác mình đang xử lý khung nào, "
      "đồng thời cho phép bên ngoài phát hiện khung bị bỏ qua.")

    p("Ba con trỏ chỉ số điều phối bộ đệm:")
    mk_table(
        ["Biến", "Giá trị", "Ý nghĩa"],
        [
            ["`latest_slot_`", "0, 1 hoặc −1",
             "Ô đang chứa **khung mới nhất, chưa ai xử lý**. −1 nghĩa là hàng đợi rỗng. Khi khung mới đến, giá trị cũ bị thay thế – "
             "chính là chỗ hiện thực chính sách latest-frame."],
            ["`processing_slot_`", "0, 1 hoặc −1", "Ô đang được luồng xử lý mượn. Luồng đọc không bao giờ chạm vào ô này."],
            ["(điều kiện)", "`i != processing_slot_ && i != latest_slot_`",
             "Điều kiện duy nhất quyết định ô nào còn trống. Với hai ô và tối đa hai ô đang bị chiếm thì luôn còn đúng một ô "
             "trống – đây là lý do `FRAME_BUFFER_COUNT` không thể đặt là 1."],
        ],
        fracs=[0.16, 0.15, 0.69],
    )

    p("`frame_mtx_` bảo vệ ba biến trên cùng lúc; `output_mtx_` là mutex riêng cho kết quả. Tách hai mutex là cần thiết: "
      "nếu dùng chung, lúc `get_latest()` đang chép kết quả sẽ chặn luồng đọc camera – mất đúng thứ độ trễ mà thiết kế này cố tránh.")

    h2("3.10. Toàn bộ biến thành viên")
    code(rng(HPP, 189, 238))
    mk_table(
        ["Dòng", "Biến", "Nhóm", "Ý nghĩa và lý do tồn tại"],
        [
            ["189–191", "`device_index_`, `fps_`, `use_v4l2_`", "Cấu hình",
             "Ba tham số constructor được lưu lại để `start()` dùng. Cần giữ vì `start()` có thể được gọi lại sau `stop()`."],
            ["193", "`cap_`", "Cấu hình", "`cv::VideoCapture` – đã mở sẵn trong `start()`."],
            ["195", "`running_{false}`", "Điều khiển",
             "Cờ dừng, kiểu `atomic<bool>`. Cả hai luồng đều đọc nó ở đầu vòng lặp và trong biểu thức điều kiện của "
             "`condition_variable::wait`."],
            ["196–197", "`capture_thread_`, `process_thread_`", "Điều khiển",
             "Hai đối tượng luồng; phải `join()` trước khi đối tượng bị huỷ, nếu không chương trình sẽ gọi `std::terminate()`."],
            ["209", "`frame_mtx_`", "Đồng bộ", "`mutable` vì được `lock()` trong hàm `const` (`buffered_frames`)."],
            ["210–211", "`frame_ready_cv_`, `frame_free_cv_`", "Đồng bộ", "Hai điều kiện ngược chiều: có-khung-mới và có-ô-trống."],
            ["213", "`output_mtx_`", "Đồng bộ", "Bảo vệ cặp `latest_output_` và `output_ready_`."],
            ["214", "`latest_output_{}`", "Kết quả", "Kết quả khung vừa xong, sẵn sàng cho `get_latest()`."],
            ["215", "`output_ready_ = false`", "Kết quả",
             "Khởi tạo false để lời gọi `get_latest()` đầu tiên, trước khi có khung nào, trả về false thay vì trả struct rỗng."],
            ["217", "`current_speed_kmh_{0.0f}`", "Điều khiển",
             "`atomic<float>` được đặt bởi `get_latest()` và đọc bởi `detect_lanes()` – đây là đường truyền tốc độ từ ESP32 vào module."],
            ["219", "`xmid_scan_`", "Thị giác",
             "Trục quét hiện tại, mặc định là giữa ảnh. **Biến thành viên** vì phải nhớ qua các khung hình: đây chính là bộ nhớ của "
             "thuật toán, cho phép trục quét bám theo làn thay vì luôn ở giữa ảnh."],
            ["220", "`lane_width_est_px_`", "Thị giác",
             "Ước lượng bề rộng làn bằng bộ lọc EMA. Cần nhớ qua các khung vì nhánh một vạch dựa vào nó để suy luận trung tâm làn."],
            ["221", "`target_speed_x10_filtered_`", "Tốc độ", "Giá trị tốc độ sau lọc mềm, ở dạng float trung gian."],
            ["222", "`target_speed_initialized_ = false`", "Tốc độ",
             "Cờ phân biệt chưa có mẫu nào với mẫu bằng 0. Nếu không có cờ này thì lần đầu tiên sẽ lọc với giá trị khởi tạo 0, "
             "khiến tốc độ chạy về 0 rồi mới tăng dần."],
            ["223", "`speed_state_ = STRAIGHT`", "Tốc độ", "Trạng thái hiện tại của máy trạng thái; cũng bị đặt lại khi mất tín hiệu làn."],
            ["225–228", "`gray_`, `blur_`, `edges_`, `morph_kernel_`", "Tái sử dụng bộ nhớ",
             "**Tối ưu quan trọng nhất của module.** Thay vì tạo `cv::Mat` cục bộ mỗi khung, tái sử dụng các thành viên này nên "
             "OpenCV chỉ cấp phát lại khi kích thước thay đổi. `morph_kernel_` tạo một lần trong constructor vì nó không đổi theo khung."],
            ["230–234", "`leftPts_prelim_`, `rightPts_prelim_`, `leftPts_`, `rightPts_`, `prelim_centers_`", "Tái sử dụng bộ nhớ",
             "Vector điểm. Bản `_prelim_` là kết quả quét toàn vùng ROI kép, bản còn lại giữ các điểm đã cắt theo `y0_used`. "
             "Mọi vector đều `reserve(128)` ở constructor để không cấp phát lại trong vòng lặp."],
            ["236", "`work_output_{}`", "Tái sử dụng bộ nhớ",
             "Struct kết quả mà luồng xử lý điền vào, rồi chép sang `latest_output_`. Tách hai bản cho phép luồng xử lý tiếp tục làm "
             "việc với bản riêng trong khi node đang đọc bản đã công bố."],
            ["237", "`next_frame_id_ = 0`", "Bộ đếm", "Bộ đếm khung, chỉ luồng đọc camera tăng. Không cần atomic vì vị trí này an toàn."],
        ],
        fracs=[0.08, 0.22, 0.15, 0.55],
    )

    pagebreak()
