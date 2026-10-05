# -*- coding: utf-8 -*-
from common import *


def render():
    h1("4. File 2 – camera_lane.cpp: cấu trúc cài đặt")
    p("File cài đặt 672 dòng, theo đúng thứ tự: constructor, destructor, start, stop, get_latest, capture_thread, "
      "process_thread, nhóm utilities, rồi detect_lanes.")

    h2("4.1. Constructor")
    code(rng(CPP, 15, 40))
    p("Constructor **không mở camera và không tạo luồng**. Nó chỉ chuẩn bị bộ nhớ:")
    bullets([
        "Dòng 20–23: khởi tạo ba biến cấu hình bằng danh sách khởi tạo và đặt `xmid_scan_` ở giữa ảnh.",
        "Dòng 25–37: `reserve(128)` cho **9 vector**. Đây là kỹ thuật then chốt của module gần như zero-allocation: sau khi "
        "reserve, các lần `push_back` sau này **không cấp phát bộ nhớ nữa**. Riêng bốn vector điểm tăng theo chiều sâu nên không "
        "cần reserve, nhưng chỉ tăng từ 128 lên 129 một lần rồi đứng yên.",
        "Dòng 39: tạo kernel hình thái 3×3 **một lần duy nhất**, lưu vào thành viên để dùng lại ở mọi khung hình.",
    ])
    note([
        "Giá trị 128 được chọn có chủ ý: số dòng quét lớn nhất xảy ra khi xe chạy nhanh nhất là (399 − 220)/4 + 1 = 45 dòng.",
        "Nên 128 là dư, đủ để không bao giờ cấp phát lại trong mọi tình huống camera hợp lệ.",
    ])

    h2("4.2. Destructor")
    code(rng(CPP, 46, 48))
    p("Chỉ gọi `stop()`. Với `std::thread`, nếu bỏ qua `join()` thì khi đối tượng bị huỷ chương trình sẽ gọi "
      "`std::terminate()` và sập. Vì vậy destructor gọi `stop()` là bắt buộc về mặt an toàn chương trình.")

    h2("4.3. start() – mở camera và khởi động hai luồng")

    h3("Giai đoạn A – Chốt sớm nếu đang chạy (dòng 54–65)")
    code(rng(CPP, 54, 65))
    p("Dòng 55–56 là **tính lũy đẳng (idempotent)**: gọi `start()` lần hai sẽ không mở camera lần nữa và không tạo luồng mới – "
      "việc tạo luồng mới khi luồng cũ còn sống sẽ khiến chương trình bị kết thúc ngay lập tức. Dòng 60 chọn backend: V4L2 trên Linux "
      "(định dạng MJPEG tốt hơn, độ trễ thấp hơn), `CAP_ANY` nếu tắt. Dòng 62–65 trả false ngay khi không mở được, để node chỉ cảnh báo.")

    h3("Giai đoạn B – Thiết lập thuộc tính camera (dòng 67–84)")
    code(rng(CPP, 67, 84))
    mk_table(
        ["Dòng", "Code", "Giải thích"],
        [
            ["67–68", "`set(CAP_PROP_FRAME_WIDTH/HEIGHT, FRAME_W/FRAME_H)`", "Yêu cầu khung 640×400."],
            ["69", "`set(CAP_PROP_FOURCC, fourcc('M','J','P','G'))`",
             "Ép **MJPEG**. Camera USB thường mặc định trả khung YUYV ở 30–60 MB/s, dễ nghẽn cổng USB và làm giảm FPS. MJPEG nén "
             "ảnh ngay ở tầng camera nên băng thông chỉ vài MB/s."],
            ["70", "`set(CAP_PROP_FPS, fps_)`", "Yêu cầu tần số lấy ảnh."],
            ["71", "`set(CAP_PROP_BUFFERSIZE, 1)`",
             "Driver chỉ giữ **một khung** trong bộ đệm phía cạnh. Rất quan trọng: nếu driver giữ năm khung thì `cap_.read()` sẽ "
             "trả về khung cũ nhất trong hàng đợi – nghĩa là độ trễ không kiểm soát được và chính sách latest-frame vô hiệu hoá."],
            ["72–75", "`#ifdef _WIN32` → `set(CAP_PROP_AUTO_EXPOSURE, 0.25)`, `set(CAP_PROP_EXPOSURE, -6)`",
             "Chỉ chạy khi biên dịch trên Windows, tức để test trên máy tính. Khoá **auto-exposure** và chốt mức phơi sáng để ảnh "
             "đủ sáng ổn định, tránh Canny nhấp nháy theo độ sáng. Giá trị 0,25 là chế độ thủ công trên V4L2 của Windows; −6 là mức "
             "log2 của thời gian phơi sáng."],
            ["77–79", "Đọc lại `CAP_PROP_FRAME_WIDTH/HEIGHT/FPS`", "Hỏi camera xem nó **thực sự** cho ra gì. Driver có thể bỏ qua yêu cầu."],
            ["81–82", "`pool_w`, `pool_h`", "Nếu driver trả về giá trị không dương (không hỗ trợ truy vấn) thì lùi về `FRAME_W` và `FRAME_H`."],
            ["84", "In ra log kích thước thật", "Xác nhận chế độ làm việc trên vạch làn màu đen."],
        ],
        fracs=[0.07, 0.33, 0.60],
    )

    h3("Giai đoạn C – Cấp phát bộ đệm và đặt lại kết quả (dòng 86–115)")
    code(rng(CPP, 86, 95))
    p("Dùng `std::lock_guard` cho khối này vì nó chạm vào `latest_slot_` và `processing_slot_` – về lý thuyết có thể trùng với "
      "luồng đọc đang chạy. `cv::Mat::create` cấp phát vùng nhớ **không khởi tạo**: nhanh hơn `Mat::zeros` và nội dung sẽ bị "
      "`cap_.read()` ghi đè toàn bộ ngay sau đó.")
    code(rng(CPP, 97, 115))
    p("Khối thứ hai đặt lại toàn bộ `latest_output_` về trạng thái STOP và gọi `release()` cho hai `cv::Mat` để **giải phóng bộ nhớ ảnh "
      "của kết quả cũ** trước khi chạy phiên mới. `output_ready_ = false` bảo đảm `get_latest()` trả false cho tới khi có khung mới.")

    h3("Giai đoạn D – Đặt lại thuật toán và cân bộ theo kích thước thật (dòng 117–133)")
    code(rng(CPP, 117, 133))
    bullets([
        "Dòng 117–125: xoá các vector và `release()` ba `cv::Mat` trung gian – giải phóng bộ nhớ cấp cho phiên trước.",
        "**Dòng 128 là dòng quan trọng**: `lane_width_est_px_ = 230 × (pool_w / 640)`. Nếu camera thực sự trả về 1280×720 thì bề rộng "
        "làn mặc định lập tức thành 460 px thay vì 230. Bản chất: **mọi ngưỡng viết bằng pixel trong header đều được hiểu là pixel ở "
        "chiều rộng 640**, và code luôn nhân với `width_scale` để quy đổi.",
        "Dòng 129–133: đặt lại bộ lọc tốc độ, cờ khởi tạo, trạng thái máy trạng thái, tốc độ hiện tại và bộ đếm khung.",
    ])

    h3("Giai đoạn E – Khởi động luồng (dòng 135–139)")
    code(rng(CPP, 135, 140))
    note([
        "Dòng 135 đặt running_ = true TRƯỚC khi tạo luồng. Thứ tự này bắt buộc: nếu tạo luồng trước, luồng mới sinh ra sẽ kiểm tra",
        "while (running_) với giá trị false và thoát ngay lập tức – start() trả về true nhưng không có luồng nào chạy.",
        "Ngược lại, stop() phải đặt running_ = false rồi đánh thức cả hai biến điều kiện, nếu không các luồng sẽ ngủ vĩnh viễn và",
        "stop() treo ở join().",
    ])

    h2("4.4. stop() – dừng an toàn")
    code(rng(CPP, 146, 172))
    mk_table(
        ["Dòng", "Ý nghĩa"],
        [
            ["147–152", "Nhánh **gọi stop() khi chưa chạy**: chỉ join nếu joinable và đóng camera. Cần thiết vì `~CameraLane()` gọi "
                        "`stop()` và có thể chạy cả khi `start()` thất bại – trường hợp này luồng vẫn phải được dọn sạch."],
            ["154–156", "Cờ dừng rồi đánh thức **cả hai** biến điều kiện. Đánh thức `frame_ready_cv_` để luồng xử lý thoát; đánh thức "
                        "`frame_free_cv_` để luồng đọc thoát khỏi lần chờ ô trống."],
            ["158–159", "Join **luồng đọc trước**. Thứ tự này có chủ ý: khi luồng đọc đã dừng thì không còn khung mới nào được tạo ra, "
                        "nên `latest_slot_` sẽ không đổi giữa lúc luồng xử lý kiểm tra và lúc nó gỡ ô ra."],
            ["160–161", "Đánh thức lại rồi join luồng xử lý – trường hợp luồng xử lý đang ngủ trong `frame_ready_cv_.wait()`."],
            ["163", "Đóng camera."],
            ["165–169", "Đặt hai chỉ số ô về −1 dưới khoá để đối tượng sẵn sàng nếu gọi `start()` lại."],
            ["171", "In log kết thúc."],
        ],
        fracs=[0.10, 0.90],
    )
    note([
        "Một giới hạn cần biết: stop() không có cách ngắt cuộc gọi cap_.read() đang bị chặn. Với V4L2 trên Linux, read() trở lại",
        "sau tối đa một chu kỳ khung hình, khoảng 33 ms ở 30 FPS, nên join() kết thúc nhanh và chấp nhận được.",
        "Nhưng nếu driver camera kẹt thì stop() sẽ treo – đây là nguyên nhân kinh điển của các sự cố treo chương trình khi tắt.",
    ])

    h2("4.5. get_latest() và buffered_frames()")
    code(rng(CPP, 178, 193))
    bullets([
        "Dòng 179–181: chuẩn hoá tốc độ **trước khi** ghi vào biến nguyên tử – loại NaN (do phép chia 0/0 ở nơi khác) và chặn trong "
        "khoảng 0–20 km/h. Đây là biện pháp phòng thủ chống NaN lan truyền vào toàn bộ phép tính ROI.",
        "Dòng 183–184: khoá `output_mtx_` rồi kiểm tra `output_ready_`. Trả false nếu chưa có khung nào xong – đây là giá trị để "
        "node biết camera chưa sẵn sàng.",
        "Dòng 186: `out = latest_output_` – **chép cả struct**. Các `cv::Mat` chỉ tăng bộ đếm tham chiếu chứ không copy dữ liệu ảnh, "
        "nên phép chép vẫn rẻ.",
        "Dòng 192: `buffered_frames()` trả 1 nếu có khung đang chờ, ngược lại 0. Biến `mutable std::mutex` cho phép khoá trong hàm `const`.",
    ])

    h2("4.6. capture_thread() – luồng đọc camera")
    code(rng(CPP, 199, 252), caption="cpp:199–252 – toàn bộ luồng đọc")
    mk_table(
        ["Dòng", "Tác dụng cụ thể"],
        [
            ["200", "`while (running_)` – điều kiện vòng lặp. Luồng tự thoát khi cờ dừng được bật."],
            ["204", "`unique_lock` chứ không phải `lock_guard`, vì `wait()` cần mở khoá trong lúc ngủ."],
            ["205–214", "Điều kiện chờ của `frame_free_cv_.wait()`: trả về true (thức dậy) khi **đã dừng** hoặc **tồn tại ô không "
                        "phải `processing_slot_` lẫn không phải `latest_slot_`**. Đây là điều kiện có đánh giá lại – `wait()` tự kiểm "
                        "tra lại trước mỗi lần ngủ nên không bao giờ thức dậy oan."],
            ["216", "Kiểm tra lại `running_` sau khi thức dậy, tránh xử lý thêm một vòng nữa."],
            ["218–226", "Quét hai ô để tìm ô trống và gán `slot_index`. Về lý thuyết luôn tìm thấy, nhưng vẫn có "
                        "`if (slot_index < 0) continue;` để an toàn – tránh truy cập `frame_pool_[-1]`."],
            ["229", "Tham chiếu tới `cv::Mat` của ô đã chọn. `cap_.read(frame)` sẽ ghi đè đúng bộ nhớ này."],
            ["230–233", "`cap_.read(frame)` **cố ý không giữ khoá** – đây là lý do tách hai luồng. Nếu giữ khoá, luồng xử lý sẽ bị "
                        "chặn trong suốt thời gian chờ khung hình mới. Nếu đọc lỗi hoặc ảnh rỗng thì ngủ 1 ms rồi thử lại để tránh "
                        "quay vòng cháy CPU."],
            ["235", "`const uint64_t frame_id = ++next_frame_id_;` – đánh số khung. Chỉ luồng này tăng biến đếm nên không cần khoá."],
            ["238–247", "Khoá lại để công bố khung. Dòng 241–243 là **trái tim chính sách latest-frame**: ô `latest_slot_` cũ bị "
                        "thay thế ngay, nếu có khung nào đang chờ thì nó bị bỏ mà không ai xử lý."],
            ["249–250", "Báo cho luồng xử lý (`frame_ready_cv_.notify_one`) và báo cho chính nó rằng ô cũ đã trống "
                        "(`frame_free_cv_.notify_one`)."],
        ],
        fracs=[0.08, 0.92],
    )
    note([
        "Vì sao thay thế luôn chứ không xếp hàng: nếu xếp hàng, khi máy tính chậm hơn camera thì lệnh lái sẽ dựa trên khung cũ.",
        "Với xe chạy 8,5 km/h = 2,36 m/s, chậm 200 ms là sai lệch 47 cm – vượt cả bề rộng làn. Bỏ khung là quyết định đúng:",
        "thông tin cũ không còn giá trị, thông tin mới mới dùng được. Đây là lý do bộ đệm chỉ cần hai ô thay vì một hàng đợi dài.",
    ])

    h2("4.7. process_thread() – luồng xử lý ảnh")
    code(rng(CPP, 258, 302), caption="cpp:258–302 – toàn bộ luồng xử lý")
    mk_table(
        ["Dòng", "Tác dụng cụ thể"],
        [
            ["259", "`while (true)` – vòng lặp vô hạn, chỉ thoát bằng `break` bên trong. Vì điều kiện thoát nằm trong nhánh "
                    "`latest_slot_ < 0` nên cấu trúc được viết theo kiểu chờ đến khi thực sự cần dừng."],
            ["264–266", "`frame_ready_cv_.wait(lock, []{ return !running_ || latest_slot_ >= 0; })` – ngủ cho tới khi có khung "
                        "**hoặc** được yêu cầu dừng. Nhờ vậy `stop()` không cần cưỡng ép luồng."],
            ["268–271", "Nếu không có khung mà đã dừng thì `break` thoát vòng lặp; nếu không có khung mà chưa dừng thì `continue` "
                        "để ngủ lại – trường hợp thức dậy oan do thông báo hụt."],
            ["273–275", "**Nhận quyền sở hữu khung**: `slot_index = latest_slot_`, rồi lập tức `latest_slot_ = -1` (đã lấy, không "
                        "còn ai chờ) và `processing_slot_ = slot_index` (đang mượn, không ai chạm vào). Trạng thái luôn nhất quán."],
            ["278–280", "Lấy ảnh của ô đã mượn và đọc tốc độ hiện tại từ biến nguyên tử."],
            ["282–287", "Đo thời gian xử lý bằng `steady_clock` rồi ghi vào `processing_ms`. Chọn `steady_clock` thay vì "
                        "`system_clock` vì đơn vị đo không bị nhảy khi đổi múi giờ NTP."],
            ["283", "`detect_lanes(frame, current_speed, work_output_)` – trọng tâm. Chạy hoàn toàn ngoài khoá nên `get_latest()` "
                    "vẫn đọc được kết quả của khung trước trong lúc khung này đang được tính."],
            ["289–293", "Chép kết quả ra `latest_output_` dưới khoá `output_mtx_` rồi bật cờ `output_ready_`."],
            ["295–298", "Trả ô về cho bộ đệm: `processing_slot_ = -1`."],
            ["300", "Báo `frame_free_cv_` để luồng đọc biết đã có ô trống."],
        ],
        fracs=[0.08, 0.92],
    )
    note([
        "Câu hỏi tự nhiên: lệnh latest_output_ = work_output_ chỉ chép con trỏ cv::Mat, vậy khung sau ghi đè có làm hỏng ảnh mà node",
        "đang xem không?  Không. Vì mỗi lần xử lý, detect_lanes() gán lại out.raw và out.vis bằng kết quả của bgr.clone(), tức",
        "phân bổ vùng nhớ MỚI. work_output_ chỉ giảm bộ đếm tham chiếu tới vùng nhớ cũ, còn latest_output_ vẫn giữ tham chiếu",
        "của nó. Đây là hành vi đúng của cv::Mat và là lý do cách chia sẻ này an toàn.",
        "Ngược lại, nếu detect_lanes() dùng lại out.raw.copyTo() hoặc ghi đè cùng một vùng nhớ thì node đang hiển thị sẽ thấy ảnh",
        "nhảy loạn. Đây là điểm cần giữ khi chỉnh sửa module.",
    ], fill="E8F1DE")

    h2("4.8. Nhóm UTILITIES – toán học và lọc nhiễu")

    h3("clamp_float và clamp_int16 (dòng 308–316)")
    code(rng(CPP, 308, 316))
    p("Hai hàm chặn giá trị. `clamp_float` viết gọn thành `max(min_value, min(value, max_value))` – đúng với mọi giá trị. "
      "`clamp_int16` dùng hai lệnh if vì ép kiểu trực tiếp sang int16_t một giá trị vượt miền là **hành vi không xác định** trong C++, "
      "thường cho ra số âm do tràn.")

    h3("median_smooth – lọc trung vị 3 điểm (dòng 318–326)")
    code(rng(CPP, 318, 326))
    p("Đây là bộ lọc trung vị cửa sổ 3, viết tay để không cần thêm thư viện. Công thức `max(min(a, max(b, c)), min(max(a, b), c))` "
      "là cách tính trung vị của ba số chỉ bằng phép min và max – nhanh hơn sắp xếp.")
    bullets([
        "Dòng 319: nếu ít hơn 3 điểm thì bỏ qua – không đủ điểm để có nội điểm.",
        "Dòng 320: vòng lặp chạy từ `i = 1` đến `size - 2`, nên **điểm đầu và điểm cuối không bao giờ bị thay đổi**. Điều này có "
        "chủ ý: điểm cuối, tức điểm gần xe nhất, chính là điểm quyết định độ lệch; giữ nguyên giúp giá trị này ổn định.",
        "Vì chỉ thay toạ độ x của điểm giữa nên thứ tự các điểm không đổi – dữ liệu vẫn sắp xếp theo y tăng dần như đầu vào.",
    ])

    h3("median_tail_x – trung vị 5 điểm gần xe (dòng 328–339)")
    code(rng(CPP, 328, 339))
    bullets([
        "Các điểm được thu theo thứ tự y tăng dần, nên phần cuối của vector chính là phần gần xe nhất – vị trí quan trọng nhất.",
        "Dòng 330–332: `std::array<int, 8>` là bộ đệm **cố định trên ngăn xếp**, không cấp phát. Giới hạn cứng tám giá trị là "
        "điểm dễ gây hiểu nhầm nếu sau này ai đó tăng `MEDIAN_TAIL_POINTS`.",
        "Dòng 337–338: `sort` rồi lấy `values[n / 2]`. Với n = 5 thì lấy `values[2]` – đúng phần tử giữa sau khi sắp xếp.",
    ])

    h3("median_value – trung vị O(n) (dòng 341–346)")
    code(rng(CPP, 341, 346))
    p("Dùng `std::nth_element` thay vì `sort`: chỉ cần bảo đảm phần tử ở vị trí `mid` là phần tử thứ tự, không cần sắp xếp toàn bộ. "
      "Phức tạp trung bình O(n) so với O(n log n). Rất quan trọng: `nth_element` **sắp xếp lại dãy một phần**, và tham số ở đây là "
      "`vector<int>&` không const. Xem mục 11.2.")

    h3("calculate_slope – hồi quy tuyến tính theo phương y (dòng 348–364)")
    code(rng(CPP, 348, 364))
    p("Đây là hồi quy bình phương tối thiểu, nhưng **hồi quy x theo y** chứ không phải hồi quy y theo x. Công thức rút ra từ "
      "tối thiểu hoá tổng bình phương sai số:")
    code([
        "  Cho tập điểm (x_i, y_i),  tìm hệ số k  sao cho  x  ~=  k · y",
        "",
        "  f(k) = Σ (x_i − k·y_i)²      đạo phái bằng 0:",
        "  Σ −2·y_i·(x_i − k·y_i) = 0",
        "  k·Σ y_i² = Σ x_i·y_i",
        "",
        "  ⇒   k = ( n·Σ(x·y) − Σx·Σy )  /  ( n·Σ(y·y) − (Σy)² )",
        "            ↑ sum_xy                ↑ sum_yy        mẫu số",
        "",
        "  và code:   denominator = n·sum_yy − sum_y·sum_y",
        "             numerator   = n·sum_xy − sum_x·sum_y",
        "             slope       = numerator / denominator",
    ], caption="Suy ra công thức của calculate_slope")
    mk_table(
        ["Dòng", "Ý nghĩa"],
        [
            ["349", "**Cần ít nhất 10 điểm** mới trả kết quả, nếu không trả 0.0f. Lưu ý ngưỡng này nghiêm hơn `MIN_FINAL_POINTS = 5` "
                    "mà phần gọi kiểm tra: vạch có 5 đến 9 điểm vẫn qua được kiểm tra ở ngoài, nhưng slope của nó **luôn bằng 0**."],
            ["350–358", "Duyệt một lần, tích luỹ bốn tổng. Dùng kiểu `double` cho các tổng để tránh sai số khi tích luỹ hàng trăm "
                        "số nguyên."],
            ["360–361", "Mẫu số gần 0 nghĩa là các điểm có toạ độ y gần như bằng nhau – phép hồi quy không xác định. Trả 0 an toàn "
                        "thay vì chia cho 0 sinh ra inf hoặc NaN."],
            ["363", "Trả kết quả ép về kiểu float."],
        ],
        fracs=[0.07, 0.93],
    )

    h3("calculate_slope_offset – bù sớm đường cong (dòng 366–375)")
    code(rng(CPP, 366, 375))
    code([
        "  magnitude = |slope|",
        "",
        "  nếu magnitude ≤ 0,80            ->   0 px          (đường thẳng, không bù)",
        "  nếu magnitude ≥ 1,00            ->   120 px         (cua gắt, bù tối đa)",
        "  giữa 0,80 và 1,00               ->   50 … 120 px    (nội suy tuyến tính)",
        "",
        "  DẤU:   slope > 0  ->  trả về −offset     (bù sang trái  ->  lệnh LEFT)",
        "         slope < 0  ->  trả về +offset     (bù sang phải  ->  lệnh RIGHT)",
    ], caption="Bảng tra giá trị của calculate_slope_offset")
    p("Ý nghĩa vật lý: khi đường cong, **các điểm 5 cm trước xe trông thẳng** dù đường đang chạy về một bên. Nếu chỉ dùng giá trị "
      "lệch thô, xe sẽ chạy thẳng vào vỉa. Vì vậy code cộng thêm một lệch có dấu ngược với độ cong để **báo động trước**. "
      "Giải thích đầy đủ ở mục 9.4.")

    h3("calculate_target_speed – máy trạng thái (dòng 377–408)")
    code(rng(CPP, 377, 408))
    p("Hàm gồm hai `switch` liên tiếp – đây là cách viết máy trạng thái rõ ràng nhất:")
    bullets([
        "`switch` thứ nhất (dòng 385–400): **quyết định trạng thái mới** dựa trên trạng thái hiện tại và bốn điều kiện ngưỡng.",
        "`switch` thứ hai (dòng 402–407): **ánh xạ trạng thái mới sang giá trị tốc độ**.",
        "Tách thành hai bước giúp đọc dễ: nhìn `switch` thứ nhất thấy luồng điều kiện, nhìn `switch` thứ hai thấy bảng tra.",
    ])
    code([
        "  slope_mag = |dominant_slope|",
        "",
        "  sharp_enter =  slope_mag >= 0,85   HOẶC   abs_dev >= 80     (cua gặt: vào)",
        "  sharp_exit  =  slope_mag <  0,75   VÀ     abs_dev <  65     (cua gặt: thoát)",
        "  curve_enter =  slope_mag >= 0,65   HOẶC   abs_dev >= 50     (cua vừa: vào)",
        "  curve_exit  =  slope_mag <  0,55   VÀ     abs_dev <  40     (cua vừa: thoát)",
        "",
        "  Lưu ý: slope dùng >= để vào, < để thoát -> có vùng đệm chống chập chờn.",
        "          abs_dev dùng >= để vào, <  để thoát -> tương tự.",
    ], caption="Bốn điều kiện ngưỡng, rút gọn từ dòng 378–383")

    pagebreak()
