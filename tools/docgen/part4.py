# -*- coding: utf-8 -*-
from common import *


def render():
    h1("5. detect_lanes(): giải thích chi tiết từng dòng")
    p("Đây là hàm quan trọng nhất của module, dòng 414–672. Nó nhận ảnh BGR, tốc độ hiện tại, và một struct `LaneOutput` rỗng, "
      "rồi điền đầy đủ kết quả. Toàn bộ được chia thành mười hai khối dưới đây.")
    code(rng(CPP, 414, 418), caption="cpp:414–418 – chữ ký hàm và ba dòng mở đầu")

    h2("5.1. Đặt lại toàn bộ trạng thái đầu ra (dòng 415–427)")
    code(rng(CPP, 415, 427))
    p("Mười ba dòng gán giá trị mặc định cho mọi trường của `out`. Đây là nguyên tắc **fail-safe**: bất kể hàm thoát sớm ở vị trí nào "
      "từ dòng 429 trở đi, người gọi cũng nhận được một kết quả hợp lệ.")
    bullets([
        "Dòng 415–417: xoá ba vector đường vạch. Dùng `clear()` chứ không phải gán vector rỗng mới, để **giữ lại vùng nhớ đã cấp** "
        "từ `reserve(128)` ở constructor.",
        "Dòng 418: lệnh mặc định là STOP.",
        "Dòng 419–425: độ lệch bằng 0, tốc độ bằng 0, cờ `valid` là false, các chỉ số chẩn đoán về 0.",
        "Dòng 426–427: `release()` hai `cv::Mat`. Hàm này giải phóng tham chiếu tới vùng nhớ ảnh của khung trước. Đây chính là "
        "cơ chế giữ cho `latest_output_` an toàn khi chia sẻ con trỏ (xem mục 4.7).",
    ])

    h2("5.2. Chặn ảnh rỗng và ảnh quá nhỏ (dòng 429–447)")
    code(rng(CPP, 429, 447))
    mk_table(
        ["Dòng", "Ý nghĩa"],
        [
            ["429–433", "Ảnh rỗng thì đặt lại cờ khởi tạo tốc độ và trạng thái máy trạng thái về STRAIGHT rồi thoát. Cần đặt lại vì "
                        "camera bị rút khi ngắt sẽ không còn làn đáng tin, và bộ lọc tốc độ phải khởi động lại từ đầu."],
            ["435–436", "Lấy kích thước thật `W` và `H` từ chính ảnh, không dùng hằng số. Nhờ vậy toàn bộ phép tính sau đó tự động "
                        "đúng với mọi độ phân giải."],
            ["437–441", "Chặn ảnh quá nhỏ (dưới 100×100). Cùng lý do: trả về sớm và đặt lại trạng thái."],
            ["443", "`xmid_default = W / 2` – **trục giữa hình học của ảnh**. Đây là mốc để tính độ lệch. Biến cố định trong "
                        "cùng một khung hình, khác với `xmid_scan_` là trục quét thích nghi."],
            ["444", "Kẹp `xmid_scan_` vào miền hợp lệ `[0, W−1]`. Cần thiết vì biến này mang giá trị từ khung trước; nếu camera đổi "
                        "độ phân giải giữa chừng thì giá trị cũ có thể vượt biên."],
            ["446–447", "Chuẩn hoá lại tốc độ lần nữa ở phía luồng xử lý. Đây là lớp phòng thủ thứ hai bên cạnh lớp trong "
                        "`get_latest()` – vì `detect_lanes` là hàm private nhưng vẫn nên tự bảo vệ."],
        ],
        fracs=[0.09, 0.91],
    )

    h2("5.3. ROI động theo tốc độ (dòng 449–466)")
    code(rng(CPP, 449, 466))
    p("Đây là một trong những quyết định thiết kế đáng chú ý nhất của module. Vùng quan tâm **thu hẹp lại khi xe chạy chậm và mở rộng ra "
      "khi xe chạy nhanh**:")
    bullets([
        "Dòng 449: mặc định dùng `ROI_FACTOR_SLOW` = 0,70.",
        "Dòng 450–453: nếu tốc độ vượt `ROI_SPEED_START` = 4 km/h thì giảm dần hệ số theo tốc độ, với độ dốc `ROI_SPEED_GAIN` = 0,02, "
        "và kẹp trong khoảng `[ROI_FACTOR_FAST_MIN, ROI_FACTOR_SLOW]`.",
        "Dòng 455: `y0_dual` là **đường trên của ROI kép**, tính bằng cách nhân chiều cao ảnh với hệ số rồi làm tròn.",
        "Dòng 456: `y0_single` là đường trên của ROI khi chỉ thấy một vạch, dùng hệ số 0,85.",
        "Dòng 457: `y1 = H − 1` – luôn quét tới đáy ảnh, vì vùng sát xe là quan trọng nhất.",
        "Dòng 459–463: chặn trường hợp ROI không còn chiều cao, ví dụ ảnh quá thấp mà hệ số lại lớn.",
        "Dòng 465–466: tạo ROI và lấy **lát cắt tham chiếu** của ảnh BGR. Không copy pixel – đây là một `cv::Mat` nhỏ chỉ giữ "
        "con trỏ và kích thước.",
    ])
    note([
        "Một điểm rất dễ hiểu sai: vòng quét ở mục 5.5 LUÔN bắt đầu từ y0_dual, kể cả khi sau đó kết luận chỉ thấy một vạch.",
        "Hệ số 0,85 chỉ được dùng ở bước cắt lọc (dòng 558 và 561) để loại bỏ các điểm nằm trên cao, giữ lại phần gần xe.",
        "Lý do làm vậy là tiết kiệm công: quét một lần duy nhất phục vụ cả hai kịch bản, thay vì quét hai lần.",
    ])

    h2("5.4. Chuyển ảnh xám, làm mờ, Canny, phép đóng (dòng 468–479)")
    code(rng(CPP, 468, 479))
    mk_table(
        ["Dòng", "Ý nghĩa cụ thể"],
        [
            ["468", "Nếu ảnh đã là một kênh thì gán thẳng, không cần chuyển đổi. Nhánh này phục vụ việc tái dùng module với ảnh xám sẵn có."],
            ["469", "Ba kênh BGR thì chuyển sang xám bằng công thức phân bổ 0,114 B + 0,587 G + 0,299 R. Vì vạch làn màu đen trên nền "
                    "sáng nên phép trung bình này cho tương phản rất cao giữa vạch và nền."],
            ["470", "Bốn kênh BGRA thì loại bỏ kênh alpha rồi mới quy ra xám."],
            ["471–475", "Số kênh lạ (ví dụ 2 kênh) thì không xử lý được, đặt lại trạng thái và thoát."],
            ["477", "`GaussianBlur` với kernel 5×5 và sigma bằng 0. Tác dụng: làm mờ nhiễu cảm biến và nhiễu nhiễu hạt, đồng thời làm "
                    "cho biên Canny đỡ gắt nét."],
            ["478", "`Canny(blur_, edges_, 30, 120)` – phát hiện biên. Ngưỡng thấp 30 vì đối tượng là vạch đen trên nền sáng, tương "
                    "phản biên rất lớn. Kết quả `edges_` là ảnh nhị phân 0 hoặc 255, cùng kích thước với ROI."],
            ["479", "`morphologyEx` với phép **CLOSE** (phép đóng) và kernel chữ nhật 3×3. CLOSE = EROSION rồi DILATION, tức **bịt lỗ "
                    "hổng trong vạch và nối các đoạn đứt**. Đây là bước bắt buộc: vạch làn trên ảnh thực thường bị gãy vì mờ, bóng, "
                    "phản xạ. Ghi kết quả đè lên chính `edges_` để không tốn bộ nhớ."],
        ],
        fracs=[0.07, 0.93],
    )

    h2("5.5. Vòng quét dòng – trái tim của module (dòng 481–516)")
    code(rng(CPP, 481, 516))
    mk_table(
        ["Dòng", "Ý nghĩa cụ thể"],
        [
            ["481–483", "Xoá ba bộ điểm và danh sách trung tâm của khung trước, dùng `clear()` để giữ vùng nhớ đã cấp."],
            ["485–487", "Tính `width_scale = W / 640`, rồi quy đổi hai ngưỡng bề rộng làn về đơn vị pixel thật của ảnh. Nhờ vậy "
                        "thuật toán giữ nguyên hành vi khi camera chạy ở độ phân giải khác."],
            ["489", "`xmid_candidate` khởi tạo bằng trục quét đã nhớ từ khung trước. Đây là cơ chế **bám làn theo thời gian**."],
            ["490–491", "Bộ tích luỹ để tính bề rộng làn trung bình và đếm số dòng hợp lệ."],
            ["493", "Vòng lặp ngoài: `y` từ `y0_dual` đến `y1`, **bước nhảy `SCAN_STEP` = 4**. Với ROI kép ở tốc độ thấp (y từ 280 "
                    "đến 399) cho 30 dòng quét; ở tốc độ cao nhất (y từ 220) cho 45 dòng."],
            ["494", "Chuyển toạ độ ảnh sang toạ độ **trong ROI** để tra cứu `edges_`, vì `edges_` chỉ chứa phần ROI."],
            ["495", "`edges_.ptr<uchar>(local_y)` – lấy con trỏ tới đầu hàng. `ptr` trả về con trỏ hằng vì biến là `const uchar*`. "
                    "Đây là thao tác nhanh nhất có thể: chỉ một phép cộng địa chỉ cho mỗi hàng, không sao chép hàng."],
            ["496", "Khởi tạo hai vị trí tìm thấy bằng −1, nghĩa là chưa thấy vạch nào ở hàng này."],
            ["498–500", "Quét **từ `xmid_candidate` đi về trái**, lấy pixel đầu tiên khác 0. Đây chính là ** mép trong của vạch trái** – "
                        "phía đối diện giữa làn. Việc lấy mép trong là lý do phải có ngưỡng bề rộng tối thiểu 200 px."],
            ["501–503", "Quét **từ `xmid_candidate` đi về phải**, lấy pixel đầu tiên khác 0 – mép trong của vạch phải."],
            ["505–506", "Nếu tìm thấy thì thêm điểm `(x, y)` vào vector tương ứng. Những hàng không thấy vạch này đơn giản là bị bỏ qua, "
                        "không cần xử lý riêng."],
            ["508–515", "Chỉ khi **cả hai vạch cùng xuất hiện** mới đo bề rộng `right − left`. Nếu bề rộng nằm trong "
                        "`[min_width, max_width]` thì cộng vào tổng, tăng bộ đếm, và lưu trung tâm `(left + right) / 2` vào "
                        "`prelim_centers_`."],
        ],
        fracs=[0.08, 0.92],
    )
    note([
        "Điểm thiết kế cốt lõi: hàm chỉ quét **hai phía của trục giữa**, không quét toàn bộ chiều rộng ảnh.",
        "Nhờ vậy chi phí mỗi dòng chỉ bằng khoảng (bề rộng làn) phép so sánh, tức khoảng 230 pixel thay vì 640.",
        "Trái đổi là trục quét phải bám theo làn – và đó chính là biến xmid_scan_ được thiết kế ở đây.",
        "Nếu xe bị đẩy sang hẳn một bên làm vạch trái nằm cùng phía với trục quét, vạch đó sẽ không được tìm thấy.",
        "Đây là lý do nhánh một vạch tồn tại: khi mất một vạch, module suy luận trung tâm từ vạch còn lại thay vì báo lỗi.",
    ], fill="E8F1DE")

    h2("5.6. Quyết định làn kép hay không (dòng 518–532)")
    code(rng(CPP, 518, 532))
    p("Đây là quyết định quan trọng nhất về mặt ý nghĩa, vì nó chọn giữa hai nhánh thuật toán hoàn toàn khác nhau.")
    mk_table(
        ["Dòng", "Điều kiện hoặc hành động", "Nghĩa"],
        [
            ["518–519", "`leftPts_prelim_.size() >= 15` và `rightPts_prelim_.size() >= 15`",
             "Mỗi vạch phải xuất hiện ở ít nhất 15 dòng quét (đúng một nửa trong số 30 dòng của ROI kép)."],
            ["523", "Đồng thời có `overlap_count >= 10`",
             "Ít nhất 10 dòng phải có **cả hai vạch** cùng hợp lệ về bề rộng. Đây là điều kiện mạnh nhất và là chốt chặn cuối."],
            ["524", "`avg_lane_width = width_sum / overlap_count`",
             "Bề rộng làn trung bình tính trên các dòng hợp lệ, tránh thiên lệch do một vài dòng nhiễu."],
            ["525–527", "Kiểm tra bề rộng trung bình còn nằm trong khoảng hợp lệ thì `is_dual_lane = true`",
             "Ngay cả khi đạt hai ngưỡng điểm, bề rộng trung bình vẫn phải hợp lệ mới công nhận là làn kép."],
            ["531", "`lane_width_est_px_ = 0,2 × đo được + 0,8 × giá trị cũ`",
             "Cập nhật ước lượng bề rộng làn bằng bộ lọc EMA. Hệ số 0,2 nhỏ vì bề rộng làn thay đổi chậm; giá trị này được "
             "dùng lại ở nhánh một vạch để suy luận trung tâm."],
        ],
        fracs=[0.07, 0.30, 0.63],
    )

    h2("5.7. Cập nhật bề rộng làn và chuẩn hoá trục quét (dòng 534–551)")
    code(rng(CPP, 534, 551))
    p("Khối này điều chỉnh trục quét theo kết quả vừa đo, nhưng **giới hạn tốc độ dịch chuyển** để trục không nhảy theo nhiễu.")
    mk_table(
        ["Dòng", "Nhánh", "Giải thích"],
        [
            ["534–541", "**Làn kép**",
             "`measured_mid` là trung vị của các trung tâm đã đo. Nếu lệch quá 20 px thì **chỉ dịch tối đa 20 px mỗi khung** "
             "(dòng 538), giữ trục quét ở gần vị trí cũ; nếu lệch ít hơn 20 px thì chuyển thẳng tới vị trí đo (dòng 540). "
             "Kết quả: trục quét bám theo làn nhưng chậm, không rung."],
            ["542–549", "**Một vạch**",
             "Mục tiêu là trả trục quét về **giữa hình học ảnh** (`xmid_default`). Giới hạn dịch chuyển ở đây chỉ là 5 px mỗi khung – "
             "nhỏ hơn nhiều – vì ở chế độ một vạch, điều chỉnh nhanh theo vạch sẽ khiến toàn bộ tín hiệu sai lệch."],
            ["551", "Kẹp kết quả vào `[0, W−1]` và ghi lại vào `xmid_scan_` để dùng cho khung sau.",
             "Biến thành viên, nên trục quét có **trí nhớ qua các khung hình** – yếu tố quyết định chất lượng bám làn."],
        ],
        fracs=[0.09, 0.16, 0.75],
    )

    h2("5.8. Cắt lọc điểm, làm trơn trung vị, tính slope (dòng 553–579)")
    code(rng(CPP, 553, 579))
    bullets([
        "Dòng 553: `y0_used` chọn ROI kép (0,70) nếu là làn kép, ngược lại dùng ROI một vạch (0,85). Đây là điểm hệ số 0,85 "
        "thực sự được dùng.",
        "Dòng 554–555: xoá hai vector đích, giữ nguyên bản sơ bộ để có thể so sánh nếu cần.",
        "Dòng 557–562: chép các điểm có `y >= y0_used`. Với ROI một vạch ở khung 400 px (y từ 340), chỉ giữ lại 15 dòng gần xe nhất.",
        "Dòng 564–565: chạy bộ lọc trung vị 3 điểm trên cả hai đường. Đây là bước khử nhiễu quan trọng nhất trên phần đường vạch.",
        "Dòng 567–569: đưa kết quả ra `out.left` và `out.right`, đồng thời tính `pixels_used` làm chỉ số chẩn đoán. Ở đây có **hai "
        "phép chép vector** – một lần vào out, một lần nữa khi chép `work_output_` sang `latest_output_`.",
        "Dòng 571–572: `has_left` và `has_right` kiểm tra ngưỡng 5 điểm sau cắt.",
        "Dòng 574–575: tính slope từng vạch, đặt bằng 0 nếu vạch không đủ điểm.",
        "Dòng 576: `dominant_slope` lấy theo vạch có **giá trị tuyệt đối lớn hơn**. Khi đường cong, cả hai vạch cùng dấu và có độ cong "
        "gần bằng nhau nên lấy của vạch nào cũng cho kết quả gần đúng.",
        "Dòng 578: `curve_angle_deg = atan(slope) · 180/π`. Vì slope là dx/dy nên góc này đo **độ nghiêng so với phương thẳng đứng**, "
        "không phải so với phương ngang. Đây chính là lý do `fusion_viz_node` chia ngược lại bằng 57,3 để khôi phục slope.",
        "Dòng 579: `curvature = |slope|` – đại lượng không dấu, thuận tiện để so sánh với ngưỡng.",
    ])

    h2("5.9. Nhánh làn kép: dựng centerline và tính độ lệch (dòng 581–608)")
    code(rng(CPP, 581, 608))
    mk_table(
        ["Dòng", "Ý nghĩa"],
        [
            ["581", "Điều kiện vào nhánh: là làn kép **và** cả hai vạch đều đủ 5 điểm sau cắt."],
            ["583–594", "Hai con trỏ `i` và `j` duyệt đồng thời hai danh sách vạch, **chỉ ghép khi các toạ độ y bằng nhau**. "
                        "Vì cả hai vạch đều được thu từ cùng một vòng quét với bước nhảy 4 nên các giá trị y luôn trùng nhau, và vòng lặp "
                        "thực tế sẽ ghép được mọi cặp. Có điều kiện so sánh y chỉ để **an toàn nếu sau này thay đổi cách lấy điểm**."],
            ["586–587", "`center_x = (x_trái + x_phải) / 2` – trung tâm làn tại từng dòng. Dùng phép chia nguyên, chấp nhận sai số dưới 1 px."],
            ["596–597", "**Điểm quyết định**: độ lệch không lấy từ trung bình mọi điểm, mà lấy **trung vị của 5 điểm cuối** – tức 5 điểm "
                        "gần xe nhất. Vị trí gần xe phản ánh trạng thái thực tế của xe, và trung vị chịu được nhiễu tốt hơn trung bình."],
            ["598", "`raw_dev = center_x − xmid_default`. **Dương khi trung tâm làn nằm bên phải trục giữa ảnh**, tức xe đang lệch sang "
                    "trái so với làn."],
            ["599", "Tính lệch bù đường cong (xem mục 9.4)."],
            ["601–604", "**Ngưỡng hồi phục**: nếu độ lệch thô vượt 95 px quy đổi thì **xoá bù**, vì lúc đó xe đã lệch xa và cần quay về "
                        "làn bằng phản hồi thực, không phải bằng bù đường cong."],
            ["606", "`dev_final_px = clamp_int16(raw_dev + slope_offset)` – cộng bù rồi chặn về miền int16_t."],
            ["607", "Đặt `valid = true`. Đến đây node mới tin là có kết quả."],
        ],
        fracs=[0.08, 0.92],
    )

    h2("5.10. Nhánh một vạch: suy luận trung tâm từ bề rộng làn (dòng 609–626)")
    code(rng(CPP, 609, 626))
    p("Khi chỉ thấy một vạch, trung tâm làn **không quan sát được** mà phải **suy luận**: nếu là vạch trái thì trung tâm nằm sang "
      "phải một nửa bề rộng làn, và ngược lại.")
    mk_table(
        ["Dòng", "Ý nghĩa"],
        [
            ["610", "Chọn vạch để tin: nếu chỉ có một vạch thì dùng vạch đó; nếu có cả hai nhưng `is_dual_lane` lại false (ví dụ vì "
                    "bề rộng không hợp lệ) thì dùng vạch có **nhiều điểm hơn**."],
            ["611", "Tham chiếu tới vector vạch đã chọn – tham chiếu, không copy."],
            ["614", "Lấy `lane_x` bằng trung vị 5 điểm gần xe, giống hệt nhánh hai vạch để hai nhánh cho kết quả liên tục khi chuyển "
                    "chế độ."],
            ["615", "Bề rộng làn lấy từ `lane_width_est_px_` đã lọc EMA ở dòng 531, rồi kẹp trong khoảng an toàn 100 đến 500 px "
                    "(đã nhân `width_scale`)."],
            ["618", "Nếu dùng vạch trái thì **cộng** nửa bề rộng vào toạ độ vạch để được trung tâm."],
            ["619", "Nếu dùng vạch phải thì **trừ** nửa bề rộng."],
            ["621", "Kẹp toạ độ trung tâm ước lượng vào miền ảnh."],
            ["622–624", "Tính độ lệch so với giữa ảnh rồi đặt `valid = true`. **Không cộng `slope_offset`** ở nhánh này – vì khi chỉ thấy "
                        "một vạch thì ước lượng độ cong kém tin cậy hơn hẳn, cộng bù vào lúc này có nguy cơ tạo ra lệnh lái sai."],
        ],
        fracs=[0.08, 0.92],
    )

    h2("5.11. Ra lệnh và bộ lập tốc độ (dòng 628–657)")
    code(rng(CPP, 628, 657))
    mk_table(
        ["Dòng", "Ý nghĩa"],
        [
            ["628–629", "Chỉ khi `valid` mới làm tiếp. Tính `abs_dev` là trị tuyệt đối của độ lệch đã qua bù."],
            ["631–633", "**Quy tắc phát lệnh**: lệch tuyệt đối dưới 15 px thì FWD; lệch âm thì LEFT; còn lại thì RIGHT. Đây là "
                        "ba lệnh rời rạc để telemetry hiển thị, **không phải** đầu vào điều khiển – giá trị điều khiển thật là "
                        "`dev_final_px` với dấu và độ lớn liên tục."],
            ["635", "Gọi máy trạng thái để xác định tốc độ mục tiêu, đồng thời truyền cả độ cong lẫn độ lệch – vì hai nguyên nhân "
                    "gây chậm là hai nguyên nhân khác nhau."],
            ["637–639", "Nếu chưa có mẫu tốc độ nào thì lấy nguyên giá trị yêu cầu, đặt cờ đã khởi tạo. Việc này tránh việc khung "
                    "đầu tiên bị lọc từ giá trị 85 xuống gần 26, gây giảm tốc oan."],
            ["641", "Bộ lọc mềm: mới = 0,7 × yêu cầu + 0,3 × giá trị đang có. Hệ số lớn vì tốc độ cần phản ứng nhanh."],
            ["644–645", "Kẹp vào 0–255 rồi làm tròn và ép sang uint8_t."],
            ["647", "`speed_factor = 100 × target / 85`. Giá trị 100 nghĩa là đang ở tốc độ tối đa. Với ba mức: 85 → 100, "
                    "60 → 71, 45 → 53."],
            ["648", "Kẹp `speed_factor` không vượt quá 100, phòng trường hợp tốc độ lọc vượt quá mức thẳng do sai số phép cộng."],
            ["649–657", "**Nhánh không hợp lệ**: đặt lệnh STOP, xoá toàn bộ số đo, đặt lại bộ lọc tốc độ về mức thẳng, xoá cờ khởi tạo "
                        "và đưa máy trạng thái về STRAIGHT. **Đặc biệt quan trọng**: xoá cờ khởi tạo để khi làn xuất hiện trở lại, bộ "
                        "lọc khởi động từ giá trị yêu cầu chứ không phải từ giá trị cũ đã bị kẹp về 0."],
        ],
        fracs=[0.09, 0.91],
    )

    h2("5.12. Xuất ảnh gốc và ảnh visualize (dòng 659–671)")
    code(rng(CPP, 659, 671))
    mk_table(
        ["Dòng", "Ý nghĩa"],
        [
            ["659", "`out.raw = bgr.clone()` – **luôn thực hiện**, không phụ thuộc cờ visualize. Đây là bản ảnh gốc publish cho "
                    "topic riêng. Lưu ý về chi phí: mỗi khung hình clone toàn bộ ảnh, tức thêm một lần cấp phát và sao chép."],
            ["661", "`if constexpr (ENABLE_VISUALIZATION)` – điều kiện **lúc biên dịch**, không phải lúc chạy. Khi cờ là false, "
                    "trình biên dịch loại hẳn khối code này khỏi mã máy."],
            ["662", "`out.vis = bgr.clone()` – lần clone thứ hai cho ảnh vẽ đè."],
            ["663", "Vẽ đường thẳng **trắng** tại `xmid_default` – trục giữa hình học, tức đường lý tưởng xe nên bám."],
            ["664", "Vẽ đường thẳng **vàng** tại `xmid_scan_` – trục quét thích nghi đang bám làn. Sự khác biệt giữa hai đường này "
                    "là cách nhanh nhất để nhìn thấy thuật toán đang làm việc."],
            ["666–667", "Vẽ đường vạch trái và phải bằng **đỏ**, độ dày 2, khử răng cưa. Tham số `false` nghĩa là **không** khép kín "
                    "đường vẽ – vì đây là hai đường mở, đóng kín sẽ tạo ra một đoạn nối bật kỳ ở cuối."],
            ["668", "Vẽ trung tâm làn bằng **xanh lá**. Chỉ có trong nhánh hai vạch."],
            ["670", "Nhánh else: giải phóng `out.vis` để không giữ tham chiếu ảnh cũ."],
        ],
        fracs=[0.08, 0.92],
    )
    pagebreak()
