#include "lane_mask.hpp"

#include <algorithm>
#include <cmath>

namespace lane_mask {

// ============================================================================
// PHÂN VỊ QUA HISTOGRAM
// O(256) thay vì sort toàn bộ ảnh. Dùng cv::histogram trên ảnh đã resize
// nhỏ để chi phí không đáng kể.
// ============================================================================

void percentiles_from_histogram(
    const cv::Mat& gray_u8,
    double p_low,
    double p_high,
    int& out_low,
    int& out_high
) {
    out_low = 0;
    out_high = 255;

    if (gray_u8.empty() || gray_u8.type() != CV_8UC1) return;

    int hist[256] = {0};

    // reduce trước để giảm số pixel phải duyệt
    cv::Mat small;
    const int max_side = 160;
    const int max_dim = std::max(gray_u8.cols, gray_u8.rows);
    if (max_dim > max_side) {
        const double s = static_cast<double>(max_side) / static_cast<double>(max_dim);
        cv::resize(gray_u8, small, cv::Size(), s, s, cv::INTER_AREA);
    } else {
        small = gray_u8;
    }

    const int rows = small.rows;
    const int cols = small.cols;
    for (int y = 0; y < rows; ++y) {
        const unsigned char* row = small.ptr<unsigned char>(y);
        for (int x = 0; x < cols; ++x) {
            ++hist[row[x]];
        }
    }

    const long long total = static_cast<long long>(rows) * static_cast<long long>(cols);
    if (total <= 0) return;

    p_low = std::clamp(p_low, 0.0, 0.999);
    p_high = std::clamp(p_high, p_low + 0.001, 1.0);

    const long long target_low = std::max<long long>(
        1, static_cast<long long>(p_low * static_cast<double>(total)));
    const long long target_high = std::min<long long>(
        total, static_cast<long long>(p_high * static_cast<double>(total)));

    long long acc = 0;
    int lo = 0;
    int hi = 255;
    bool got_lo = false;

    for (int v = 0; v < 256; ++v) {
        acc += hist[v];
        if (!got_lo && acc >= target_low) {
            lo = v;
            got_lo = true;
        }
        if (acc >= target_high) {
            hi = v;
            break;
        }
    }

    if (hi <= lo) {
        // Ảnh gần như đồng nghĽa (phòng tối hoàn toàn). Trả dải hẹp quanh
        // giá trị đó để ngưỡng không bị chia 0.
        lo = std::max(0, lo - 4);
        hi = std::min(255, lo + 8);
    }

    out_low = lo;
    out_high = hi;
}

// ============================================================================
// CHE NẮP XE
// ============================================================================

void mask_bonnet(cv::Mat& mask, double x0, double x1, double y0) {
    if (mask.empty()) return;

    const int w = mask.cols;
    const int h = mask.rows;

    const int x_start = std::max(0, static_cast<int>(x0 * static_cast<double>(w)));
    const int x_end = std::min(w, static_cast<int>(x1 * static_cast<double>(w)));
    const int y_start = std::max(0, static_cast<int>(y0 * static_cast<double>(h)));

    if (x_end <= x_start || y_start >= h) return;

    cv::rectangle(
        mask,
        cv::Rect(x_start, y_start, x_end - x_start, h - y_start),
        cv::Scalar(0),
        cv::FILLED
    );
}

// ============================================================================
// HÀM CHÍNH
// ============================================================================

bool build_lane_mask(
    const cv::Mat& bgr_full,
    const Options& opts,
    cv::Mat& mask_out,
    Result* debug
) {
    if (bgr_full.empty() || bgr_full.cols < 32 || bgr_full.rows < 32) return false;

    const int work_w = std::max(64, opts.work_w);
    const int work_h = std::max(48, opts.work_h);

    // -----------------------------------------------------------------------
    // 0. Chuẩn hoá về BGR 3 kênh
    // -----------------------------------------------------------------------
    cv::Mat bgr;
    if (bgr_full.channels() == 3) {
        bgr = bgr_full;
    } else if (bgr_full.channels() == 4) {
        cv::cvtColor(bgr_full, bgr, cv::COLOR_BGRA2BGR);
    } else if (bgr_full.channels() == 1) {
        cv::cvtColor(bgr_full, bgr, cv::COLOR_GRAY2BGR);
    } else {
        return false;
    }

    // -----------------------------------------------------------------------
    // 1. Resize về kích thước làm việc
    // -----------------------------------------------------------------------
    cv::Mat work;
    if (bgr.cols != work_w || bgr.rows != work_h) {
        cv::resize(bgr, work, cv::Size(work_w, work_h), 0, 0, cv::INTER_AREA);
    } else {
        work = bgr;
    }

    // Che nắp xe: KHÔNG vẽ trên ảnh BGR. Nếu xoá pixel ở đây thì vùng nắp xe
    // trở thành vùng đen tuyệt đối, sau đó lọt vào nhánh "vạch tối" và tạo
    // thành một blob lớn nằm đúng giữa bird view - đúng chỗ hai làn hội tụ.
    // Đã chuyển sang xoá trên mask nhị phân, xem bước 4b.

    // -----------------------------------------------------------------------
    // 2. Chuyển Lab -> CLAHE trên kênh L -> trở lại BGR
    // CLAHE trong OpenCV chỉ hỗ trợ ảnh 1 kênh. Chuyển Lab, apply L, convert back.
    // -----------------------------------------------------------------------
    static cv::Ptr<cv::CLAHE> cached_clahe;
    static int cached_clip = -1;
    static int cached_gx = -1;
    static int cached_gy = -1;

    if (!cached_clahe ||
        cached_clip != static_cast<int>(std::lround(opts.clahe_clip)) ||
        cached_gx != opts.clahe_grid_x ||
        cached_gy != opts.clahe_grid_y) {
        cached_clahe = cv::createCLAHE(
            opts.clahe_clip,
            cv::Size(opts.clahe_grid_x, opts.clahe_grid_y)
        );
        cached_clip = static_cast<int>(std::lround(opts.clahe_clip));
        cached_gx = opts.clahe_grid_x;
        cached_gy = opts.clahe_grid_y;
    }

    cv::Mat lab;
    cv::cvtColor(work, lab, cv::COLOR_BGR2Lab);

    std::vector<cv::Mat> lab_channels;
    cv::split(lab, lab_channels);
    cv::Mat l_channel = lab_channels[0];

    cv::Mat l_equalized;
    cached_clahe->apply(l_channel, l_equalized);
    lab_channels[0] = l_equalized;

    cv::Mat equalized_lab;
    cv::merge(lab_channels, equalized_lab);

    cv::Mat equalized;
    cv::cvtColor(equalized_lab, equalized, cv::COLOR_Lab2BGR);

    // L trong OpenCV Lab nằm ở byte 0 (0..255 đã scale sẵn).
    // Đã có l_channel từ trên

    // -----------------------------------------------------------------------
    // 3. Ngưỡng thích nghi theo phân vị
    // -----------------------------------------------------------------------
    int p_lo = 0;
    int p_hi = 255;
    percentiles_from_histogram(l_channel, opts.p_low, opts.p_high, p_lo, p_hi);

    const int span = std::max(1, p_hi - p_lo);
    const int dark_thr = std::clamp(
        p_lo + static_cast<int>(std::lround(opts.dark_band * static_cast<double>(span))),
        1, 254);
    const int light_thr = std::clamp(
        p_hi - static_cast<int>(std::lround(opts.light_band * static_cast<double>(span))),
        1, 254);

    if (debug) {
        cv::Scalar luma_mean;
        cv::Scalar luma_stddev;
        cv::meanStdDev(l_channel, luma_mean, luma_stddev);
        debug->luma_mean = luma_mean[0];
        debug->luma_stddev = luma_stddev[0];
        debug->dark_threshold = dark_thr;
        debug->light_threshold = light_thr;
        debug->l_channel = l_channel;
    }

    // -----------------------------------------------------------------------
    // 4. Nhị phân hoá: vạch tối (đường sáng) + tùy chọn vạch sáng (đường tối)
    // -----------------------------------------------------------------------
    cv::Mat mask;
    cv::threshold(l_channel, mask, dark_thr, 255, cv::THRESH_BINARY_INV);

    if (opts.enable_light_lanes) {
        cv::Mat mask_light;
        cv::threshold(l_channel, mask_light, light_thr, 255, cv::THRESH_BINARY);
        cv::bitwise_or(mask, mask_light, mask);
    }

    // -----------------------------------------------------------------------
    // 4b. Che nắp xe trên MASK NHỊ PHÂN
    // Xoá ở bước này (sau ngưỡng, trước morphology) thì vùng nắp xe biến mất
    // khỏi mask mà không tạo thêm ứng viên nào, đồng thời không làm méo phân
    // vị của kênh L vốn quyết định ngưỡng.
    // -----------------------------------------------------------------------
    if (opts.bonnet_enable) {
        mask_bonnet(mask, opts.bonnet_x0, opts.bonnet_x1, opts.bonnet_y0);
    }

    // -----------------------------------------------------------------------
    // 5. SobelX chuẩn hoá (chỉ để chấm điểm blob, KHÔNG gate mask)
    // Bản cũ AND mask với inRange(SobelX, 30, 255) -> chỉ giữ hai mép vạch
    // chứ không giữ thân vạch, rồi lại đòi peak >= 60 (tuyệt đối). Ở phòng
    // tối cả hai điều kiện đều hỏng.
    // -----------------------------------------------------------------------
    cv::Mat sobel;
    cv::Sobel(l_channel, sobel, CV_16S, 1, 0, 3, 1.0, 0.0, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(sobel, sobel);

    double sobel_p99 = 0.0;
    {
        // p99 SobelX: dùng histogram 256 bin.
        int hist[256] = {0};
        const int rows = sobel.rows;
        const int cols = sobel.cols;
        for (int y = 0; y < rows; ++y) {
            const unsigned char* row = sobel.ptr<unsigned char>(y);
            for (int x = 0; x < cols; ++x) ++hist[row[x]];
        }
        const long long total = static_cast<long long>(rows) * static_cast<long long>(cols);
        long long acc = 0;
        const long long target = static_cast<long long>(0.99 * static_cast<double>(total));
        for (int v = 0; v < 256; ++v) {
            acc += hist[v];
            if (acc >= target) {
                sobel_p99 = static_cast<double>(v);
                break;
            }
        }
    }

    if (debug) {
        debug->sobel_x = sobel;
        debug->sobel_p99 = sobel_p99;
        debug->edge_quality = sobel_p99 / 255.0;
    }

    // -----------------------------------------------------------------------
    // 6. Morphology
    // Đóng theo phương Y (15) để nối vạch đứt do nhiễu, không dùng kernel vuông
    // vì sẽ xoá các đoạn vạch mỏng ở xa.
    // -----------------------------------------------------------------------
    {
        static cv::Mat close_kernel;
        static cv::Mat open_kernel;
        static int cached_cw = -1, cached_ch = -1, cached_ok = -1;
        if (cached_cw != opts.close_w || cached_ch != opts.close_h || cached_ok != opts.open_k) {
            close_kernel = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(opts.close_w, opts.close_h));
            open_kernel = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(opts.open_k, opts.open_k));
            cached_cw = opts.close_w;
            cached_ch = opts.close_h;
            cached_ok = opts.open_k;
        }
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);
        if (opts.open_k > 0) {
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel);
        }
    }

    // -----------------------------------------------------------------------
    // 7. Lọc blob theo ROI
    // Bản cũ chạy compare + bitwise_and + minMaxLoc + bitwise_or trên TOÀN ẢNH
    // cho MỖI component: O(N·W·H). Với N=20 và 640x400 là ~29 ms/frame.
    // Ở đây giới hạn thao tác trong bounding box lấy từ stats, O(Σ bbox).
    // -----------------------------------------------------------------------
    const float lane_width_work = static_cast<float>(work_w) *
        (230.0f / 640.0f);  // giữ đúng tỉ lệ DEFAULT_LANE_WIDTH_PX cũ
    const int max_blob_width = std::clamp(
        static_cast<int>(std::lround(lane_width_work * opts.max_blob_width_ratio)),
        opts.max_blob_width_min, opts.max_blob_width_max);

    const double edge_floor = opts.require_edge
        ? opts.edge_peak_ratio * sobel_p99
        : 0.0;

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int components = cv::connectedComponentsWithStats(
        mask, labels, stats, centroids, 8, CV_32S);

    mask_out = cv::Mat::zeros(mask.size(), CV_8UC1);

    int kept = 0;
    for (int i = 1; i < components; ++i) {
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        const int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        const int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        const int bx = stats.at<int>(i, cv::CC_STAT_LEFT);
        const int by = stats.at<int>(i, cv::CC_STAT_TOP);

        if (area < opts.min_area) continue;
        if (height < opts.min_height) continue;
        if (width > max_blob_width) continue;

        const int x_end = std::min(mask.cols, bx + width);
        const int y_end = std::min(mask.rows, by + height);
        const cv::Rect roi(bx, by, x_end - bx, y_end - by);
        if (roi.width <= 0 || roi.height <= 0) continue;

        if (edge_floor > 0.0) {
            cv::Mat labels_roi = labels(roi);
            cv::Mat blob_roi;
            cv::compare(labels_roi, i, blob_roi, cv::CMP_EQ);

            cv::Mat sobel_roi = sobel(roi);
            cv::Mat sobel_masked;
            cv::bitwise_and(sobel_roi, blob_roi, sobel_masked);

            double peak = 0.0;
            cv::minMaxLoc(sobel_masked, nullptr, &peak);
            if (peak < edge_floor) continue;

            cv::bitwise_or(mask_out(roi), blob_roi, mask_out(roi));
        } else {
            cv::Mat labels_roi = labels(roi);
            cv::Mat blob_roi;
            cv::compare(labels_roi, i, blob_roi, cv::CMP_EQ);
            cv::bitwise_or(mask_out(roi), blob_roi, mask_out(roi));
        }
        ++kept;
    }

    if (debug) debug->kept_blobs = kept;

    return kept > 0;
}

}  // namespace lane_mask
