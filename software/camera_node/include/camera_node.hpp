#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// CAMERA LANE - nhanh 2 lan bang cua so truot
// ----------------------------------------------------------------------------
// Anh goc 640x480, xu ly o 320x240 (dung mot nua, giu nguyen ti le 4:3).
//
// Quy trinh xu ly anh:
//   1 resize          6 Otsu nguong + morphology
//   2 ROI hinh thang  7 cua so truot: chiieu cot -> seed -> dinh
//   3 xam             8 kiem tra 2 lan o du N cua so
//   4 GaussianBlur    9 fit duong bac 2 + do lech tam theo met
//   5 CLAHE
//
// Khong dung IPM: do bề rộng làn ra mét bằng công thức pinhole
// W = w_px * CAMERA_HEIGHT_M / (y - HORIZON_Y), chi cần 2 so.
// Nhờ vậy dev_px vẫn là pixel ảnh gốc, đúng quy ước firmware đang dùng.
// ============================================================================

class CameraLane {

public:

    // ------------------------------------------------------------------------
    // Kich thuoc xu ly
    // ------------------------------------------------------------------------

    static constexpr int WORK_W = 320;
    static constexpr int WORK_H = 240;

    // ------------------------------------------------------------------------
    // Tham so cua so truot
    // ------------------------------------------------------------------------

    // So cua so, moi cua so cao bao nhieu hang
    static constexpr int N_WINDOWS = 5;

    // Ban cot toi da de doi chieu moi cua so
    static constexpr int WINDOW_MARGIN = 30;

    // It nhat bao nhieu pixel vanh trong mot cua so moi coi la "co vanh"
    static constexpr int WINDOW_MIN_POINTS = 8;

    // Do rong toi da cua mot vanh. Lon hon gia tri nay -> bo qua (bong do,
    // vat can, goc phong) thay vi coi nhu la vanh duong.
    static constexpr int MAX_RUN_WIDTH = 25;

    // Hai vanh phai tach nhau it nhat bao nhieu pixel, neu khong thi co the
    // mot khoi bi tach thanh hai dinh va tao hieu nham la 2 vanh.
    static constexpr int MIN_LANE_GAP = 30;

    // Be rong giua 2 vanh o cua so thap nhat (diem rong nhat trong vung canh)
    static constexpr int LANE_WIDTH_MIN = 50;

    // PHAI lon hon be rong cua lan o cua so day, neu khong xe se tu dung.
    // Voi hang 233 va cam 0.30m: lan 0.50m ra 188 px, lan 0.61m ra 230 px.
    // Muon keo ROI_BOTTOM_FRAC xuong hon thi phai tang han so nay theo.
    static constexpr int LANE_WIDTH_MAX = 230;

    // Can it nhat bao nhieu cua so co du ca 2 phia moi tinh la "2 vanh"
    static constexpr int MIN_MATCHED_WINDOWS = 3;

    // So cua so thu de tim cap 2 vanh (tinh tu cua so thap nhat di len)
    static constexpr int MAX_SEED_SEARCH = 3;

    // ------------------------------------------------------------------------
    // Tien xu ly anh
    // ------------------------------------------------------------------------

    // Loc Gaussian truocc khi nguong, bo pixel nhieu ma giu vien
    static constexpr int BLUR_KSIZE = 5;

    // CLAHE keo duong sang vung toi chang deu, giu vien khong bi chet o cho toi
    static constexpr double CLAHE_CLIP = 2.0;
    static constexpr int CLAHE_TILE = 8;

    // ------------------------------------------------------------------------
    // Hinh hoc de do centimet
    // ------------------------------------------------------------------------

    // Chieu cao cam len mat duong. Do chinh xac cua don vi cm ty le voi so nay.
    static constexpr float CAMERA_HEIGHT_M = 0.30f;

    // Hang chan troi trong anh WORK_W x WORK_H. Do lai bang cach nhin cho
    // troi gap duong, dung 120 neu khong do duoc.
    static constexpr int HORIZON_Y = 120;

    // Hang lay dev: nam giua vung duoi cham troi, dung de nhin truoc
    static constexpr float LOOKAHEAD_FRAC = 0.58f;

    // ------------------------------------------------------------------------
    // Vung lam viec
    // ------------------------------------------------------------------------

    // Camera cao 0.30m nen chan troi nam quanh hang 120. Hang 108 (frac 0.45)
    // nam tren chan troi tuc la cua so do vai troi. Chan de lai 0.58.
    static constexpr float ROI_TOP_FRAC = 0.58f;

    // Keo toi sat dau xe. Cua so day cung la noi lay seed, o day 2 vanh
    // rong nhat nen seed tin cay hon.
    static constexpr float ROI_BOTTOM_FRAC = 0.97f;

    // ------------------------------------------------------------------------
    // Loc va do tre
    // ------------------------------------------------------------------------

    // Lam mem gia tri dev de chong rung lai
    static constexpr float EMA_ALPHA = 0.35f;

    // Tuoi cua frame moi nhat vuot qua gia tri nay -> coi nhu mat camera
    static constexpr int STALE_AGE_MS = 200;

    // So frame doc lien tiep that bai truoc khi bo cua so
    static constexpr int MAX_READ_FAIL = 25;

    // ------------------------------------------------------------------------
    // Ket qua mot lan nhan
    // ------------------------------------------------------------------------

    struct LaneOutput {
        // Duong 2 vanh, toa do trong khung WORK_W x WORK_H
        std::vector<cv::Point> left_pts;
        std::vector<cv::Point> right_pts;

        // True chi khi tim duoc ca 2 vanh o du N_MATCHED_WINDOWS cua so tro len
        bool two_lanes = false;

        // Do lech tam 2 vanh so voi chinh giua anh, don vi pixel anh goc.
        // Chi co y nghia khi two_lanes = true, neu la = 0.
        int dev_px = 0;

        // Cung thong tin tren nhung theo don vi centimet, chi de hien thi.
        // Khong gui xuong ESP32: firmware van nhan dev_px.
        float dev_cm = 0.0f;
        float lane_width_cm = 0.0f;

        // True neu fit duong bac 2 thanh cong. Neu false, dev_px lay tu
        // trung vi cua 5 cua so nhu phien ban truoc.
        bool fit_ok = false;

        // Tuoi frame moi nhat (ms), -1 neu chua co frame nao
        unsigned long age_ms = 0;
        bool stale = true;

        // Anh co ve de quan sat
        cv::Mat vis;

        // Thoi gian xu ly frame vua roi (ms)
        double proc_ms = 0.0;

        long frame_id = 0;
    };

    // ------------------------------------------------------------------------
    // Vong doi
    // ------------------------------------------------------------------------

    // camera_index < 0 -> tu doi kiem tra cac cong may 0..3
    explicit CameraLane(
        int camera_index = -1,
        int target_fps = 30
    );

    ~CameraLane();

    CameraLane(const CameraLane&) = delete;
    CameraLane& operator=(const CameraLane&) = delete;

    // Mo camera va bat luong doc. Tra false neu khong mo duoc camera nao.
    bool start();

    void stop();

    bool is_running() const;

    // ------------------------------------------------------------------------
    // Lay ket qua frame moi nhat
    // ------------------------------------------------------------------------

    // copy_vis: vong dieu khien 100 Hz khong can anh, chi luong ve 10 Hz moi
    // can, nen mac dinh false de khong copy ma 640x480 moi frame.
    void get_latest(
        LaneOutput& out,
        bool copy_vis = false
    ) const;

    // ------------------------------------------------------------------------
    // Tinh chinh
    // ------------------------------------------------------------------------

    // 0.0..1.0, phan tram tren cua khung anh. Mac dinh ROI_TOP_FRAC.
    void set_roi_top_frac(float frac);

private:

    // ------------------------------------------------------------------------
    // Camera
    // ------------------------------------------------------------------------

    bool open_camera();

    void capture_loop();

    // ------------------------------------------------------------------------
    // Detector
    // ------------------------------------------------------------------------

    bool detect(
        const cv::Mat& frame,
        LaneOutput& out
    );

    // Tim cap 2 vanh trong histogram cot cua 1 cua so.
    // Tra false neu khong tach duoc 2 vanh sach.
    static bool seed_pair_from(
        const cv::Mat& col_sum,
        int& seed_left,
        int& seed_right
    );

    // Tim dinh cot cua 1 vanh trong ban [seed - margin, seed + margin].
    static bool peak_in(
        const cv::Mat& col_sum,
        int seed,
        int& peak
    );

    // Fit x = a*t^2 + b*t + c qua cac diem, t = (y - y0) / (y1 - y0).
    // Toa do y chuan hoa ve [0,1] truoc khi giai de so lon khong phinh to.
    // y0, y1 tra ve de danh gia duong fit o hang bat ky. Tra false neu that bai.
    static bool fit_poly2(
        const std::vector<cv::Point>& pts,
        double coef[3],
        int& y0,
        int& y1
    );

    // Do rong theo centimet cua mot doan ngang pixel, do tai hang y.
    static float px_to_cm(float px, int y);

    // ------------------------------------------------------------------------
    // Trang thai
    // ------------------------------------------------------------------------

    int camera_index_;
    int target_fps_;

    std::atomic<float> roi_top_frac_{ROI_TOP_FRAC};

    cv::VideoCapture cap_;

    std::atomic<bool> running_{false};
    std::thread worker_;

    mutable std::mutex mtx_;

    LaneOutput latest_;

    // Gia tri dev da lam mem, chi dung trong luong doc
    int dev_ema_ = 0;

    // CLAHE tao 1 lan, dung lai moi frame. Tao moi frame la ton CPU.
    cv::Ptr<cv::CLAHE> clahe_;

    long frame_id_ = 0;

    std::chrono::steady_clock::time_point last_frame_time_{};
};