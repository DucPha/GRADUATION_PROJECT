#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ============================================================================
// CAMERA LANE - nhan lan tren MAT SAN (BEV, don vi met), port tu lane_bev.py
// ----------------------------------------------------------------------------
// Camera doc MJPG (mac dinh 1920x1080), tu giai ma bang libjpeg-turbo o 1/2
// kich thuoc, thu ve khung lam viec WORK_W = 320 cot (giu dung ti le anh).
//
// HINH HOC CAMERA (do tren anh that cua xe, log Python 2026-10-07/08):
//   - camera CUI XUONG ~42 do, cao ~0.30 m -> chan troi nam TREN mep anh, mat
//     san nhin thay tu ~0.13 m toi ~1.1 m truoc chan camera. (Ban cu gia dinh
//     chan troi o 20% chieu cao anh ~ cui 17 do -> moi khoang cach / be rong
//     sai 2-3 lan, lan 0.42 m bi tinh thanh ~1 m -> loai cap vach.)
//   - che do 1920x1080 va 640x480 cua cam nay co CUNG goc nhin DOC (4:3 chi
//     cat bot 2 ben) -> profile dung goc doc vfov, dung cho moi do phan giai.
//
// Quy trinh moi frame (giong lane_bev.py da chay tren xe):
//   1. Chieu anh xuong luoi mat san 1 cm/px (remap, bang tra tinh san)
//   2. Mask vach TOI: toi hon nen (closing 13 cm) theo TI LE + toi hon MUC SAN
//      xung quanh (phan vi 40%) -> chong loa den tran, khe gach giua 2 vet sang
//   3. Loc do vat: xoa phan DAY cua mask truoc (giay, o cam, ghe...) de vach
//      dinh vao vat tach ra; bo vet hinh NEM chia ve camera (chan ban/ghe),
//      net manh + nhat (khe gach). Bang keo bong loa den tran -> noi qua.
//      Ghep cap moi: xe phai nam GIUA vach trai va vach phai.
//   4. Bam lai 2 vach frame truoc (du doan theo odometry), DI DOC vach theo
//      cua so truot, het vach thi thu cac huong GAP (25-120 do) -> theo duoc
//      goc cua 90 do. Thieu vach -> tim vach moi, ghep cap dung be rong lan.
//   5. Tam lan = vach doi nua lan theo PHAP TUYEN, noi goc kieu MITRE -> o goc
//      gap 90 do duong tam di dung giua goc, khong cat qua vach trong.
//   6. Pure pursuit tu truc sau -> dev_px (thang do firmware), phat hien goc
//      gap phia truoc -> he so toc do.
//
// Toa do mat dat: X (m) sang PHAI, Z (m) ve PHIA TRUOC, goc = diem tren san
// ngay duoi camera.
//
// QUY UOC dev_px: luon tinh theo anh THAM CHIEU rong DEV_REF_W = 640 px tai
// khoang cach DEV_REF_DIST_M (1 px ~ DEV_M_PER_PX met), bat ke do phan giai.
// Firmware (deadzone, bao hoa) duoc chinh theo thang do nay.
// ============================================================================

enum class LaneState {
  LOST = 0,     // khong thay vach nao dung duoc
  ONE_LINE = 1, // chi thay 1 vach (da tung la 1 cap lan hop le)
  TWO_LINES = 2 // thay du 2 vach
};

// ----------------------------------------------------------------------------
// Thong so lap dat camera. Doi cam chi can doi struct nay, khong sua .cpp.
// ----------------------------------------------------------------------------
struct CameraProfile {
  // Do cao tam ong kinh so voi mat san (m)
  float height_m = 0.30f;

  // Goc cui cua truc quang so voi phuong ngang (do), CO DINH. Do bang diem
  // tu cua 2 dai thang song song tren anh that (doi goc camera -> do lai).
  // (Ban tu hieu chinh khi chay da bo: tren duong that no nhay 38-46 do.)
  float pitch_deg = 35.5f; // toi 09/10 camera ngua len (08/10: 42)

  // Goc nhin DOC (do). Tieu cu 250 px o anh 320x240 (lane_bev.py) = 51 do.
  float vfov_deg = 51.0f;

  // Hinh hoc xe (du doan vach giua 2 frame, pure pursuit tu truc sau)
  float cam_to_rear_m = 0.18f; // chan camera nam truoc truc sau (do lai tren xe 2026-10-08)
  float wheelbase_m = 0.26f;

  // Vung anh dung de nhan lan, ti le chieu cao anh tinh tu tren xuong
  float roi_top_frac = 0.0f;
  float roi_bottom_frac = 0.98f;
};

class CameraLane {

public:
  // ------------------------------------------------------------------------
  // Kich thuoc xu ly
  // ------------------------------------------------------------------------

  // Be ngang khung lam viec. Chieu cao = WORK_W * frame_h / frame_w.
  static constexpr int WORK_W = 320;

  // Anh tham chieu cho dev_px (xem dau file)
  static constexpr int DEV_REF_W = 640;
  static constexpr float DEV_REF_DIST_M = 0.65f;
  static constexpr float DEV_M_PER_PX = 0.00155f;

  // Anh quan sat gui cho GUI duoc thu ve be ngang nay
  static constexpr int VIS_W = 640;

  // ------------------------------------------------------------------------
  // Luoi mat san (BEV)
  // ------------------------------------------------------------------------

  static constexpr float BEV_RES_M = 0.01f;   // 1 o luoi = 1 cm
  static constexpr float BEV_X_HALF_M = 0.80f;
  static constexpr float BEV_Z_MAX_M = 1.60f; // xa hon khong xet
  // Be rong lan (tam vach - tam vach) KHONG cai dat: do moi frame thay du 2
  // vach. Ghep cap chap nhan LANE_W_MIN_M..LANE_W_MAX_M. LANE_W_INIT_M chi dung
  // truoc lan do dau tien (xe xuat phat thuong thay du 2 vach -> do ngay).
  static constexpr float LANE_W_MIN_M = 0.28f;
  static constexpr float LANE_W_MAX_M = 0.70f;
  static constexpr float LANE_W_INIT_M = 0.45f;
  // Be rong tai cho: EMA moi frame du 2 vach; khong thay du 2 vach qua
  // LANE_W_LOCAL_HOLD_SEC thi tro dan ve be rong hoc dai han
  static constexpr float LANE_W_LOCAL_ALPHA = 0.35f;
  static constexpr float LANE_W_LOCAL_HOLD_SEC = 3.0f;

  // ------------------------------------------------------------------------
  // Mask vach toi (cung thong so voi Python)
  // ------------------------------------------------------------------------

  static constexpr float BG_KERNEL_M = 0.13f;     // > be rong vach
  static constexpr float DARK_RATIO_INIT = 0.15f; // toi hon nen >= 15%
  static constexpr float DARK_RATIO_MIN = 0.10f;
  static constexpr float DARK_RATIO_MAX = 0.35f;
  static constexpr int DARK_MIN_ABS = 10;         // va >= 10 muc xam
  // Bang keo den toi hon muc san ~60-75%; khe gach, vien sang quanh vet loa
  // den, mep bong vat chi toi hon 10-20% -> doi >= 25%. (Python dung 0.10;
  // thu tren log that 0.10 rung gap ~3 lan khi xe dung giua 2 vach)
  static constexpr float FLOOR_DARK_RATIO = 0.25f;
  static constexpr float FLOOR_PCT = 0.40f;        // muc san = phan vi 40%
  static constexpr float FLOOR_BLOCK_W_M = 0.40f;
  static constexpr float FLOOR_BLOCK_H_M = 0.20f;
  static constexpr float NOISE_FILL = 0.10f;      // mask > 10% vung nhin = nhieu
  // Nguong toi = CONTRAST_FRAC x do tuong phan cua chinh vach dang bam
  static constexpr float CONTRAST_FRAC = 0.35f;
  static constexpr float CONTRAST_THR_MAX = 0.25f;

  // LOC MAU: vach la bang keo DEN (bao hoa mau thap). Pixel du sang ma bao
  // hoa > COLOR_MAX_SAT (0..255) -> khong phai vach (ghe cam, vat mau).
  static constexpr int COLOR_MAX_SAT = 100;
  static constexpr int COLOR_MIN_V = 60;

  // ------------------------------------------------------------------------
  // Loc do vat
  // ------------------------------------------------------------------------

  static constexpr int MIN_AREA_PX = 12;
  // Bang keo den tren san ~7 cm, nhung tren mask BEV day 8-10 cm (nhoe anh,
  // goc cui lech vai do, mep toi). Phan day hon TAPE_MAX_THICK_M ma khong
  // thuon dai, hoac day hon BLOB_THICK_M = vat khac. (Ban truoc 0.07 / 0.14
  // dung bang be rong bang keo -> doan vach gan xe / goc gap bi loai.)
  static constexpr float TAPE_MAX_THICK_M = 0.10f;
  static constexpr float BLOB_THICK_M = 0.18f;
  static constexpr float RADIAL_DEG = 8.0f;
  static constexpr float RADIAL_NEAR_GAP_M = 0.12f;
  // Doan vach chia thang ve chan camera (lech <= RADIAL_LINE_DEG) dai >=
  // RADIAL_LINE_MIN_M = chan ban / ghe (vat dung dung chieu xuong san thanh
  // tia ve camera). Bang keo chi chia ve camera khi xe nam tren duong keo dai
  // cua no. Loc khoi (filter_clutter) khong bat duoc khi chan ban CHAM vach
  // (dinh chung 1 khoi). 2 truong hop (ban 09/10 15:47):
  //  - doan dau vach, bat dau cach mep duoi tam nhin >= RADIAL_LINE_GAP_M
  //    (chan ban dung rieng) -> bo ca vach,
  //  - doan sau 1 goc gap >= RADIAL_KINK_DEG (di doc bang keo roi re vao chan
  //    ban dung sat vach) -> cat tu goc gap.
  static constexpr float RADIAL_LINE_DEG = 12.0f;
  static constexpr float RADIAL_LINE_MIN_M = 0.12f;
  static constexpr float RADIAL_LINE_GAP_M = 0.06f;
  static constexpr float RADIAL_KINK_DEG = 30.0f;
  // TACH KHOI DAY: vung nao chua vua 1 hinh tron duong kinh BLOB_OPEN_M
  // (giay, o cam, ghe, tui...) la vat, khong phai bang keo -> xoa vung do
  // (kem vien BLOB_EAT_PX) truoc khi chia thanh phan. Nho vay vach dinh vao
  // vat chi mat doan cham vat chu khong bi loai ca vach. O GOC GAP (chu L)
  // bang keo chua vua hinh tron ~1.2 lan be rong -> khoi "than vat" nho hon
  // BODY_CORNER_DISCS hinh tron, hoac dai manh (bang keo nhoe), duoc GIU.
  // (Ban truoc xoa ca goc cua -> vach dut doi, duong tam di thang ra ngoai.)
  static constexpr float BLOB_OPEN_M = 0.09f;
  static constexpr float BODY_CORNER_DISCS = 2.5f;
  static constexpr int BLOB_EAT_PX = 2;
  // Net qua manh (khe gach, vet nut: nua be rong < STROKE_MIN_HALF_PX) ma
  // khong du toi (rel < STROKE_THIN_REL) -> khong phai bang keo
  static constexpr float STROKE_MIN_HALF_PX = 1.4f;
  static constexpr float STROKE_THIN_REL = 0.35f;

  // LOA DEN TRAN: pixel san sang gan bao hoa. Bang keo bong phan chieu den
  // thanh vet trang -> vach dut. Di doc vach ma gap vung loa thi cho noi qua
  // xa hon (GLARE_BRIDGE_STEPS buoc thay vi 3).
  static constexpr int GLARE_MIN_V = 245; // = LINE_GLARE_LEVEL cua Python (chay sang)
  static constexpr float GLARE_FLOOR_GAIN = 1.20f; // hoac sang hon muc san 20%
  static constexpr int GLARE_BRIDGE_STEPS = 8;     // 8 x 3 cm = 24 cm
  // Vung loa de "to den lai" vach: nen cuc bo sang hon muc san cua khoi >= 10%
  static constexpr float GLARE_ZONE_GAIN = 1.10f;
  // Khoi mask nam trong dai +-CORRIDOR_HALF_M quanh vach dang bam (du doan)
  // it nhat CORRIDOR_FRAC dien tich -> giu du manh / nhat / chia ve camera
  static constexpr float CORRIDOR_HALF_M = 0.04f;
  static constexpr float CORRIDOR_FRAC = 0.7f;
  static constexpr float CORRIDOR_OTHER_HALF_M = 0.07f; // vach kia (du doan tu be rong lan)
  // Cho dut binh thuong noi toi da 2 buoc (6 cm). Ban truoc 3 buoc (9 cm):
  // dau vach nhay sang bui day cap / to giay nam gan -> vach cong queo.
  static constexpr int GAP_STEPS = 2;
  // Noi xa qua vung loa: diem noi lech duong keo dai toi da BRIDGE_MAX_OFF_M,
  // 3 buoc sau do moi buoc re toi da BRIDGE_MAX_TURN_DEG
  static constexpr float BRIDGE_MAX_OFF_M = 0.025f;
  static constexpr float BRIDGE_MAX_TURN_DEG = 20.0f;

  // ------------------------------------------------------------------------
  // Do vach
  // ------------------------------------------------------------------------

  static constexpr float WALK_STEP_M = 0.03f;
  static constexpr int WIN_PX = 3;
  static constexpr int MIN_WIN_PX = 4;
  // Pixel MOI trong cua so moi buoc (phan da di qua bi an): bang keo 7 cm ~14,
  // khe gach 1-2 cm ~2-4. Goc gap: doan sau goc phai co >= KINK_MIN_PX (bang
  // keo, khong phai khe gach). Di thang ma so pixel tut duoi THIN_DROP_FRAC
  // lan muc TB cua vach THIN_DROP_STEPS buoc lien tiep = het bang keo, dang
  // lan sang khe gach -> dung.
  static constexpr int KINK_MIN_PX = 12;
  static constexpr int THICK_REF_PX = 9;
  static constexpr float THIN_DROP_FRAC = 0.40f;
  static constexpr int THIN_DROP_STEPS = 3;
  static constexpr float SEED_RADIUS_M = 0.06f;
  static constexpr float SEED_SEARCH_M = 0.60f;
  static constexpr float MAX_LINE_M = 2.5f;
  static constexpr float TRACK_GATE_M = 0.07f; // vach moi lech xa hon du doan -> vat khac
  // Ghep cap moi: vach trai duoc lan sang phai chan camera toi da bay nhieu
  // (va nguoc lai) - xe phai nam giua 2 vach
  static constexpr float CAR_SIDE_SLACK_M = 0.05f;
  static constexpr float COAST_SEC = 0.5f;     // mat vach ngan hon: giu vach du doan
  // Vach dang du doan (bi che) hien lai cach du doan < REACQ_GATE_M -> gan lai
  static constexpr float REACQ_GATE_M = 0.15f;
  static constexpr float REACQ_MAX_DEG = 30.0f;
  // ... hoac la doan SAU GOC GAP (goc nam duoi vet loa): keo dai nguoc vach
  // moi cham vach du doan (< REACQ_KINK_MEET_M), lech huong < REACQ_KINK_MAX_DEG
  static constexpr float REACQ_KINK_MEET_M = 0.06f;
  static constexpr float REACQ_KINK_MAX_DEG = 100.0f;
  // Chi thay 1 vach tu dau (xe lech, vach kia ra khoi khung): bam duoc lien
  // tiep bay nhieu frame thi dung de lai (Python: LEFT_ONLY / RIGHT_ONLY dung
  // ngay). Truoc day 1 vach chua tung ghep cap = LOST -> xe dung han, khong
  // bao gio tu chay lai.
  static constexpr int SINGLE_CONFIRM_FRAMES = 3;
  static constexpr float RDP_EPS_M = 0.015f;
  // Doan vach sau goc gap phai dai it nhat bay nhieu moi la goc cua that
  static constexpr float MIN_KINK_TAIL_M = 0.10f;

  // ------------------------------------------------------------------------
  // Lai (pure pursuit tu truc sau) -> dev_px
  // ------------------------------------------------------------------------

  static constexpr float PP_LOOKAHEAD_M = 0.55f;
  static constexpr float PP_LOOKAHEAD_CORNER_M = 0.80f; // thay goc gap phia truoc
  static constexpr float PP_MIN_FWD_M = 0.25f;
  static constexpr float CORNER_KINK_DEG = 30.0f; // tam lan gap >= bay nhieu = goc cua

  // Gioi han dev gui xuong (px anh 640)
  static constexpr int DEV_MAX_REF_PX = 200;

  // Chan nhay: dev moi lech dev dang loc qua JUMP_GATE_REF_PX thi giu gia tri
  // cu; lech nhu vay JUMP_CONFIRM_FRAMES frame lien tiep thi moi tin.
  static constexpr int JUMP_GATE_REF_PX = 90;
  static constexpr int JUMP_CONFIRM_FRAMES = 2;

  // EMA cua dev: lon = nhay. 2 vach (tin cay) loc nhe, 1 vach loc vua.
  // 0.70 / frame o 30 fps = CX_EMA_ALPHA 0.85 / frame o 20 fps cua Python
  // (cung hang so thoi gian ~26 ms).
  static constexpr float EMA_ALPHA_TWO = 0.70f;
  static constexpr float EMA_ALPHA_ONE = 0.50f;
  // Dev da loc doi toi da bay nhieu px / frame. Python MAX_CENTER_STEP_PX
  // 25 px / frame (anh 320, 20 fps) ~ 0.72 m/s ngang -> 15 px ref / frame
  // o 30 fps: 1 frame nhan sai khong lam xe giat lai.
  static constexpr int DEV_MAX_STEP_REF_PX = 15;

  // Doi nguon tam lan (2 vach <-> 1 vach trai <-> 1 vach phai): tam tinh
  // tu 1 vach lech tam tinh tu 2 vach vai cm (be rong lan uoc luong) ->
  // giu tam lien mach, phan chenh giam dan ve 0 voi hang so STATE_BLEND_SEC.
  // Python: STATE_BLEND_SEC 0.5 s, STATE_BLEND_MAX_PX 40 px (~6 cm).
  static constexpr float STATE_BLEND_SEC = 0.5f;
  static constexpr float STATE_BLEND_MAX_M = 0.06f;

  // ------------------------------------------------------------------------
  // He so toc do de nghi (speed_scale: 1 = duong thang, 0 = cua gat nhat)
  // ------------------------------------------------------------------------

  static constexpr float CURVE_HEADING_START_DEG = 15.0f;
  static constexpr float CURVE_HEADING_FULL_DEG = 50.0f;
  static constexpr float CURVE_CHORD_M = 0.20f;
  static constexpr float CURVE_K_START = 0.40f; // 1/m (R 2.5 m)
  static constexpr float CURVE_K_FULL = 1.50f;  // 1/m (R 0.67 m)
  // Goc gap phia truoc (tinh tu truc sau): gan hon CORNER_SLOW_M -> cham nhat
  static constexpr float CORNER_SLOW_M = 0.70f;
  static constexpr float CORNER_SLOW_START_M = 1.20f;

  // ------------------------------------------------------------------------
  // Camera
  // ------------------------------------------------------------------------

  static constexpr int STALE_AGE_MS = 200;
  static constexpr int MAX_READ_FAIL = 25;
  static constexpr int REOPEN_PERIOD_MS = 2000;

  // Khoa phoi sang (camera_exposure = 0): do sang san muc tieu va gioi han
  // exposure. Giong Python: CAMERA_TARGET_BRIGHTNESS 110, bat dau 300, dai
  // 80..1500. (Do tren cam nay: 1080p MJPG ra ~25.5 fps o MOI muc exposure
  // 150..1000, nen exposure khong lam giam fps.)
  static constexpr double EXPOSURE_TARGET = 110.0;
  static constexpr int EXPOSURE_START = 300;
  static constexpr int EXPOSURE_MIN = 80;
  static constexpr int EXPOSURE_MAX = 1500;

  // Mat lan: giu dev_px cuoi toi da bay nhieu ms
  static constexpr int HOLD_MS = 500;

  // ------------------------------------------------------------------------
  // Ket qua mot lan nhan
  // ------------------------------------------------------------------------

  struct LaneOutput {
    // 2 vach da chon, toa do khung lam viec (gan -> xa)
    std::vector<cv::Point> left_pts;
    std::vector<cv::Point> right_pts;

    bool two_lanes = false; // true chi khi state == TWO_LINES
    LaneState state = LaneState::LOST;

    // Lenh lai, px anh THAM CHIEU 640 (xem dau file). > 0: lai phai.
    // Khi LOST: giu gia tri cuoi toi da HOLD_MS.
    int dev_px = 0;

    // Do lech ngang tuong duong (cm), chi de hien thi
    float dev_cm = 0.0f;
    float lane_width_cm = 0.0f;

    // Do cong duong tam (1/m). > 0: re phai, < 0: re trai.
    float curvature = 0.0f;

    // Lech ngang (cm) cua duong tam doan xa so voi doan gan
    float path_dx_cm = 0.0f;

    // 1 = duong thang, 0 = cua gat nhat. Node dieu khien noi suy toc do
    // giua speed_corner va speed theo he so nay.
    float speed_scale = 1.0f;

    bool fit_ok = false;

    // Goc gap phia truoc tren duong tam: khoang cach tu truc sau (m, < 0 =
    // khong co), huong (+1 phai, -1 trai), do gap (do)
    float kink_dist_m = -1.0f;
    int kink_dir = 0;
    float kink_deg = 0.0f;

    // Toa do MAT DAT (m, goc = chan camera, X phai, Z truoc), gan -> xa.
    // PathTracker dung de nho duong qua vung mu truoc xe. Rong = khong thay.
    std::vector<cv::Point2f> centre_g; // duong tam lan, lay mau 5 cm
    std::vector<cv::Point2f> left_g;   // vach trai
    std::vector<cv::Point2f> right_g;  // vach phai
    std::chrono::steady_clock::time_point stamp{}; // luc nhan frame

    // True khi frame nay bi chan nhay (dev giu gia tri cu)
    bool gated = false;

    float horizon_frac = 0.0f; // chan troi (ti le chieu cao anh, < 0 = tren mep anh)
    float pitch_deg = 0.0f;    // goc cui camera dang dung
    float near_z_m = 0.0f;     // mep gan nhat camera thay duoc (m truoc chan camera)
    float dark_ratio = 0.0f;   // nguong toi dang dung

    unsigned long age_ms = 0;
    bool stale = true;

    // Kich thuoc anh goc camera
    int frame_w = 0;
    int frame_h = 0;

    // Anh cho GUI. Chi co khi get_latest(copy_vis = true).
    cv::Mat vis; // anh camera + overlay detector (rong VIS_W)
    cv::Mat raw; // anh camera chua ve (rong VIS_W)
    cv::Mat roi; // anh mat san (BEV) + overlay
    cv::Mat bin; // mask vach 0/1 tren luoi mat san
    long vis_frame_id = 0;

    double proc_ms = 0.0;
    long frame_id = 0;
  };

  // ------------------------------------------------------------------------
  // Vong doi
  // ------------------------------------------------------------------------

  // camera_index < 0 -> tu do 0..3. width/height = do phan giai yeu cau.
  // start() tra false neu chua mo duoc camera ngay, nhung luong camera van
  // chay va tu thu mo lai moi REOPEN_PERIOD_MS.
  explicit CameraLane(int camera_index = -1, int target_fps = 30,
                      int width = 1920, int height = 1080,
                      const CameraProfile &profile = CameraProfile{});

  ~CameraLane();

  CameraLane(const CameraLane &) = delete;
  CameraLane &operator=(const CameraLane &) = delete;

  bool start();
  void stop();
  bool is_running() const;

  // copy_vis = false: vong dieu khien khong copy anh. copy_vis = true con
  // bao cho luong camera ve anh quan sat o frame ke tiep.
  void get_latest(LaneOutput &out, bool copy_vis = false) const;

  void set_roi_top_frac(float frac);

  // GHI ANH khi chay (de chinh detector bang anh duong dua that): moi frame
  // da giai ma (rong VIS_W, dung anh detector xu ly) luu JPEG vao dir, kem
  // frames.csv (thoi gian, trang thai lan, be rong, toc do, goc banh) de phat
  // lai offline. Ghi o luong rieng, day hang doi thi bo frame (khong lam
  // cham detector). dir rong = tat. set_recording(true) chi khi xe dang chay.
  void set_record_dir(const std::string &dir);
  void set_recording(bool on) { rec_on_.store(on); }

  // Toc do xe (m/s) va goc banh (do, > 0 = phai) hien tai: de du doan vi tri
  // vach giua 2 frame. Goi tu vong dieu khien, an toan da luong.
  void set_motion(float v_mps, float wheel_deg) {
    v_mps_.store(v_mps);
    wheel_deg_.store(wheel_deg);
  }

  // Xu ly 1 anh BGR co san (khong can camera). Goi tuan tu tu 1 luong,
  // KHONG goi khi start() dang chay. stamp: thoi diem chup (mo phong / chay
  // lai log nhanh hon thoi gian thuc); mac dinh = dong ho that.
  bool process(const cv::Mat &bgr, LaneOutput &out, bool draw = true,
               std::chrono::steady_clock::time_point stamp = {});

  // Phoi sang (V4L2 exposure_time_absolute, don vi 100 us). > 0 = co dinh,
  // 0 = tu do roi KHOA (xem lock_exposure), < 0 = de camera tu dong.
  // Goi TRUOC start().
  void set_manual_exposure(int value) { exposure_ = value; }

private:
  using Poly = std::vector<cv::Point2f>;

  // Mot vach dang bam (toa do mat dat, gan -> xa)
  struct Track {
    bool valid = false;
    Poly pts;
    bool observed = false; // thay o frame nay
    float missed = 0.0f;   // so giay lien tuc khong thay
    bool verified = false; // da tung la 1 cap lan hop le (khong phai nhieu)
    int seen = 0;          // so frame LIEN TIEP thay vach nay
    float gate_err = 0.0f;
  };

  bool open_camera();
  void capture_loop();

  // Doc 1 khung va dua ve BGR (tu giai ma MJPG neu driver tra goi nen)
  bool grab_frame(cv::Mat &raw, cv::Mat &bgr);
  void lock_exposure();
  double floor_brightness();

  // Tinh lai tieu cu / chan troi / luoi BEV khi biet kich thuoc anh
  void update_geometry(int frame_w, int frame_h);
  void set_pitch(double pitch_deg);
  void build_bev();

  bool detect(const cv::Mat &frame, LaneOutput &out, bool draw,
              std::chrono::steady_clock::time_point stamp = {});

  // ---- BEV (lane_bev.cpp) ----
  void line_mask(const cv::Mat &bev_bgr, cv::Mat &mask, cv::Mat &rel,
                 cv::Mat &glare) const;
  cv::Mat floor_level(const cv::Mat &gray) const;
  void filter_clutter(const cv::Mat &mask, const cv::Mat &rel, cv::Mat &clean,
                      cv::Mat &rejected) const;
  bool in_glare(const cv::Point2f &g) const;
  // Vi tri ngang (m, > 0 = ben phai) cua duong keo dai doan dau vach tai
  // ngang chan camera
  static float x_at_car(const Poly &p);

  void px_of(const cv::Point2f &g, float &c, float &r) const;
  cv::Point2f ground_of(float c, float r) const;
  bool inside(const cv::Point2f &g) const;
  bool window(const cv::Mat &work, const cv::Point2f &p, int r,
              cv::Point2f &mean, int &count) const;
  void consume(cv::Mat &work, const cv::Point2f &p, int r) const;
  Poly walk(cv::Mat &work, cv::Point2f p, cv::Point2f d, float max_len,
            bool allow_kink) const;
  Poly trace_from(cv::Mat &work, const cv::Point2f &seed,
                  const cv::Point2f &d) const;
  bool follow(cv::Mat &work, const Poly &pred, float gate, Poly &pts,
              float &err) const;
  std::vector<Poly> candidates(cv::Mat &work, int max_n = 8) const;
  bool trim_upright(Poly &pts) const;

  void predict_tracks(float dt);
  void pair_geom(const Poly &a, const Poly &b, float &dist, float &ang,
                 bool &right) const;
  void acquire(const std::vector<Poly> &cands, bool have_left,
               bool have_right, bool allow_single);

  bool straight_head(const Poly &pts, float length, cv::Point2f &a,
                     cv::Point2f &b) const;
  void learn_width(const Poly &left, const Poly &right);
  void adapt_threshold(const cv::Mat &rel, const std::vector<const Poly *> &obs);

  bool target_on(const Poly &path, float L, cv::Point2f &target,
                 float &ext) const;
  bool kink_ahead(const Poly &path, float &dist, int &dir, float &deg,
                  cv::Point2f &pt) const;

  // ---- Hinh hoc mat dat ----
  bool ground_to_img(const cv::Point2f &g, cv::Point2d &p) const;
  static bool to_image(double pitch, double f, double h, double cx, double cy,
                       double X, double Z, double &u, double &v);
  double ground_dist_m(double y) const;

  static float dist_to_polyline(const cv::Point2f &p, const Poly &poly,
                                bool *interior = nullptr,
                                float *side = nullptr);
  static Poly resample(const Poly &p, float step);

  // ------------------------------------------------------------------------
  // Trang thai
  // ------------------------------------------------------------------------

  int camera_index_;
  int target_fps_;
  int req_w_;
  int req_h_;
  CameraProfile profile_;
  int exposure_ = -1;
  int locked_exposure_ = 0; // gia tri da do, dung lai khi mo lai camera

  bool raw_mjpg_ = false;
  bool quiet_open_ = false;
  int cam_w_ = 0; // do phan giai that cua camera
  int cam_h_ = 0;

  // Hinh hoc suy ra tu CameraProfile + kich thuoc anh
  int work_h_ = 180;
  int geom_w_ = 0;
  int geom_h_ = 0;
  float h_ = 0.30f;    // do cao cam (m)
  double pitch_ = 0.0; // goc cui cua truc quang (rad)
  double f_px_ = 0.0;  // tieu cu tren khung lam viec (px)
  double cy_ = 90.0;
  double horizon_y_ = 0.0;
  float roi_top_used_ = -1.0f;
  float roi_bottom_frac_ = 0.98f;
  bool logged_geometry_ = false;

  // Luoi BEV: cot = X, hang 0 = xa nhat (z_max_)
  int nx_ = 0;
  int nz_ = 0;
  float x_min_ = -BEV_X_HALF_M;
  float z_min_ = 0.1f;
  float z_max_ = 1.2f;
  cv::Mat mapx_, mapy_; // CV_32F, toa do khung lam viec
  cv::Mat valid_;       // CV_8U 0/255
  cv::Mat glare_;       // CV_8U 0/255, vung loa den cua frame dang xu ly
  int n_valid_ = 1;
  std::vector<float> z_near_col_; // Z gan nhat nhin thay theo tung cot

  std::atomic<float> roi_top_frac_{0.0f};
  std::atomic<float> v_mps_{0.0f};
  std::atomic<float> wheel_deg_{0.0f};

  cv::VideoCapture cap_;

  std::atomic<bool> running_{false};
  std::thread worker_;

  // ---- Ghi anh (xem set_record_dir) ----
  struct RecItem {
    cv::Mat img;
    std::string name;
    std::string csv;
  };
  void record_loop();
  void stop_recorder();
  std::string rec_dir_;
  std::atomic<bool> rec_on_{false};
  bool rec_running_ = false;
  std::thread rec_thread_;
  std::mutex rec_mtx_;
  std::condition_variable rec_cv_;
  std::deque<RecItem> rec_q_;
  static constexpr size_t REC_QUEUE_MAX = 60;

  mutable std::atomic<bool> vis_wanted_{true};

  mutable std::mutex mtx_;
  LaneOutput latest_;     // ket qua moi nhat (khong kem anh)
  LaneOutput latest_vis_; // bo anh quan sat moi nhat

  // ---- Cac bien duoi day chi dung trong luong camera ----
  float dev_ema_ = 0.0f;
  // Chuyen tam muot khi doi nguon vach (xem STATE_BLEND_SEC)
  int blend_key_ = 0;          // bit 1 = vach trai, bit 2 = vach phai
  float blend_off_m_ = 0.0f;   // do lech ngang dang cong vao tam lan
  float blend_prev_x_ = 0.0f;  // X diem ngam frame truoc (da cong lech)
  bool blend_have_ = false;
  bool ema_primed_ = false;
  int jump_count_ = 0;
  std::chrono::steady_clock::time_point last_valid_time_{};
  std::chrono::steady_clock::time_point last_detect_time_{};

  Track left_;
  Track right_;
  float lane_w_m_ = LANE_W_INIT_M;      // be rong hoc dai han (trung vi)
  float lane_w_local_ = LANE_W_INIT_M;  // be rong vua do (EMA), dung khi 1 vach
  float lane_w_local_age_ = 0.0f;       // s tu lan cuoi do duoc be rong
  bool have_width_ = false;             // da do duoc be rong lan nao chua
  std::deque<float> width_samples_;
  float dark_ratio_ = DARK_RATIO_INIT;
  float contrast_ = -1.0f;
  float last_fill_ = 0.0f;
  float speed_est_ = 0.0f;
  float dt_ = 0.04f;

  long frame_id_ = 0;

  std::chrono::steady_clock::time_point last_frame_time_{};
};
