#pragma once

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
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
//   - camera CUI XUONG ~36 do (09/10, ban dau 42), cao ~0.30 m -> chan troi
//     nam TREN mep anh, mat san nhin thay tu ~0.17 m toi ~1.6 m truoc chan
//     camera (BEV chi xet toi BEV_Z_MAX_M). (Ban cu gia dinh
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
//   6. Goc cui camera tu hieu chinh tu diem tu cua 2 vach thang.
//   7. Pure pursuit tu truc sau -> dev_px (thang do firmware), phat hien goc
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

  // Goc cui cua truc quang so voi phuong ngang (do). Do tren log that:
  // 2 vach thang keo dai gap nhau o hang -106 cua anh 320x240 -> ~42 do.
  // 09/10 camera duoc ngua len: diem tu 4 mep 2 vach thang tren anh live
  // 1920x1080 o hang -275 (anh 960x540) -> 35.6-36.0 do; tu hieu chinh cua
  // detector ra 35.89; be rong lan khong doi theo khoang cach -> 35.9.
  // auto_pitch = true: tu chinh lai khi chay thang thay 2 vach. MAC DINH TAT:
  // thu vong kin (goc that co dinh 42) no nhay 42 -> 44.5 -> 42.7 -> 44.5...
  // moi lan doi > 2 do con xoa vach dang bam -> mat lan giua duong.
  float pitch_deg = 35.9f;
  bool auto_pitch = false;
  float pitch_max_dev_deg = 8.0f; // chi nhan hieu chinh trong +- bay nhieu do

  // Goc nhin DOC (do). Tieu cu 250 px o anh 320x240 (lane_bev.py) = 51 do.
  float vfov_deg = 51.0f;

  // Be rong lan, tam vach toi tam vach (m). Tu hoc lai khi chay.
  float lane_width_m = 0.42f;

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

  // ------------------------------------------------------------------------
  // Mask vach toi (cung thong so voi Python)
  // ------------------------------------------------------------------------

  static constexpr float BG_KERNEL_M = 0.13f;     // > be rong vach
  static constexpr float DARK_RATIO_INIT = 0.15f; // toi hon nen >= 15%
  static constexpr float DARK_RATIO_MIN = 0.10f;
  static constexpr float DARK_RATIO_MAX = 0.42f;
  static constexpr int DARK_MIN_ABS = 10;         // va >= 10 muc xam
  // Bang keo den toi hon muc san ~60-75%; khe gach, vien sang quanh vet loa
  // den, mep bong vat chi toi hon 10-20% -> doi >= 25%. (Python dung 0.10;
  // thu tren log that 0.10 rung gap ~3 lan khi xe dung giua 2 vach)
  static constexpr float FLOOR_DARK_RATIO = 0.25f;
  static constexpr float FLOOR_PCT = 0.40f;        // muc san = phan vi 40%
  static constexpr float FLOOR_BLOCK_W_M = 0.40f;
  static constexpr float FLOOR_BLOCK_H_M = 0.20f;
  static constexpr float NOISE_FILL = 0.10f;      // mask > 10% vung nhin = nhieu
  // Nguong toi = CONTRAST_FRAC x do tuong phan cua chinh vach dang bam.
  // Do tren log that 08/10: bang keo toi hon nen ~0.66 (5% thap nhat 0.53),
  // deu tu 0.2 toi 1.0 m. BONG chan ban / ghe chi toi hon 0.25-0.40 -> ban
  // cu (0.35 x, toi da 0.25) nhan ca bong: bong dinh vao bang keo thanh khoi
  // day bi loai ca vach, hoac vet bong thanh "vach" gia. Nay 0.6 x (~0.40).
  static constexpr float CONTRAST_FRAC = 0.60f;
  static constexpr float CONTRAST_THR_MAX = 0.42f;
  // Do toi TRUNG VI doc vach (bo diem loa den) phai >= bay nhieu lan:
  //  - vach dang bam: do toi cua CHINH vach do o cac frame truoc (bang keo
  //    khong doi do toi; nhay sang mep vet bong thi nhat han),
  //  - vach moi: vach dang bam NHAT hon (2 vach co the sang khac nhau).
  // Log that: trung vi 0.66, 5% thap nhat 0.53 (~0.8 x).
  static constexpr float LINE_MIN_CONTRAST_FRAC = 0.75f;
  // Do tuong phan chi dung de loc khi con moi: mat het vach lau hon (anh
  // sang doi, vung anh toi hon) thi tam bo loc nay de bat lai vach that
  static constexpr float CONTRAST_STALE_SEC = 1.0f;

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
  // cua no. 2 truong hop:
  //  - doan dau vach, bat dau cach mep duoi tam nhin >= RADIAL_LINE_GAP_M
  //    (chan ghe dung rieng / dinh voi bong thanh chu V) -> bo ca vach,
  //  - doan sau 1 goc gap >= RADIAL_KINK_DEG (di doc bang keo roi re vao chan
  //    ghe dung sat vach) -> cat tu goc gap. Cung tron cua vach that khong
  //    co goc gap nen khong bi cat.
  static constexpr float RADIAL_LINE_DEG = 12.0f;
  static constexpr float RADIAL_LINE_MIN_M = 0.12f;
  static constexpr float RADIAL_LINE_GAP_M = 0.06f;
  static constexpr float RADIAL_KINK_DEG = 30.0f;
  // RANG CUA: 2 goc gap NGUOC chieu nhau (moi goc >= ZIGZAG_DEG) cach nhau
  // < ZIGZAG_GAP_M = mep vet bong / do vat, khong phai bang keo (bang keo dan
  // tay chi gap 1 chieu o goc cua)
  static constexpr float ZIGZAG_DEG = 35.0f;
  static constexpr float ZIGZAG_GAP_M = 0.12f;
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
  // Cho dut binh thuong noi toi da 2 buoc (6 cm). Ban truoc 3 buoc (9 cm):
  // dau vach nhay sang bui day cap / to giay nam gan -> vach cong queo.
  static constexpr int GAP_STEPS = 2;
  // VACH CHAY VAO DO VAT: di doc vach gap mat cat NGANG (tren mask da loc)
  // rong > WIDE_FRAC x be rong cua chinh vach (TB WIDE_REF_STEPS buoc dau roi
  // EMA) va > WIDE_MIN_M -> dung vach o do (WIDE_STEPS buoc). Cua so truot chi
  // rong ~ bang keo nen truoc day khong thay: vach bo theo to giay / vung bong
  // o dau xa -> "goc cua" gia, toc do tut ve 0. Thu tren log that 08/10 (2870
  // frame) + anh live 09/10: bo 20 goc cua gia (xem tung frame), khong mat
  // them frame nao; live: bao cua nham 19.6% -> 3.3%, giat dev 3.2 -> 0.9 px.
  // 1.6x / 6 cm bat dau mat vach that -> giu 2.0x / 8 cm.
  static constexpr float WIDE_FRAC = 2.0f;
  static constexpr int WIDE_STEPS = 1;
  static constexpr float WIDE_MIN_M = 0.08f;
  static constexpr int WIDE_REF_STEPS = 3;
  static constexpr int WIDE_AFTER_KINK = 2; // ngay sau goc gap (dinh chu L rong) bo qua
  // Duoi vach be >= TAIL_BEND_DEG (re tu tu, khong qua buoc tim goc gap) ma
  // doan sau cho be < MIN_KINK_TAIL_M -> cat. 35 do lam mat vach that tren log.
  static constexpr float TAIL_BEND_DEG = 45.0f;
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
  // Be rong lan hoc = trung vi WIDTH_WINDOW mau gan nhat (doan thang du 2
  // vach, ~25 mau / s), dung khi co >= WIDTH_MIN_SAMPLES mau. Ban truoc 100
  // mau: lan doi be rong (bang keo dan tay) thi ~3 s sau moi theo kip, trong
  // luc do cua chi thay 1 vach tinh tam lech vai cm.
  static constexpr size_t WIDTH_WINDOW = 30;
  static constexpr size_t WIDTH_MIN_SAMPLES = 8;
  // Be rong lan KHONG co dinh: bang keo dan tay, moi doan rong / hep khac
  // nhau. Ghep cap / kiem tra cap chi doi be rong trong khoang tuyet doi nay
  // (khong theo be rong da hoc: hoc 0.36 m o doan hep roi toi doan 0.52 m la
  // loai ca cap dung). lane_width_m chi la gia tri ban dau.
  static constexpr float LANE_W_MIN_M = 0.28f;
  static constexpr float LANE_W_MAX_M = 0.70f;
  // Be rong lan TAI CHO: do moi frame thay du 2 vach (EMA), dung khi chi
  // thay 1 vach (doi tam nua be rong nay). Lau khong thay 2 vach thi tro ve
  // be rong hoc dai han.
  static constexpr float LANE_W_LOCAL_ALPHA = 0.35f;
  static constexpr float LANE_W_LOCAL_HOLD_SEC = 3.0f;
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
  // Chi thay 1 vach: do dai vach con lai (du doan cach vach dang thay
  // OTHER_VIEW_WIDTH x be rong lan) nam trong vung nhin thay (cach mep tam
  // nhin >= OTHER_VIEW_MARGIN_M, Z <= OTHER_VIEW_MAX_Z_M)
  static constexpr float OTHER_VIEW_WIDTH = 1.3f;
  static constexpr float OTHER_VIEW_STEP_M = 0.02f;
  static constexpr float OTHER_VIEW_MARGIN_M = 0.05f;
  static constexpr float OTHER_VIEW_MAX_Z_M = 0.90f;
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

    // Chi khi ONE_LINE: chieu dai (m) vach CON LAI (du doan tu vach dang thay
    // + 1.3 x be rong lan) nam trong vung camera nhin thay. Lon (>= ~0.25 m) =
    // vach do le ra phai thay -> vach mo / bi loa, xe dang o giua; ~0 = vach
    // nam ngoai khung (xe lech ve phia vach dang thay, hoac lan rong hon).
    float other_view_m = 0.0f;

    // Toa do MAT DAT (m, goc = chan camera, X phai, Z truoc), gan -> xa.
    // PathTracker dung de nho duong qua vung mu truoc xe. Rong = khong thay.
    std::vector<cv::Point2f> centre_g; // duong tam lan, lay mau 5 cm
    std::vector<cv::Point2f> left_g;   // vach trai
    std::vector<cv::Point2f> right_g;  // vach phai
    std::chrono::steady_clock::time_point stamp{}; // luc nhan frame

    // True khi frame nay bi chan nhay (dev giu gia tri cu)
    bool gated = false;

    // Phep chieu mat dat -> anh GUI (vis, rong VIS_W): node ve them duong da
    // nho cua PathTracker len anh camera (xem project_vis). proj_f <= 0 = chua co
    float proj_f = 0.0f, proj_cx = 0.0f, proj_cy = 0.0f, proj_h = 0.0f, proj_pitch = 0.0f;

    float horizon_frac = 0.0f; // chan troi (ti le chieu cao anh, < 0 = tren mep anh)
    float pitch_deg = 0.0f;    // goc cui camera dang dung
    bool pitch_confirmed = false;
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

  // Diem mat dat g (m, goc = chan camera, X phai, Z truoc) -> pixel tren anh
  // vis cua frame `o`. false = sau camera / chua co hinh hoc.
  static bool project_vis(const LaneOutput &o, const cv::Point2f &g, cv::Point &p);

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
    float dark = -1.0f;    // do toi trung vi doc vach (EMA), < 0 = chua do
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
  // Be rong (px) doan mask lien tuc cat ngang vach tai g, vuong goc huong d
  float cross_width(const cv::Point2f &g, const cv::Point2f &d) const;
  Poly walk(cv::Mat &work, cv::Point2f p, cv::Point2f d, float max_len,
            bool allow_kink) const;
  Poly trace_from(cv::Mat &work, const cv::Point2f &seed,
                  const cv::Point2f &d) const;
  bool follow(cv::Mat &work, const Poly &pred, float gate, Poly &pts,
              float &err) const;
  std::vector<Poly> candidates(cv::Mat &work, int max_n = 8) const;
  // ref > 0: so voi do toi nay; <= 0: so voi vach dang bam nhat hon
  bool dark_enough(const Poly &pts, float ref = -1.0f) const;
  // Cat doan chan ban / ghe; false = ca vach la chan ban / ghe
  bool trim_upright(Poly &pts) const;
  // Cat vach tai cho bat dau RANG CUA; true neu da cat
  static bool trim_zigzag(Poly &pts);
  float line_dark(const Poly &pts) const;
  // a nam ben TRAI b, cach >= 1/2 be rong lan toi thieu (khong phai cung 1 vach)
  bool left_of(const Poly &a, const Poly &b) const;

  void predict_tracks(float dt);
  void pair_geom(const Poly &a, const Poly &b, float &dist, float &ang,
                 bool &right) const;
  void acquire(const std::vector<Poly> &cands, bool have_left,
               bool have_right, bool allow_single);

  bool straight_head(const Poly &pts, float length, cv::Point2f &a,
                     cv::Point2f &b) const;
  void learn_width(const Poly &left, const Poly &right);
  bool pitch_from_pair(const cv::Point2f ha[2], const cv::Point2f hb[2],
                       double &pitch) const;
  void calibrate_pitch(const std::vector<const Poly *> &lines);
  void adapt_threshold(const cv::Mat &rel, const std::vector<const Poly *> &obs);

  bool target_on(const Poly &path, float L, cv::Point2f &target,
                 float &ext) const;
  bool kink_ahead(const Poly &path, float &dist, int &dir, float &deg,
                  cv::Point2f &pt) const;

  // ---- Hinh hoc mat dat ----
  bool ground_to_img(const cv::Point2f &g, cv::Point2d &p) const;
  static bool to_image(double pitch, double f, double h, double cx, double cy,
                       double X, double Z, double &u, double &v);
  static cv::Point2f to_ground(double pitch, double f, double h, double cx,
                               double cy, double u, double v);
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
  cv::Mat rel_;         // CV_8U, do toi tuong doi cua frame dang xu ly
  cv::Mat clean_ref_;   // mask vach da loc (chua bi an), de do be rong ngang vach
  int n_valid_ = 1;
  std::vector<float> z_near_col_; // Z gan nhat nhin thay theo tung cot

  std::atomic<float> roi_top_frac_{0.0f};
  std::atomic<float> v_mps_{0.0f};
  std::atomic<float> wheel_deg_{0.0f};

  cv::VideoCapture cap_;

  std::atomic<bool> running_{false};
  std::thread worker_;

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
  float lane_w_m_ = 0.42f;     // be rong hoc dai han (trung vi doan thang)
  float lane_w_local_ = 0.42f; // be rong tai cho (frame 2 vach gan nhat)
  float lane_w_local_age_ = 0.0f;
  std::deque<float> width_samples_;
  std::deque<float> pitch_samples_;
  bool pitch_confirmed_ = false;
  double pending_pitch_ = -1.0; // goc cui moi, ap dung o frame sau
  bool pending_reset_ = false;
  float dark_ratio_ = DARK_RATIO_INIT;
  float contrast_ = -1.0f;
  float contrast_age_ = 0.0f; // giay tu lan cuoi do duoc contrast_
  float last_fill_ = 0.0f;
  float speed_est_ = 0.0f;
  float dt_ = 0.04f;

  long frame_id_ = 0;

  std::chrono::steady_clock::time_point last_frame_time_{};
};
