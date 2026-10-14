# docs/tools — công cụ mô phỏng & kiểm chứng

Các script Python dùng để **thiết kế và kiểm chứng** bộ điều khiển bám làn
trước khi nạp lên xe (không cần `colcon build`).

## `mpc_sim.py` — mô phỏng vòng kín MPC + LQI

Bản port trung thành của `src/camera_node/lane_keeping.hpp::lk_mpc_solve()`
(cùng ma trận A/B, hàm chi phí `qy·e_y² + qp·e_psi² + r·u² + rd·Δu²`) ghép với
mô hình xe (động học theo `L`, `g`, `b` + servo bậc 1 `TAU`).

Mục đích: chứng minh **"xe lệch dần"** là **sai số xác lập (steady-state
offset)** do lệch mô hình (`b`, `g`, deadband servo, camera lệch góc) — MPC
thuần túy **không** khử được — và **thành phần tích phân LQI** (`q_i`, `i_max`,
`s0`) khử được.

```powershell
python docs/tools/mpc_sim.py
```

Các kịch bản: S1 mô hình đúng, S2 lệch `b`, S3 `b` được oracle học đúng,
S4 thêm deadband servo, S5 `g` sai (0.42 vs 0.70).

## `mpc_sim2.py` — tinh chỉnh `q_i` cho bền vững

Vòng 2: quét `q_i`/`i_max` với nhiễu cảm biến, góc cua (bước `rho`), khôi phục
từ lệch lớn — dùng để chọn `q_i = 60`, `i_max = 0.06`.

```powershell
python docs/tools/mpc_sim2.py
```

**Kết quả chính** (`q_i = 60`): lệch tĩnh ~0 cm trong mọi trường hợp lệch mô
hình, hội tụ < 2 s, vẫn ổn định khi có nhiễu và vào cua. `q_i = 0` giữ nguyên
hành vi cũ (lệch ~+4 cm khi `b = 3°`). Chi tiết tuning: `docs/TUNING.md` §8.

> Yêu cầu: `numpy`. Chạy từ thư mục `docs/tools/` (do `mpc_sim2.py` import
> `mpc_sim`).
