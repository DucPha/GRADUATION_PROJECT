#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np
from collections import deque

try:
    import ncnn  # type: ignore
except Exception:
    from ncnn_vulkan import ncnn  # type: ignore


MODEL_PARAM = "/home/pi/models/turn_lr_ncnn_model/model.ncnn.param"
MODEL_BIN   = "/home/pi/models/turn_lr_ncnn_model/model.ncnn.bin"
INPUT_BLOB  = "in0"
OUTPUT_BLOB = "out0"  # cat_18 -> out0 [file:173]


def letterbox_bgr(img, new_size=320, color=(114, 114, 114)):
    h, w = img.shape[:2]
    r = min(new_size / w, new_size / h)
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((new_size, new_size, 3), color, dtype=np.uint8)
    pad_w = (new_size - nw) // 2
    pad_h = (new_size - nh) // 2
    canvas[pad_h:pad_h + nh, pad_w:pad_w + nw] = resized
    return canvas, r, pad_w, pad_h


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def ncnn_mat_to_np(mat_out):
    a = np.array(mat_out).astype(np.float32)
    w = getattr(mat_out, "w", None)
    h = getattr(mat_out, "h", None)
    c = getattr(mat_out, "c", None)

    # thÆ°á»ng out0 ra dáº¡ng (h, w) vá»›i h=6 w=2100
    if w is not None and h is not None and a.size == h * w:
        return a.reshape(h, w)

    if c is not None and w is not None and h is not None and a.size == c * h * w:
        return a.reshape(c, h, w)

    return a.reshape(-1)


class TurnDetectorNCNNVulkan(Node):
    def __init__(self):
        super().__init__("turn_detector_ncnn_vulkan")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, "/image_raw", self.image_callback, 10)
        self.pub_decision = self.create_publisher(String, "/turn_detector/decision", 10)
        self.pub_img = self.create_publisher(Image, "/traffic_light/image_debug", 10)
        self.model_param = self.declare_parameter("model_param", MODEL_PARAM).value
        self.model_bin = self.declare_parameter("model_bin", MODEL_BIN).value

        self.history = deque(maxlen=5)
        self.last_frame = None

        # Params
        self.declare_parameter("infer_hz", 30.0)
        self.declare_parameter("imgsz", 320)

        # threshold class prob (vÃ¬ out0 lÃ  bbox + 2 class sigmoid theo graph) [file:173]
        self.declare_parameter("conf_keep", 0.05)

        # filter bbox quÃ¡ nhá» (trÃ¡nh bbox â€œtÃ­ honâ€)
        self.declare_parameter("min_box_area_ratio", 0.02)  # theo ROI area

        self.declare_parameter("bbox_shrink", 0.12)
        self.declare_parameter("deadzone_ratio", 0.12)
        self.declare_parameter("min_area_ratio", 0.02)

        self.declare_parameter("invert_class", False)

        self.infer_hz = float(self.get_parameter("infer_hz").value)
        if self.infer_hz < 1.0:
            self.infer_hz = 1.0

        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf_keep = float(self.get_parameter("conf_keep").value)
        self.min_box_area_ratio = float(self.get_parameter("min_box_area_ratio").value)

        self.bbox_shrink = float(self.get_parameter("bbox_shrink").value)
        self.deadzone_ratio = float(self.get_parameter("deadzone_ratio").value)
        self.min_area_ratio = float(self.get_parameter("min_area_ratio").value)

        self.invert_class = bool(self.get_parameter("invert_class").value)

        # --- NCNN Vulkan ---
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = True
        self.net.opt.num_threads = 4

        self.net.load_param(self.model_param)
        self.net.load_model(self.model_bin)

        self.logged_once = False
        self.last_log_ns = 0

        self.get_logger().info("âœ… TurnDetector NCNN Vulkan STARTED (iGPU/Vulkan)")
        self.timer = self.create_timer(1.0 / self.infer_hz, self.process_latest)

    def image_callback(self, msg: Image):
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def detect_arrow_direction(self, crop):
        if crop is None or crop.size == 0:
            return None

        ch, cw = crop.shape[:2]
        if ch < 10 or cw < 10:
            return None

        # ✅ GAMMA CORRECTION: Tăng độ sáng cho màn hình tối
        gamma = 1.5  # > 1 = sáng hơn, < 1 = tối hơn
        inv_gamma = 1.0 / gamma
        table = np.array([(i / 255.0) ** inv_gamma * 255 
                        for i in np.arange(0, 256)]).astype("uint8")
        brightened = cv2.LUT(crop, table)
        
        hsv = cv2.cvtColor(brightened, cv2.COLOR_BGR2HSV)
        
        # Giảm threshold
        lower_white = np.array([0, 0, 130])  # ← GIẢM THÊM
        upper_white = np.array([180, 100, 255])  # ← NỚI RỘNG Saturation
        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < self.min_area_ratio * (cw * ch):
            return None

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])

        farthest_point = None
        max_dist = 0
        for p in cnt:
            x, y = p[0]
            dist = (x - cx) ** 2
            if dist > max_dist:
                max_dist = dist
                farthest_point = (x, y)

        if farthest_point is None:
            return None

        fx, _ = farthest_point
        dx = fx - cx

        dead = self.deadzone_ratio * cw
        if abs(dx) < dead:
            return None

        # báº¡n Ä‘Ã£ Ä‘áº£o cho phÃ¹ há»£p camera thá»±c táº¿
        return "left" if dx < 0 else "right"

    def decode_box_auto(self, b4):
        b4 = b4.astype(np.float32).copy()

        # normalized?
        if np.max(np.abs(b4)) <= 2.0:
            b4 *= float(self.imgsz)

        x1, y1, x2, y2 = b4
        if (x2 > x1) and (y2 > y1):
            return x1, y1, x2, y2

        cx, cy, bw, bh = b4
        return cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2

    """ def publish_result(self, frame, decision):
        cv2.putText(frame, f"TURN: {decision}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        self.pub_decision.publish(String(data=decision))
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns > 1_000_000_000:
            self.last_log_ns = now_ns
            l = self.history.count("left")
            r = self.history.count("right")
            self.get_logger().info(f"decision={decision}  L={l} R={r}") """
    def publish_result(self, frame, decision):
        # Hiá»‡n text vá»›i mÃ u khÃ¡c nhau
        if decision == "TURN_LEFT":
            cv2.putText(frame, "TURN LEFT", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
        elif decision == "TURN_RIGHT":
            cv2.putText(frame, "TURN RIGHT", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
        # KhÃ´ng hiá»‡n gÃ¬ khi NONE
        
        self.pub_decision.publish(String(data=decision))
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns > 1_000_000_000:
            self.last_log_ns = now_ns
            l = self.history.count("left")
            r = self.history.count("right")
            self.get_logger().info(f"{decision=} L={l} R={r}")

    def process_latest(self):
        if self.last_frame is None:
            return

        frame = self.last_frame
        H, W, _ = frame.shape

        roi_y1 = int(0.05 * H)
        roi_y2 = int(0.90 * H)
        roi_x1 = int(0.05 * W)
        roi_x2 = int(0.95 * W)
        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        rh, rw = roi.shape[:2]
        roi_area = float(rh * rw)

        img_lb_bgr, r, padw, padh = letterbox_bgr(roi, new_size=self.imgsz)

        # IMPORTANT: Ultralytics preprocess cÃ³ bÆ°á»›c BGR->RGB, nÃªn custom ncnn cÅ©ng pháº£i Ä‘Æ°a RGB vÃ o [web:227][web:231]
        img_lb_rgb = cv2.cvtColor(img_lb_bgr, cv2.COLOR_BGR2RGB)

        mat_in = ncnn.Mat.from_pixels(img_lb_rgb, ncnn.Mat.PixelType.PIXEL_RGB, self.imgsz, self.imgsz)
        mat_in.substract_mean_normalize([], [1/255.0, 1/255.0, 1/255.0])

        ex = self.net.create_extractor()
        ex.input(INPUT_BLOB, mat_in)

        ret, mat_out = ex.extract(OUTPUT_BLOB)
        if ret != 0 or mat_out is None:
            return

        mat = ncnn_mat_to_np(mat_out)

        # chuáº©n hoÃ¡ out vá» (N,6)
        if isinstance(mat, np.ndarray) and mat.ndim == 2:
            if mat.shape[1] == 6:
                out = mat
            elif mat.shape[0] == 6:
                out = mat.T
            else:
                flat = mat.reshape(-1)
                if flat.size % 6 != 0:
                    return
                out = flat.reshape(-1, 6)
        else:
            flat = np.array(mat).reshape(-1)
            if flat.size % 6 != 0:
                return
            out = flat.reshape(-1, 6)

        # Debug 1 láº§n
        if not self.logged_once:
            self.logged_once = True
            self.get_logger().info(
                f"out0 mat w={getattr(mat_out,'w',None)} h={getattr(mat_out,'h',None)} c={getattr(mat_out,'c',None)} "
                f"-> out shape={out.shape}, min={out.min():.3f}, max={out.max():.3f}"
            )
            self.get_logger().info(f"out0 row0={out[0].tolist()}")

        boxes4 = out[:, 0:4]
        probs = out[:, 4:6]

        # náº¿u probs Ä‘ang lÃ  logits thÃ¬ sigmoid láº¡i (Ä‘á»ƒ cháº¯c cháº¯n) [file:173]
        if probs.max() > 1.0 or probs.min() < 0.0:
            probs = sigmoid(probs)

        cls = np.argmax(probs, axis=1)
        score = probs[np.arange(probs.shape[0]), cls]

        # chá»n candidate theo conf_keep
        cand = np.where(score >= self.conf_keep)[0]
        if cand.size == 0:
            self.history.append("none")
            self.publish_result(frame, "NONE")
            return

        # decode bbox cho candidates + lá»c bbox quÃ¡ nhá» + chá»n best theo score*sqrt(area)
        best_i = None
        best_metric = -1.0
        best_box = None
        best_cls = 0
        best_score = 0.0

        for i in cand[:300]:  # giá»›i háº¡n Ä‘á»ƒ nháº¹
            x1, y1, x2, y2 = self.decode_box_auto(boxes4[i])

            # undo letterbox -> ROI
            x1 = (x1 - padw) / r
            x2 = (x2 - padw) / r
            y1 = (y1 - padh) / r
            y2 = (y2 - padh) / r

            x1 = float(max(0.0, min(rw - 1.0, x1)))
            x2 = float(max(0.0, min(rw * 1.0, x2)))
            y1 = float(max(0.0, min(rh - 1.0, y1)))
            y2 = float(max(0.0, min(rh * 1.0, y2)))
            if x2 <= x1 or y2 <= y1:
                continue

            area = (x2 - x1) * (y2 - y1)
            if area < self.min_box_area_ratio * roi_area:
                continue

            metric = float(score[i]) * np.sqrt(area)
            if metric > best_metric:
                best_metric = metric
                best_i = int(i)
                best_box = (x1, y1, x2, y2)
                best_cls = int(cls[i])
                best_score = float(score[i])

        if best_i is None or best_box is None:
            self.history.append("none")
            self.publish_result(frame, "NONE")
            return

        x1, y1, x2, y2 = best_box

        # vá» full-frame coords
        x1 = int(x1 + roi_x1); x2 = int(x2 + roi_x1)
        y1 = int(y1 + roi_y1); y2 = int(y2 + roi_y1)

        # siáº¿t bbox + clamp
        padx = int(self.bbox_shrink * (x2 - x1))
        pady = int(self.bbox_shrink * (y2 - y1))
        x1 += padx; x2 -= padx
        y1 += pady; y2 -= pady

        x1 = max(0, min(x1, W - 1))
        x2 = max(0, min(x2, W))
        y1 = max(0, min(y1, H - 1))
        y2 = max(0, min(y2, H))
        if x2 <= x1 or y2 <= y1:
            self.history.append("none")
            self.publish_result(frame, "NONE")
            return

        # direction by model class (fallback)
        if self.invert_class:
            best_cls = 1 - best_cls
        direction_model = "left" if best_cls == 0 else "right"

        crop = frame[y1:y2, x1:x2]
        direction_contour = self.detect_arrow_direction(crop)
        direction = direction_contour if direction_contour else direction_model

        self.history.append(direction if direction else "none")
        l = self.history.count("left")
        r = self.history.count("right")

        if l >= 3:
            decision = "TURN_LEFT"
        elif r >= 3:
            decision = "TURN_RIGHT"
        else:
            decision = "NONE"
        # debug
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"arrow: {direction} ({best_score:.2f})",
                    (x1, max(0, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.publish_result(frame, decision)


def main():
    rclpy.init()
    node = TurnDetectorNCNNVulkan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()