#!/usr/bin/env python3
"""
Traffic Light Detector using NCNN + Vulkan acceleration.
Publishes detection decisions to /traffic_light/decision and debug images to /traffic_light/image_debug.
"""

import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
import numpy as np

try:
    import ncnn
except Exception:
    from ncnn_vulkan import ncnn

# Default model paths (can be overridden by ROS parameters)
# Ưu tiên: biến môi trường TRAFFIC_MODEL_DIR -> package share -> fallback cũ
DEFAULT_TRAFFIC_MODEL_DIR = os.environ.get(
    "TRAFFIC_MODEL_DIR",
    os.path.expanduser("~/yolo_ws/runs/detect/traffic_all_red_turn_e10/weights/best_ncnn_model")
)
DEFAULT_MODEL_PARAM = os.path.join(DEFAULT_TRAFFIC_MODEL_DIR, "model.ncnn.param")
DEFAULT_MODEL_BIN = os.path.join(DEFAULT_TRAFFIC_MODEL_DIR, "model.ncnn.bin")

INPUT_BLOB = "in0"
OUTPUT_BLOB = "out0"

# Class mapping from model output
CLASS_NAMES_MAP = {0: "yellow", 1: "green", 2: "red", 5: "stop_sign", 6: "speed_20"}


def letterbox_bgr(img, new_size=512, color=(114, 114, 114)):
    """Resize with unchanged aspect ratio using padding."""
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
    """Convert ncnn Mat to numpy array."""
    a = np.array(mat_out).astype(np.float32)
    w = getattr(mat_out, "w", None)
    h = getattr(mat_out, "h", None)
    c = getattr(mat_out, "c", None)

    if w is not None and h is not None and a.size == h * w:
        return a.reshape(h, w)
    if c is not None and w is not None and h is not None and a.size == c * h * w:
        return a.reshape(c, h, w)
    return a.reshape(-1)


def iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    iw = max(0.0, inter_x2 - inter_x1)
    ih = max(0.0, inter_y2 - inter_y1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter + 1e-9
    return inter / union


def nms_xyxy(boxes, scores, iou_thres=0.45, max_det=30, classes=None):
    """Greedy NMS.

    `classes` bắt buộc nếu muốn giữ nhiều lớp: NMS toàn cục sẽ lo mất hẳn
    một biển nằm trùng vùng với một đèn (ví dụ cổng 20 nằm ngay cạnh đèn đỏ),
    vì chỉ còn một hộp có score cao nhất được giữ lại.
    """
    if len(boxes) == 0:
        return []

    keep = []
    # Nhóm chỉ số theo lớp rồi NMS riêng từng lớp => đèn và biển cùng vùng
    # vẫn cùng tồn tại.
    if classes is None:
        groups = [np.arange(len(boxes), dtype=np.int64)]
    else:
        groups = [np.where(classes == c)[0].astype(np.int64) for c in np.unique(classes)]

    for group in groups:
        if group.size == 0:
            continue
        idxs = group[np.argsort(scores[group])[::-1]]
        while idxs.size > 0 and len(keep) < max_det:
            i = int(idxs[0])
            keep.append(i)
            if idxs.size == 1:
                break
            rest = idxs[1:]
            ious = [iou_xyxy(boxes[i], boxes[int(j)]) for j in rest]
            idxs = rest[np.array(ious, dtype=np.float32) <= iou_thres]
    return keep


class TrafficLightDetectorNCNNVulkan(Node):
    def __init__(self):
        super().__init__("traffic_light_detector_ncnn_vulkan")
        self.model_ready = False

        self.bridge = CvBridge()

        # Topic là tham số: trước đây viết cứng trong code nên không override
        # được từ launch, và node C++ chỉ subscribe đúng tên mặc định.
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("debug_image_topic", "/traffic_light/image_debug")
        self.declare_parameter("decision_topic", "/traffic_light/decision")

        topic_input = self.get_parameter("image_topic").value
        topic_debug = self.get_parameter("debug_image_topic").value
        topic_decision = self.get_parameter("decision_topic").value

        self.sub = self.create_subscription(Image, topic_input, self.image_callback, 10)
        self.pub_img = self.create_publisher(Image, topic_debug, 10)
        self.pub_state = self.create_publisher(String, topic_decision, 10)

        # Parameters
        self.declare_parameter("model_param", DEFAULT_MODEL_PARAM)
        self.declare_parameter("model_bin", DEFAULT_MODEL_BIN)
        self.declare_parameter("infer_hz", 30.0)
        self.declare_parameter("imgsz", 512)

        self.declare_parameter("conf_min", 0.3)
        self.declare_parameter("iou_thres", 0.45)
        self.declare_parameter("max_det", 1)
        self.declare_parameter("min_box_area_ratio", 0.02)

        self.declare_parameter("CONF_RED", 0.55)
        self.declare_parameter("CONF_YELLOW", 0.55)
        self.declare_parameter("CONF_GREEN", 0.45)
        self.declare_parameter("CONF_STOP", 0.45)
        self.declare_parameter("CONF_SPEED", 0.35)

        self.declare_parameter("RED_LOCK_TIME", 2.0)
        self.declare_parameter("YELLOW_LOCK_TIME", 1.2)
        self.declare_parameter("STOP_LOCK_TIME", 1.5)
        self.declare_parameter("SPEED_LOCK_TIME", 1.2)

        # Load parameters
        self.model_param = self.get_parameter("model_param").value
        self.model_bin = self.get_parameter("model_bin").value
        self.infer_hz = float(self.get_parameter("infer_hz").value)
        self.imgsz = int(self.get_parameter("imgsz").value)

        self.conf_min = float(self.get_parameter("conf_min").value)
        self.iou_thres = float(self.get_parameter("iou_thres").value)
        self.max_det = int(self.get_parameter("max_det").value)
        self.min_box_area_ratio = float(self.get_parameter("min_box_area_ratio").value)

        self.CONF_RED = float(self.get_parameter("CONF_RED").value)
        self.CONF_YELLOW = float(self.get_parameter("CONF_YELLOW").value)
        self.CONF_GREEN = float(self.get_parameter("CONF_GREEN").value)
        self.CONF_STOP = float(self.get_parameter("CONF_STOP").value)
        self.CONF_SPEED = float(self.get_parameter("CONF_SPEED").value)

        self.RED_LOCK_TIME = float(self.get_parameter("RED_LOCK_TIME").value)
        self.YELLOW_LOCK_TIME = float(self.get_parameter("YELLOW_LOCK_TIME").value)
        self.STOP_LOCK_TIME = float(self.get_parameter("STOP_LOCK_TIME").value)
        self.SPEED_LOCK_TIME = float(self.get_parameter("SPEED_LOCK_TIME").value)

        self.last_red_time = -999.0
        self.last_yellow_time = -999.0
        self.last_stop_time = -999.0
        self.last_speed_time = -999.0
        self.red_frame_count = 0
        self.last_decision = "NONE"

        # Chỉ phát String khi quyết định đổi, nhưng vẫn phát lại định kỳ để
        # watchdog `ai_timeout_s` của fusion_viz_node không xoá nhầm.
        self._last_published_decision = None
        self._last_decision_pub_ns = 0
        self.DECISION_HEARTBEAT_NS = 500_000_000   # 0.5s, nhỏ hơn ai_timeout_s (2s)

        self.last_frame = None
        # Định danh frame: tăng mỗi khi có ảnh mới, dùng để bỏ qua việc infer
        # lại cùng một frame.
        self.frame_seq = 0
        self.last_inferred_seq = -1
        self.logged_once = False
        self._warned = set()

        # Initialize NCNN
        self._init_ncnn()

        self.get_logger().info("TrafficLightDetector NCNN Vulkan STARTED (iGPU/Vulkan)")
        self.get_logger().info(f"  model_param = {self.model_param}")
        self.get_logger().info(f"  infer_hz = {self.infer_hz}")

        self.timer = self.create_timer(1.0 / max(1.0, self.infer_hz), self.process_latest)

    def _init_ncnn(self):
        """Initialize NCNN network with error handling."""
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = True
        self.net.opt.num_threads = 4

        # Validate model files exist
        if not os.path.exists(self.model_param):
            raise FileNotFoundError(f"Model param file not found: {self.model_param}")
        if not os.path.exists(self.model_bin):
            raise FileNotFoundError(f"Model bin file not found: {self.model_bin}")

        # load_param/load_model return int (0 = success)
        rc_param = self.net.load_param(self.model_param)
        rc_model = self.net.load_model(self.model_bin)
        if rc_param != 0 or rc_model != 0:
            self.get_logger().error(
                f"Failed to load NCNN model (param rc={rc_param}, bin rc={rc_model}).\n"
                f"  model_param: {self.model_param}\n"
                f"  model_bin  : {self.model_bin}\n"
                f"Pass correct paths via ROS parameters: model_param:=... model_bin:=..."
            )
            raise RuntimeError(f"NCNN model load failed: {self.model_param}")

        self.model_ready = True

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        # Đánh dấu định danh frame: process_latest chỉ chạy trên frame MỚI.
        # Nếu không, timer 30 Hz sẽ infer lại cùng một frame nhiều lần và
        # đẩy red_frame_count lên ngưỡng chỉ vì một lần nhìn thấy đèn đỏ.
        self.frame_seq += 1
        self.last_frame = frame

    def decode_box_auto(self, b4):
        b4 = b4.astype(np.float32).copy()
        if np.max(np.abs(b4)) <= 2.0:
            b4 *= float(self.imgsz)

        x1, y1, x2, y2 = b4
        if (x2 > x1) and (y2 > y1):
            return x1, y1, x2, y2

        cx, cy, bw, bh = b4
        return cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2

    def cls_name(self, idx: int) -> str:
        return CLASS_NAMES_MAP.get(idx, f"cls{idx}")

    def _warn_once(self, key: str, msg: str):
        """Log một lần cho mỗi loại chuyển nhãn để không spam mỗi frame."""
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(msg)

    def verify_red_color(self, crop):
        """Check if bbox contains red color (traffic light)."""
        if crop is None or crop.size == 0:
            return False

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        lower_yellow = np.array([15, 50, 50])
        upper_yellow = np.array([35, 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)
        yellow_pixels = cv2.countNonZero(mask_yellow)
        total_pixels = crop.shape[0] * crop.shape[1]

        red_ratio = red_pixels / total_pixels

        return red_ratio > 0.05 and red_pixels > max(green_pixels, yellow_pixels)

    def verify_yellow_color(self, crop):
        """Check if bbox contains yellow color."""
        if crop is None or crop.size == 0:
            return False

        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([35, 255, 255])

        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])

        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        yellow_pixels = cv2.countNonZero(mask_yellow)
        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)
        total_pixels = crop.shape[0] * crop.shape[1]

        yellow_ratio = yellow_pixels / total_pixels

        return yellow_ratio > 0.05 and yellow_pixels > max(red_pixels, green_pixels)

    def process_latest(self):
        if not self.model_ready:
            self.get_logger().warn("Model not ready, skipping inference")
            return

        if self.last_frame is None:
            return

        # Chỉ infer frame mới. Nếu không, timer 30 Hz sẽ xử lý lại cùng một
        # frame nhiều lần: red_frame_count tăng theo số lần timer chạy chứ
        # không theo số lần thực sự nhìn thấy đèn đỏ.
        if self.frame_seq == self.last_inferred_seq:
            return
        self.last_inferred_seq = self.frame_seq

        frame = self.last_frame
        h, w, _ = frame.shape
        roi_y1 = int(0.15 * h)
        roi_y2 = int(0.85 * h)
        roi_x1 = int(0.20 * w)
        roi_x2 = int(0.90 * w)
        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        rh, rw = roi.shape[:2]
        roi_area = float(rh * rw)

        if rw < 2 or rh < 2:
            return

        img_lb_bgr, r, padw, padh = letterbox_bgr(roi, new_size=self.imgsz)
        img_lb_rgb = cv2.cvtColor(img_lb_bgr, cv2.COLOR_BGR2RGB)

        mat_in = ncnn.Mat.from_pixels(img_lb_rgb, ncnn.Mat.PixelType.PIXEL_RGB, self.imgsz, self.imgsz)
        mat_in.substract_mean_normalize([], [1/255.0, 1/255.0, 1/255.0])

        ex = self.net.create_extractor()
        ex.input(INPUT_BLOB, mat_in)
        ret, mat_out = ex.extract(OUTPUT_BLOB)
        if ret != 0 or mat_out is None:
            return

        mat = ncnn_mat_to_np(mat_out)

        if isinstance(mat, np.ndarray) and mat.ndim == 2:
            out = mat
            if out.shape[0] <= 64 and out.shape[1] > out.shape[0]:
                out = out.T
        else:
            flat = np.array(mat).reshape(-1)
            return

        if not self.logged_once:
            self.logged_once = True
            self.get_logger().info(
                f"out0 w={getattr(mat_out,'w',None)} h={getattr(mat_out,'h',None)} c={getattr(mat_out,'c',None)} -> out={out.shape}"
            )
            if out.size:
                self.get_logger().info(f"row0={out[0].tolist()}")

        if out.shape[1] < 5:
            return

        D = out.shape[1]
        nc = D - 4
        if nc <= 0:
            return

        boxes4 = out[:, 0:4]
        probs = out[:, 4:4+nc]

        # Apply sigmoid only if logits (values outside [0,1])
        if probs.size and (probs.max() > 1.0 or probs.min() < -1e-4):
            probs = sigmoid(probs)

        cls = np.argmax(probs, axis=1)
        score = probs[np.arange(probs.shape[0]), cls]

        keep = score >= self.conf_min
        if not np.any(keep):
            self.publish(frame, "NONE")
            return

        cand = np.where(keep)[0]

        boxes = []
        scores = []
        clses = []

        for i in cand[:2000]:
            x1, y1, x2, y2 = self.decode_box_auto(boxes4[i])

            x1 = (x1 - padw) / r
            x2 = (x2 - padw) / r
            y1 = (y1 - padh) / r
            y2 = (y2 - padh) / r

            x1 = float(max(0.0, min(rw - 1.0, x1)))
            x2 = float(max(0.0, min(rw - 1.0, x2)))
            y1 = float(max(0.0, min(rh - 1.0, y1)))
            y2 = float(max(0.0, min(rh - 1.0, y2)))
            if x2 <= x1 or y2 <= y1:
                continue

            area = (x2 - x1) * (y2 - y1)
            if area < self.min_box_area_ratio * roi_area:
                continue

            boxes.append([x1, y1, x2, y2])
            scores.append(float(score[i]))
            clses.append(int(cls[i]))

        if not boxes:
            self.publish(frame, "NONE")
            return

        boxes = np.array(boxes, dtype=np.float32)
        scores = np.array(scores, dtype=np.float32)
        clses = np.array(clses, dtype=np.int32)

        # NMS theo từng lớp: đèn và biển 20 có thể cùng nằm trong một vùng.
        keep_ids = nms_xyxy(boxes, scores, iou_thres=self.iou_thres,
    max_det=self.max_det, classes=clses)

        now = time.time()
        detected_red = detected_yellow = detected_green = detected_stop = detected_speed = False

        for k in keep_ids:
            x1, y1, x2, y2 = boxes[k]
            conf = float(scores[k])
            c = int(clses[k])
            label = self.cls_name(c)

            crop = roi[int(y1):int(y2), int(x1):int(x2)]

            # Kiểm tra HSV chỉ dùng để NÂNG mức an toàn (nhẹ -> nặng), không dùng để hạ.
            # Hạ "red" thành "yellow" là lỗi nguy hiểm: bản cũ làm vậy, và chỉ
            # cần một frame sương mù/vàng chói là xe chạy thẳng qua đèn đỏ.
            if label == "green":
                if self.verify_red_color(crop):
                    self._warn_once("green->red", "HSV check: green -> RED")
                    label = "red"
                elif self.verify_yellow_color(crop):
                    self._warn_once("green->yellow", "HSV check: green -> YELLOW")
                    label = "yellow"

            elif label == "yellow":
                if self.verify_red_color(crop):
                    self._warn_once("yellow->red", "HSV check: yellow -> RED")
                    label = "red"

            # red giữ nguyên: không có bước xác nhận nào có thể hạ đèn đỏ.

            if label == "red" and conf >= self.CONF_RED:
                detected_red = True
                self.last_red_time = now
                self.red_frame_count += 1
            elif label == "yellow" and conf >= self.CONF_YELLOW:
                detected_yellow = True
                self.last_yellow_time = now
            elif label == "green" and conf >= self.CONF_GREEN:
                detected_green = True
            elif label == "stop_sign" and conf >= self.CONF_STOP:
                detected_stop = True
                self.last_stop_time = now
            elif label == "speed_20" and conf >= self.CONF_SPEED:
                detected_speed = True
                self.last_speed_time = now

            fx1 = int(x1 + roi_x1); fx2 = int(x2 + roi_x1)
            fy1 = int(y1 + roi_y1); fy2 = int(y2 + roi_y1)
            cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (fx1, max(0, fy1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if not detected_red:
            self.red_frame_count = max(0, self.red_frame_count - 1)

        # Priority: STOP > RED > YELLOW > 20 > GREEN > NONE
        # GREEN only when actually detected (not default)
        if detected_stop or (now - self.last_stop_time < self.STOP_LOCK_TIME):
            decision = "STOP"
        elif detected_red or self.red_frame_count >= 2 or (now - self.last_red_time < self.RED_LOCK_TIME):
            decision = "RED"
        elif detected_yellow or (now - self.last_yellow_time < self.YELLOW_LOCK_TIME):
            decision = "YELLOW"
        elif detected_speed or (now - self.last_speed_time < self.SPEED_LOCK_TIME):
            decision = "20"
        elif detected_green:
            decision = "GREEN"
        else:
            decision = "NONE"

        self.last_decision = decision

        cv2.putText(frame, f"DECISION: {decision}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3)

        self.publish(frame, decision)

    def publish(self, frame, decision):
        """Publish debug image always; publish decision String only on change.

        Gửi String mỗi vòng infer (30 Hz) là 30 topic message/s cho một giá trị
        vốn đổi vài lần mỗi phút. Nhưng KHÔNG được im luôn: fusion_viz_node có
        watchdog `ai_timeout_s`, tức quyết định cũ sẽ bị xoá về "NONE" sau
        vài giây im lặng. Vì vậy luôn phát lại theo nhịp heartbeat, chỉ bỏ qua
        ở nhịp timer khi không có thay đổi.
        """
        try:
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            now_ns = self.get_clock().now().nanoseconds
            changed = decision != self._last_published_decision
            heartbeat_due = (now_ns - self._last_decision_pub_ns) >= self.DECISION_HEARTBEAT_NS

            if changed or heartbeat_due:
                self.pub_state.publish(String(data=decision))
                self._last_published_decision = decision
                self._last_decision_pub_ns = now_ns
        except Exception as e:
            self.get_logger().error(f"Failed to publish: {e}")


def main():
    rclpy.init()
    try:
        node = TrafficLightDetectorNCNNVulkan()
    except (FileNotFoundError, RuntimeError) as e:
        print(f"[traffic_light_detector] ERROR: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()