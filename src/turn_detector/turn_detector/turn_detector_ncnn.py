#!/usr/bin/env python3
"""
Turn Direction Detector using NCNN + Vulkan acceleration.
Publishes detection decisions to /turn_detector/decision and debug images to /turn_detector/image_debug.
"""

import os
import cv2
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    import ncnn  # type: ignore
except Exception:
    from ncnn_vulkan import ncnn  # type: ignore

# Default model paths (can be overridden by ROS parameters)
DEFAULT_MODEL_PARAM = os.path.expanduser("~/models/turn_lr_ncnn_model/model.ncnn.param")
DEFAULT_MODEL_BIN = os.path.expanduser("~/models/turn_lr_ncnn_model/model.ncnn.bin")

INPUT_BLOB = "in0"
OUTPUT_BLOB = "out0"

# Unique topics for this node (avoid collision with traffic_light_detector)
TOPIC_IMAGE = "/turn_detector/image_debug"
TOPIC_DECISION = "/turn_detector/decision"
TOPIC_INPUT = "/image_raw"


def letterbox_bgr(img, new_size=320, color=(114, 114, 114)):
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


class TurnDetectorNCNNVulkan(Node):
    def __init__(self):
        super().__init__("turn_detector_ncnn_vulkan")
        self.model_ready = False

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, TOPIC_INPUT, self.image_callback, 10)
        self.pub_decision = self.create_publisher(String, TOPIC_DECISION, 10)
        self.pub_img = self.create_publisher(Image, TOPIC_IMAGE, 10)
        self.model_param = self.declare_parameter("model_param", DEFAULT_MODEL_PARAM).value
        self.model_bin = self.declare_parameter("model_bin", DEFAULT_MODEL_BIN).value

        self.history = deque(maxlen=5)
        self.last_frame = None

        # Parameters
        self.declare_parameter("infer_hz", 30.0)
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("conf_keep", 0.05)
        self.declare_parameter("min_box_area_ratio", 0.02)
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

        # Initialize NCNN
        self._init_ncnn()

        self.logged_once = False
        self.last_log_ns = 0

        self.get_logger().info("TurnDetector NCNN Vulkan STARTED (iGPU/Vulkan)")
        self.get_logger().info(f"  model_param = {self.model_param}")
        self.get_logger().info(f"  decision topic = {TOPIC_DECISION}")
        self.timer = self.create_timer(1.0 / self.infer_hz, self.process_latest)

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
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def detect_arrow_direction(self, crop):
        """Detect arrow direction using contour analysis."""
        if crop is None or crop.size == 0:
            return None

        ch, cw = crop.shape[:2]
        if ch < 10 or cw < 10:
            return None

        # Gamma correction for dark screens
        gamma = 1.5
        inv_gamma = 1.0 / gamma
        table = np.array([(i / 255.0) ** inv_gamma * 255 for i in np.arange(0, 256)]).astype("uint8")
        brightened = cv2.LUT(crop, table)

        hsv = cv2.cvtColor(brightened, cv2.COLOR_BGR2HSV)

        # White color threshold
        lower_white = np.array([0, 0, 130])
        upper_white = np.array([180, 100, 255])
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

        # Direction mapping (calibrated for actual camera)
        return "left" if dx < 0 else "right"

    def decode_box_auto(self, b4):
        b4 = b4.astype(np.float32).copy()

        if np.max(np.abs(b4)) <= 2.0:
            b4 *= float(self.imgsz)

        x1, y1, x2, y2 = b4
        if (x2 > x1) and (y2 > y1):
            return x1, y1, x2, y2

        cx, cy, bw, bh = b4
        return cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2

    def publish_result(self, frame, decision):
        """Publish decision and debug image."""
        if decision == "TURN_LEFT":
            cv2.putText(frame, "TURN LEFT", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
        elif decision == "TURN_RIGHT":
            cv2.putText(frame, "TURN RIGHT", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)

        self.pub_decision.publish(String(data=decision))
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_log_ns > 1_000_000_000:
            self.last_log_ns = now_ns
            l = self.history.count("left")
            r = self.history.count("right")
            self.get_logger().info(f"{decision=} L={l} R={r}")

    def process_latest(self):
        if not self.model_ready:
            self.get_logger().warn("Model not ready, skipping inference")
            return

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

        if rw < 2 or rh < 2:
            return

        img_lb_bgr, r, padw, padh = letterbox_bgr(roi, new_size=self.imgsz)
        img_lb_rgb = cv2.cvtColor(img_lb_bgr, cv2.COLOR_BGR2RGB)

        mat_in = ncnn.Mat.from_pixels(
            img_lb_rgb, ncnn.Mat.PixelType.PIXEL_RGB, self.imgsz, self.imgsz
        )
        mat_in.substract_mean_normalize([], [1 / 255.0, 1 / 255.0, 1 / 255.0])

        ex = self.net.create_extractor()
        ex.input(INPUT_BLOB, mat_in)

        ret, mat_out = ex.extract(OUTPUT_BLOB)
        if ret != 0 or mat_out is None:
            return

        mat = ncnn_mat_to_np(mat_out)

        # Normalize output to (N, 6)
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

        if not self.logged_once:
            self.logged_once = True
            self.get_logger().info(
                f"out0 mat w={getattr(mat_out,'w',None)} h={getattr(mat_out,'h',None)} "
                f"c={getattr(mat_out,'c',None)} -> out shape={out.shape}, "
                f"min={out.min():.3f}, max={out.max():.3f}"
            )
            self.get_logger().info(f"out0 row0={out[0].tolist()}")

        boxes4 = out[:, 0:4]
        probs = out[:, 4:6]

        # Apply sigmoid if logits
        if probs.max() > 1.0 or probs.min() < 0.0:
            probs = sigmoid(probs)

        cls = np.argmax(probs, axis=1)
        score = probs[np.arange(probs.shape[0]), cls]

        # Filter by confidence
        cand = np.where(score >= self.conf_keep)[0]
        if cand.size == 0:
            self.history.append("none")
            self.publish_result(frame, "NONE")
            return

        # Select best candidate by score * sqrt(area)
        best_i = None
        best_metric = -1.0
        best_box = None
        best_cls = 0
        best_score = 0.0

        for i in cand[:300]:
            x1, y1, x2, y2 = self.decode_box_auto(boxes4[i])

            # Undo letterbox
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

        # Convert to full-frame coordinates
        x1 = int(x1 + roi_x1); x2 = int(x2 + roi_x1)
        y1 = int(y1 + roi_y1); y2 = int(y2 + roi_y1)

        # Shrink bbox
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

        # Direction from model class (with optional inversion)
        if self.invert_class:
            best_cls = 1 - best_cls
        direction_model = "left" if best_cls == 0 else "right"

        crop = frame[y1:y2, x1:x2]
        direction_contour = self.detect_arrow_direction(crop)
        direction = direction_contour if direction_contour else direction_model

        self.history.append(direction if direction else "none")

        # Voting: require clear majority (>=3 votes with >=2 margin)
        l = self.history.count("left")
        r = self.history.count("right")

        if l >= 3 and l - r >= 2:
            decision = "TURN_LEFT"
        elif r >= 3 and r - l >= 2:
            decision = "TURN_RIGHT"
        else:
            decision = "NONE"

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"arrow: {direction} ({best_score:.2f})",
                    (x1, max(0, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.publish_result(frame, decision)


def main():
    rclpy.init()
    try:
        node = TurnDetectorNCNNVulkan()
    except (FileNotFoundError, RuntimeError) as e:
        print(f"[turn_detector] ERROR: {e}")
        rclpy.shutdown()
        raise SystemExit(1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()