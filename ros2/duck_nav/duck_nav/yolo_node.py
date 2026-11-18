"""yolo_node -- YOLOv8 object detector on the head camera (ONNX Runtime, CPU).

Subscribes  /camera/image_raw (rgb8 / bgr8 / mono8, any size; queue depth 1 = newest frame only)
Publishes   /yolo/detections (vision_msgs/Detection2DArray; class names from `class_names`; same stamp as the image)
            /yolo/image (rgb8 with the boxes drawn in), /yolo/stats (String: frames, dropped, inference ms)
Preprocessing: letterbox (uniform scale + centred grey padding) to the model's input size, read from the ONNX
input shape when it is static, otherwise from `imgsz`; boxes are mapped back through the inverse transform.
For the 320x240 sim camera and the shipped 256x320 model this is exactly the old "pad 8 rows" behaviour.
The shipped model was fine-tuned on MuJoCo renders only (scripts/yolo_dataset.py, yolo_train.py); it must be
re-trained / validated on images from the real camera before its detections mean anything on hardware.
"""
import os
import time

import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

PALETTE = [(255, 0, 255), (255, 220, 0), (0, 200, 255), (0, 255, 0)]


def nms(boxes, scores, thr=0.5):
    order = scores.argsort()[::-1]; keep = []
    while len(order):
        i = order[0]; keep.append(i)
        rest = order[1:]
        x0 = np.maximum(boxes[i, 0], boxes[rest, 0]); y0 = np.maximum(boxes[i, 1], boxes[rest, 1])
        x1 = np.minimum(boxes[i, 2], boxes[rest, 2]); y1 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.clip(x1 - x0, 0, None) * np.clip(y1 - y0, 0, None)
        area = lambda b: (b[..., 2] - b[..., 0]) * (b[..., 3] - b[..., 1])
        order = rest[inter / (area(boxes[i]) + area(boxes[rest]) - inter + 1e-9) < thr]
    return keep


def letterbox(rgb, in_h, in_w):
    """Resize with a uniform scale and centre-pad with grey 114. Returns (image, scale, pad_x, pad_y)."""
    h, w = rgb.shape[:2]
    s = min(in_w / w, in_h / h)
    nw, nh = int(round(w * s)), int(round(h * s))
    if (nw, nh) != (w, h):
        yi = np.clip(((np.arange(nh) + 0.5) / s - 0.5).round().astype(int), 0, h - 1)   # nearest-neighbour resize
        xi = np.clip(((np.arange(nw) + 0.5) / s - 0.5).round().astype(int), 0, w - 1)
        rgb = rgb[yi][:, xi]
    px, py = (in_w - nw) // 2, (in_h - nh) // 2
    out = np.full((in_h, in_w, 3), 114, np.uint8)
    out[py:py + nh, px:px + nw] = rgb
    return out, s, px, py


def model_input_size(session, default_hw):
    shape = session.get_inputs()[0].shape                 # [N, 3, H, W]; dynamic dims are strings / None
    if len(shape) == 4 and all(isinstance(v, int) and v > 0 for v in shape[2:]):
        return int(shape[2]), int(shape[3])
    return default_hw


def detect(session, rgb, conf=0.5, in_hw=(256, 320), iou=0.5):
    """rgb (H, W, 3) uint8 -> list of (class index, score, x0, y0, x1, y1) in original image pixels."""
    h, w = rgb.shape[:2]
    x, s, px, py = letterbox(rgb, *in_hw)
    x = x.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    out = session.run(None, {session.get_inputs()[0].name: x})[0][0].T          # (N, 4 + classes)
    scores = out[:, 4:]; cls = scores.argmax(1); sc = scores.max(1)
    k = sc > conf
    if not k.any():
        return []
    b = out[k, :4]; cls, sc = cls[k], sc[k]
    boxes = np.stack([b[:, 0] - b[:, 2] / 2, b[:, 1] - b[:, 3] / 2, b[:, 0] + b[:, 2] / 2, b[:, 1] + b[:, 3] / 2], 1)
    boxes = (boxes - [px, py, px, py]) / s                                       # inverse letterbox
    res = []
    for c in np.unique(cls):
        idx = np.nonzero(cls == c)[0]
        for i in nms(boxes[idx], sc[idx], iou):
            res.append((int(c), float(sc[idx][i]), *np.clip(boxes[idx][i], 0, [w - 1, h - 1, w - 1, h - 1]).tolist()))
    return res


def image_to_rgb(msg):
    """sensor_msgs/Image -> (H, W, 3) uint8 RGB, honouring encoding and row stride."""
    ch = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(msg.encoding)
    if ch is None:
        raise ValueError(f"unsupported encoding {msg.encoding}")
    buf = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    img = buf.reshape(msg.height, msg.width, ch)
    if msg.encoding == "bgr8":
        img = img[..., ::-1]
    elif ch == 1:
        img = np.repeat(img, 3, axis=2)
    return np.ascontiguousarray(img)


def ort_options():
    """ONNX Runtime session options. DUCK_ORT_THREADS=N caps inference at N threads without spin-waiting
    (ORT's default takes every core, which starves the simulator when several trials share a machine;
    detections are unchanged). Unset or 0 keeps ORT's default."""
    so = ort.SessionOptions()
    n = int(os.environ.get("DUCK_ORT_THREADS", "0") or 0)
    if n > 0:
        so.intra_op_num_threads = n; so.inter_op_num_threads = 1
        so.add_session_config_entry("session.intra_op.allow_spinning", "0")
    return so


class YoloNode(Node):
    def __init__(self):
        super().__init__("duck_yolo")
        self.declare_parameter("model", ""); self.declare_parameter("conf", 0.5); self.declare_parameter("iou", 0.5)
        self.declare_parameter("class_names", ["beacon", "box"])
        self.declare_parameter("imgsz", [256, 320])          # [H, W], used only if the ONNX input is dynamic
        self.session = ort.InferenceSession(self.get_parameter("model").value, sess_options=ort_options(),
                                            providers=["CPUExecutionProvider"])
        self.names = list(self.get_parameter("class_names").value)
        self.in_hw = model_input_size(self.session, tuple(int(v) for v in self.get_parameter("imgsz").value))
        nout = self.session.get_outputs()[0].shape[1]
        if isinstance(nout, int) and nout - 4 != len(self.names):
            raise ValueError(f"model has {nout - 4} classes but class_names has {len(self.names)}")
        self.pub = self.create_publisher(Detection2DArray, "yolo/detections", 5)
        self.pub_img = self.create_publisher(Image, "yolo/image", 2)
        self.pub_stats = self.create_publisher(String, "yolo/stats", 2)
        newest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, "camera/image_raw", self.on_image, newest)
        self.counts = {n: 0 for n in self.names}; self.frames = 0; self.ms = 0.0
        self.create_timer(30.0, self.report)
        self.get_logger().info(f"YOLO input {self.in_hw[1]}x{self.in_hw[0]}, classes {self.names}")

    def report(self):
        txt = f"{self.frames} frames, {self.ms:.0f} ms/inference, detections so far: {self.counts}"
        self.get_logger().info(txt); self.pub_stats.publish(String(data=txt))

    def on_image(self, msg):
        try:
            rgb = image_to_rgb(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=10.0); return
        t0 = time.monotonic()
        dets = detect(self.session, rgb, self.get_parameter("conf").value, self.in_hw, self.get_parameter("iou").value)
        self.ms = 0.9 * self.ms + 0.1 * (time.monotonic() - t0) * 1000 if self.frames else (time.monotonic() - t0) * 1000
        self.frames += 1
        arr = Detection2DArray(); arr.header = msg.header
        vis = rgb.copy()
        for c, score, x0, y0, x1, y1 in dets:
            d = Detection2D(); d.header = msg.header
            d.bbox.center.position.x, d.bbox.center.position.y = (x0 + x1) / 2, (y0 + y1) / 2
            d.bbox.size_x, d.bbox.size_y = x1 - x0, y1 - y0
            h = ObjectHypothesisWithPose(); h.hypothesis.class_id = self.names[c]; h.hypothesis.score = score
            d.results.append(h); arr.detections.append(d)
            self.counts[self.names[c]] += 1
            col = PALETTE[c % len(PALETTE)]
            a, b, e, f = int(x0), int(y0), int(x1), int(y1)
            vis[b:f + 1, a:a + 2] = col; vis[b:f + 1, e - 1:e + 1] = col
            vis[b:b + 2, a:e + 1] = col; vis[f - 1:f + 1, a:e + 1] = col
        self.pub.publish(arr)
        out = Image(); out.header = msg.header; out.height, out.width, out.encoding = vis.shape[0], vis.shape[1], "rgb8"
        out.step = 3 * vis.shape[1]; out.data = vis.tobytes(); self.pub_img.publish(out)


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                                    # noqa: BLE001  torn down by a signal mid-wait
        if rclpy.ok():
            raise
    finally:
        node.destroy_node(); rclpy.try_shutdown()
