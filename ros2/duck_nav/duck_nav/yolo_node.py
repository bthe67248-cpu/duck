"""yolo_node -- YOLOv8-nano object detector on the head camera (ONNX Runtime, CPU).

Subscribes  /camera/image_raw (rgb8 320x240)
Publishes   /yolo/detections (vision_msgs/Detection2DArray; class_id "beacon" or "box"; same stamp as the image)
            /yolo/image (rgb8 with the boxes drawn in: magenta = beacon, yellow = box)
The model was fine-tuned on MuJoCo renders of this kind of room (scripts/yolo_dataset.py, yolo_train.py).
"""
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

NAMES = ["beacon", "box"]
COLORS = [(255, 0, 255), (255, 220, 0)]
PAD = 8                                         # 240 rows -> 256 (a multiple of 32), 8 px top and bottom


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


def detect(session, rgb, conf=0.5):
    """rgb (240, 320, 3) uint8 -> list of (class index, score, x0, y0, x1, y1) in image pixels."""
    x = np.full((256, 320, 3), 114, np.uint8); x[PAD:PAD + 240] = rgb
    x = x.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    out = session.run(None, {session.get_inputs()[0].name: x})[0][0].T          # (N, 4 + classes)
    scores = out[:, 4:]; cls = scores.argmax(1); sc = scores.max(1)
    k = sc > conf
    if not k.any():
        return []
    b = out[k, :4]; cls, sc = cls[k], sc[k]
    boxes = np.stack([b[:, 0] - b[:, 2] / 2, b[:, 1] - b[:, 3] / 2 - PAD, b[:, 0] + b[:, 2] / 2, b[:, 1] + b[:, 3] / 2 - PAD], 1)
    res = []
    for c in np.unique(cls):
        idx = np.nonzero(cls == c)[0]
        for i in nms(boxes[idx], sc[idx]):
            res.append((int(c), float(sc[idx][i]), *np.clip(boxes[idx][i], 0, [319, 239, 319, 239]).tolist()))
    return res


class YoloNode(Node):
    def __init__(self):
        super().__init__("duck_yolo")
        self.declare_parameter("model", ""); self.declare_parameter("conf", 0.5)
        self.session = ort.InferenceSession(self.get_parameter("model").value, providers=["CPUExecutionProvider"])
        self.pub = self.create_publisher(Detection2DArray, "yolo/detections", 5)
        self.pub_img = self.create_publisher(Image, "yolo/image", 2)
        self.create_subscription(Image, "camera/image_raw", self.on_image, 2)
        self.counts = {n: 0 for n in NAMES}; self.frames = 0
        self.create_timer(30.0, lambda: self.get_logger().info(f"{self.frames} frames, detections so far: {self.counts}"))

    def on_image(self, msg):
        rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        dets = detect(self.session, rgb, self.get_parameter("conf").value)
        self.frames += 1
        arr = Detection2DArray(); arr.header = msg.header
        vis = rgb.copy()
        for c, score, x0, y0, x1, y1 in dets:
            d = Detection2D(); d.header = msg.header
            d.bbox.center.position.x, d.bbox.center.position.y = (x0 + x1) / 2, (y0 + y1) / 2
            d.bbox.size_x, d.bbox.size_y = x1 - x0, y1 - y0
            h = ObjectHypothesisWithPose(); h.hypothesis.class_id = NAMES[c]; h.hypothesis.score = score
            d.results.append(h); arr.detections.append(d)
            self.counts[NAMES[c]] += 1
            a, b, e, f = int(x0), int(y0), int(x1), int(y1)
            vis[b:f + 1, a:a + 2] = COLORS[c]; vis[b:f + 1, e - 1:e + 1] = COLORS[c]
            vis[b:b + 2, a:e + 1] = COLORS[c]; vis[f - 1:f + 1, a:e + 1] = COLORS[c]
        self.pub.publish(arr)
        out = Image(); out.header = msg.header; out.height, out.width, out.encoding, out.step = msg.height, msg.width, "rgb8", msg.step
        out.data = vis.tobytes(); self.pub_img.publish(out)


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
