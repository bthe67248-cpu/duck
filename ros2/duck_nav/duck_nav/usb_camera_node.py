"""usb_camera_node -- hardware camera driver for a UVC USB camera (target: Arducam B0385, OV9782 colour global
shutter; alternative: Arducam B0196 IMX219 UVC). UNTESTED ON HARDWARE: written against V4L2 via OpenCV.

Publishes  /camera/image_raw (rgb8), /camera/camera_info (from a calibration YAML), /camera/status (String)
Parameters device        stable path, e.g. /dev/v4l/by-id/usb-Arducam_..-video-index0 (not /dev/video0)
           width/height/fps   requested V4L2 mode (check `v4l2-ctl --list-formats-ext` for real modes)
           fourcc        MJPG or YUYV
           calibration   camera_info YAML from `ros2 run camera_calibration cameracalibrator` (required for
                         SLAM/ToF association; without it CameraInfo is not published and detections stay unranged)
           frame_id      optical frame name (head_cam_optical: z forward, x right, y down -- REP 103)
           exposure      manual exposure in V4L2 units (0 = leave auto); fix it to bound motion blur
           stale_after   s without a frame -> status "stale", reopen the device
Frame convention: this publishes images in a REP-103 optical frame. slam_node expects /camera/pose_level in the
MuJoCo camera convention (looks along -z, y up) -- the hardware pose publisher must convert (rotate 180 deg about x).
Stamps: the capture time is taken when the frame is dequeued (host clock); latency of the USB pipeline is not
compensated -- measure it (e.g. LED + photodiode test) and set `latency` to subtract it.
"""
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def load_camera_info(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    ci = CameraInfo()
    ci.width, ci.height = int(c["image_width"]), int(c["image_height"])
    ci.distortion_model = c.get("distortion_model", "plumb_bob")
    ci.d = [float(v) for v in c["distortion_coefficients"]["data"]]
    ci.k = [float(v) for v in c["camera_matrix"]["data"]]
    ci.r = [float(v) for v in c["rectification_matrix"]["data"]]
    ci.p = [float(v) for v in c["projection_matrix"]["data"]]
    return ci


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__("usb_camera")
        for n, v in (("device", "/dev/video0"), ("width", 640), ("height", 480), ("fps", 30), ("fourcc", "MJPG"),
                     ("calibration", ""), ("frame_id", "head_cam_optical"), ("exposure", 0), ("stale_after", 1.0),
                     ("latency", 0.0)):
            self.declare_parameter(n, v)
        import cv2                                           # imported here so the sim does not need OpenCV
        self.cv2 = cv2
        self.info = None
        cal = self.get_parameter("calibration").value
        if cal:
            self.info = load_camera_info(cal)
        else:
            self.get_logger().warn("no calibration: CameraInfo not published; detections cannot be ranged")
        self.pub = self.create_publisher(Image, "camera/image_raw", 1)
        self.pub_info = self.create_publisher(CameraInfo, "camera/camera_info", 1)
        self.pub_status = self.create_publisher(String, "camera/status", 1)
        self.latest = None; self.lock = threading.Lock(); self.last_frame = 0.0; self.cap = None
        self.running = True; self.reopen = False; self.calibration_mismatch = False
        threading.Thread(target=self.capture_loop, daemon=True).start()
        self.create_timer(0.005, self.publish_latest)
        self.create_timer(0.5, self.watchdog)

    def open(self):
        cv2 = self.cv2
        cap = cv2.VideoCapture(self.get_parameter("device").value, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.get_parameter("fourcc").value))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("width").value))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("height").value))
        cap.set(cv2.CAP_PROP_FPS, int(self.get_parameter("fps").value))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)                  # newest frame, not a backlog
        exp = int(self.get_parameter("exposure").value)
        if exp > 0:
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1); cap.set(cv2.CAP_PROP_EXPOSURE, exp)
        if not cap.isOpened():
            return None
        w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"opened {self.get_parameter('device').value}: {w}x{h} @ {cap.get(cv2.CAP_PROP_FPS):.0f} fps")
        if self.info is not None and (w, h) != (self.info.width, self.info.height):
            self.get_logger().error(f"capture {w}x{h} does not match calibration {self.info.width}x{self.info.height}; images and CameraInfo withheld")
        return cap

    def capture_loop(self):
        while self.running:
            if self.reopen and self.cap is not None:
                self.cap.release(); self.cap = None
            self.reopen = False
            if self.cap is None:
                self.cap = self.open()
                if self.cap is None:
                    time.sleep(1.0); continue
            ok, bgr = self.cap.read()
            t = self.get_clock().now()
            if not ok:
                self.cap.release(); self.cap = None; continue
            with self.lock:
                self.latest = (t, bgr); self.last_frame = time.monotonic()

    def publish_latest(self):
        with self.lock:
            item, self.latest = self.latest, None
        if item is None:
            return
        t, bgr = item
        lat = float(self.get_parameter("latency").value)
        stamp = (t - Duration(seconds=lat)).to_msg() if lat > 0 else t.to_msg()
        rgb = np.ascontiguousarray(bgr[..., ::-1])
        im = Image(); im.header.stamp = stamp; im.header.frame_id = self.get_parameter("frame_id").value
        im.height, im.width = rgb.shape[:2]; im.encoding = "rgb8"; im.step = 3 * rgb.shape[1]; im.data = rgb.tobytes()
        self.calibration_mismatch = self.info is not None and (im.width, im.height) != (self.info.width, self.info.height)
        if self.calibration_mismatch:
            return  # old CameraInfo may be cached downstream: withhold images too
        self.pub.publish(im)
        if self.info is not None:
            self.info.header = im.header; self.pub_info.publish(self.info)

    def watchdog(self):
        stale = time.monotonic() - self.last_frame > float(self.get_parameter("stale_after").value)
        self.pub_status.publish(String(data="stale" if stale else
                                      "calibration_mismatch" if self.calibration_mismatch else "ok"))
        if stale and self.cap is not None and not self.reopen:
            self.get_logger().warn("no frames: reopening camera", throttle_duration_sec=5.0)
            self.reopen = True                               # the capture thread owns the device handle


def main():
    rclpy.init()
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:                                    # noqa: BLE001  torn down by a signal mid-wait
        if rclpy.ok():
            raise
    finally:
        node.running = False; node.destroy_node(); rclpy.try_shutdown()
