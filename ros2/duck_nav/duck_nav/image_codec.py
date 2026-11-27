"""image_codec -- decode sensor_msgs/Image buffers (numpy only; shared by slam_node's optical flow and yolo_node).

ROS images carry `step`, the length of one row in bytes, which may be larger than width x channels (padded rows,
common from V4L2 drivers and some bridges). Decoding must take the first width x channels bytes of each row, not
reshape the whole buffer, or a padded image either fails to reshape or is silently misaligned.
"""
import numpy as np

CHANNELS = {"rgb8": 3, "bgr8": 3, "mono8": 1}


class ImageDecodeError(ValueError):
    """The message cannot be decoded: unsupported encoding or an inconsistent buffer."""


def decode(msg):
    """sensor_msgs/Image -> (H, W, C) uint8 array in the message's own channel order (C = 3 or 1).
    Raises ImageDecodeError for an unsupported encoding, an endianness the 8-bit formats do not have, a step shorter
    than a row, or a buffer shorter than height x step."""
    ch = CHANNELS.get(msg.encoding)
    if ch is None:
        raise ImageDecodeError(f"unsupported encoding {msg.encoding!r} (supported: {', '.join(CHANNELS)})")
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    if h <= 0 or w <= 0:
        raise ImageDecodeError(f"empty image {w}x{h}")
    if step < w * ch:
        raise ImageDecodeError(f"step {step} is shorter than one row ({w} x {ch} bytes)")
    data = np.frombuffer(bytes(msg.data) if not isinstance(msg.data, (bytes, bytearray, memoryview)) else msg.data,
                         np.uint8)
    if data.size < h * step:
        raise ImageDecodeError(f"buffer has {data.size} bytes, needs height x step = {h * step}")
    rows = data[:h * step].reshape(h, step)[:, :w * ch]
    return np.ascontiguousarray(rows.reshape(h, w, ch))


def to_rgb(msg):
    """(H, W, 3) uint8 RGB."""
    img = decode(msg)
    if msg.encoding == "bgr8":
        return np.ascontiguousarray(img[..., ::-1])
    if img.shape[2] == 1:
        return np.repeat(img, 3, axis=2)
    return img


def to_gray(msg, cv2=None):
    """(H, W) uint8 grey. With OpenCV, colour conversion uses cv2.cvtColor (the same weights as before);
    without it, the ITU-R BT.601 weights OpenCV uses, rounded."""
    img = decode(msg)
    if img.shape[2] == 1:
        return img[:, :, 0]
    if cv2 is not None:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY if msg.encoding == "bgr8" else cv2.COLOR_RGB2GRAY)
    rgb = img[..., ::-1] if msg.encoding == "bgr8" else img
    g = rgb[..., 0] * 0.299 + rgb[..., 1] * 0.587 + rgb[..., 2] * 0.114
    return np.clip(np.rint(g), 0, 255).astype(np.uint8)
