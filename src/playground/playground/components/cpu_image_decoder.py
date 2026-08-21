"""
cpu_image_decoder.py
====================
ROS 2 node that decompresses sensor_msgs/CompressedImage on a thread pool,
publishing sensor_msgs/Image at full camera rate.

Decode chain (fastest available wins):
  1. simplejpeg  — pure libjpeg-turbo, no OpenCV copy
  2. turbojpeg   — PyTurboJPEG binding to libjpeg-turbo
  3. cv2.imdecode — OpenCV fallback (always available)

Parameters
----------
num_threads   : int  (default 4)  — ThreadPoolExecutor workers
queue_size    : int  (default 5)  — subscriber / publisher queue depth
encoding      : str  (default 'bgr8') — output image encoding

Topics
------
~/in/compressed  [sensor_msgs/CompressedImage]  — input
~/out            [sensor_msgs/Image]             — output
"""

import threading
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


# ---------------------------------------------------------------------------
# Probe decode backends at import time so the per-frame path has no overhead
# ---------------------------------------------------------------------------

def _build_decode_fn():
    """Return the fastest available JPEG/PNG decode callable."""

    # 1 — simplejpeg (libjpeg-turbo, returns numpy directly, zero-copy)
    try:
        import simplejpeg  # noqa: PLC0415

        def _decode_simplejpeg(data: bytes, encoding: str) -> np.ndarray:
            colorspace = 'BGR' if 'bgr' in encoding else 'RGB'
            return simplejpeg.decode_jpeg(data, colorspace=colorspace)

        return 'simplejpeg', _decode_simplejpeg
    except ImportError:
        pass

    # 2 — PyTurboJPEG (libjpeg-turbo via ctypes)
    try:
        from turbojpeg import TurboJPEG, TJPF_BGR, TJPF_RGB  # noqa: PLC0415

        _turbo = TurboJPEG()

        def _decode_turbojpeg(data: bytes, encoding: str) -> np.ndarray:
            pf = TJPF_BGR if 'bgr' in encoding else TJPF_RGB
            return _turbo.decode(data, pixel_format=pf)

        return 'turbojpeg', _decode_turbojpeg
    except (ImportError, Exception):
        pass

    # 3 — OpenCV fallback (always available in ROS 2 environments)
    import cv2  # noqa: PLC0415

    def _decode_cv2(data: bytes, encoding: str) -> np.ndarray:
        arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError('cv2.imdecode returned None — corrupt frame?')
        if 'rgb' in encoding:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    return 'cv2.imdecode', _decode_cv2


_BACKEND_NAME, _DECODE_FN = _build_decode_fn()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class CpuImageDecoder(Node):

    def __init__(self):
        super().__init__('cpu_image_decoder')

        # Parameters
        self.declare_parameter('num_threads',  4)
        self.declare_parameter('queue_size',   5)
        self.declare_parameter('encoding',    'bgr8')

        num_threads = self.get_parameter('num_threads').value
        queue_size  = self.get_parameter('queue_size').value
        self._encoding = self.get_parameter('encoding').value

        self.get_logger().info(
            f'[cpu_image_decoder] decode backend: {_BACKEND_NAME} | '
            f'workers: {num_threads} | queue: {queue_size} | enc: {self._encoding}'
        )

        self._bridge   = CvBridge()
        self._executor = ThreadPoolExecutor(max_workers=num_threads)
        self._lock     = threading.Lock()          # guards publisher (thread-safe)
        self._cb_group = ReentrantCallbackGroup()  # allow concurrent callbacks

        self._pub = self.create_publisher(
            Image, 'out', queue_size
        )

        self._sub = self.create_subscription(
            CompressedImage,
            'in/compressed',
            self._callback,
            queue_size,
            callback_group=self._cb_group,
        )

    # ------------------------------------------------------------------

    def _callback(self, msg: CompressedImage) -> None:
        """Submit decode to thread pool; return immediately to free callback thread."""
        self._executor.submit(self._decode_and_publish, msg)

    def _decode_and_publish(self, msg: CompressedImage) -> None:
        try:
            img = _DECODE_FN(bytes(msg.data), self._encoding)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'[cpu_image_decoder] decode failed: {exc}')
            return

        try:
            out_msg = self._bridge.cv2_to_imgmsg(img, encoding=self._encoding)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'[cpu_image_decoder] cv_bridge failed: {exc}')
            return

        out_msg.header = msg.header  # preserve original timestamp + frame_id

        with self._lock:
            self._pub.publish(out_msg)

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._executor.shutdown(wait=False)
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CpuImageDecoder()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
