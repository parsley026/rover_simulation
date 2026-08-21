"""
gpu_nvidia_decoder.py
=====================
ROS 2 node that decompresses sensor_msgs/CompressedImage using NVIDIA GPU
acceleration (CUDA). Probe order at startup:

  1. nvjpeg   — NVIDIA HW JPEG decoder via pynvjpeg
  2. cv2.cuda — OpenCV CUDA imdecode  (requires OpenCV built with CUDA)
  3. CPU      — simplejpeg / cv2.imdecode graceful fallback

The selected backend is logged at INFO level on node startup so you can
confirm hardware acceleration is active.

Parameters
----------
gpu_device_id : int  (default 0)    — CUDA device index
queue_size    : int  (default 5)    — subscriber / publisher queue depth
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


# ===========================================================================
# Backend probe — runs once at import time
# ===========================================================================

def _probe_backends(device_id: int, encoding: str, logger):
    """
    Try each NVIDIA/CUDA decode backend in order.
    Returns (backend_name, decode_fn) — always succeeds (CPU fallback).

    decode_fn signature:  (data: bytes) -> np.ndarray
    """

    # --- 1. nvjpeg via pynvjpeg -------------------------------------------
    try:
        import pynvjpeg  # noqa: PLC0415

        _dec = pynvjpeg.NvJpegDecoder(device_id=device_id)
        _cs  = 'BGR' if 'bgr' in encoding else 'RGB'

        def _decode_nvjpeg(data: bytes) -> np.ndarray:
            return _dec.decode(data, colorspace=_cs)

        logger.info('[gpu_nvidia_decoder] backend: nvjpeg (pynvjpeg)')
        return 'nvjpeg', _decode_nvjpeg
    except (ImportError, Exception) as exc:  # noqa: BLE001
        logger.debug(f'[gpu_nvidia_decoder] nvjpeg unavailable: {exc}')

    # --- 2. cv2.cuda.imdecode ---------------------------------------------
    try:
        import cv2  # noqa: PLC0415

        n_dev = cv2.cuda.getCudaEnabledDeviceCount()
        if n_dev == 0:
            raise RuntimeError('no CUDA-enabled device found by OpenCV')

        cv2.cuda.setDevice(device_id)

        def _decode_cv2_cuda(data: bytes) -> np.ndarray:
            arr     = np.frombuffer(data, dtype=np.uint8)
            gpu_mat = cv2.cuda_GpuMat()
            gpu_mat.upload(arr)
            gpu_img = cv2.cuda.imdecode(gpu_mat, cv2.IMREAD_COLOR)
            img     = gpu_img.download()
            if 'rgb' in encoding:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            return img

        logger.info('[gpu_nvidia_decoder] backend: cv2.cuda.imdecode')
        return 'cv2.cuda', _decode_cv2_cuda
    except (AttributeError, Exception) as exc:  # noqa: BLE001
        logger.debug(f'[gpu_nvidia_decoder] cv2.cuda unavailable: {exc}')

    # --- 3. CPU fallback --------------------------------------------------
    try:
        import simplejpeg  # noqa: PLC0415

        _cs = 'BGR' if 'bgr' in encoding else 'RGB'

        def _decode_simplejpeg(data: bytes) -> np.ndarray:
            return simplejpeg.decode_jpeg(data, colorspace=_cs)

        logger.warn('[gpu_nvidia_decoder] no CUDA backend found — using simplejpeg (CPU fallback)')
        return 'simplejpeg (CPU)', _decode_simplejpeg
    except ImportError:
        pass

    import cv2 as _cv2  # noqa: PLC0415

    def _decode_cv2_cpu(data: bytes) -> np.ndarray:
        arr = np.frombuffer(data, dtype=np.uint8)
        img = _cv2.imdecode(arr, _cv2.IMREAD_COLOR)
        if img is None:
            raise RuntimeError('cv2.imdecode returned None — corrupt frame?')
        if 'rgb' in encoding:
            img = _cv2.cvtColor(img, _cv2.COLOR_BGR2RGB)
        return img

    logger.warn('[gpu_nvidia_decoder] no CUDA backend found — using cv2.imdecode (CPU fallback)')
    return 'cv2.imdecode (CPU)', _decode_cv2_cpu


# ===========================================================================
# Node
# ===========================================================================

class GpuNvidiaDecoder(Node):

    def __init__(self):
        super().__init__('gpu_nvidia_decoder')

        self.declare_parameter('gpu_device_id', 0)
        self.declare_parameter('queue_size',    5)
        self.declare_parameter('encoding',     'bgr8')

        device_id      = self.get_parameter('gpu_device_id').value
        queue_sz       = self.get_parameter('queue_size').value
        self._encoding = self.get_parameter('encoding').value

        self._backend_name, self._decode_fn = _probe_backends(
            device_id, self._encoding, self.get_logger()
        )

        self._bridge   = CvBridge()
        # 2 workers: one decoding on GPU, one handling memcpy / next frame
        self._pool     = ThreadPoolExecutor(max_workers=2)
        self._lock     = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()

        self._pub = self.create_publisher(Image, 'out', queue_sz)
        self._sub = self.create_subscription(
            CompressedImage,
            'in/compressed',
            self._callback,
            queue_sz,
            callback_group=self._cb_group,
        )

    # ------------------------------------------------------------------

    def _callback(self, msg: CompressedImage) -> None:
        self._pool.submit(self._decode_and_publish, msg)

    def _decode_and_publish(self, msg: CompressedImage) -> None:
        try:
            img = self._decode_fn(bytes(msg.data))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'[gpu_nvidia_decoder] decode error ({self._backend_name}): {exc}'
            )
            return

        try:
            out_msg = self._bridge.cv2_to_imgmsg(img, encoding=self._encoding)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'[gpu_nvidia_decoder] cv_bridge error: {exc}')
            return

        out_msg.header = msg.header

        with self._lock:
            self._pub.publish(out_msg)

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._pool.shutdown(wait=False)
        super().destroy_node()


# ===========================================================================
# Entry point
# ===========================================================================

def main(args=None):
    rclpy.init(args=args)
    node = GpuNvidiaDecoder()
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
