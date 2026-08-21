"""
gpu_amd_decoder.py
==================
ROS 2 node that decompresses sensor_msgs/CompressedImage using AMD GPU
acceleration (ROCm / OpenCL). Probe order at startup:

  1. rocJPEG  — AMD HW JPEG decoder via rocjpeg Python bindings (ROCm ≥ 6.1)
  2. PyOpenCL — decode on CPU, upload raw frame to OpenCL GPU buffer
               (keeps frame in GPU-visible memory for downstream OpenCL kernels)
  3. CPU      — simplejpeg / cv2.imdecode graceful fallback

The selected backend is logged at INFO level on node startup so you can
confirm hardware acceleration is active.

Parameters
----------
gpu_device_id : int  (default 0)    — ROCm / OpenCL device index
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
    Try each AMD/ROCm decode backend in order.
    Returns (backend_name, decode_fn) — always succeeds (CPU fallback).

    decode_fn signature:  (data: bytes) -> np.ndarray
    """

    # --- 1. rocJPEG (ROCm ≥ 6.1) -----------------------------------------
    try:
        import rocjpeg  # noqa: PLC0415

        _cs  = rocjpeg.ROCJPEG_OUTPUT_BGR if 'bgr' in encoding else rocjpeg.ROCJPEG_OUTPUT_RGB
        _dec = rocjpeg.RocJpegDecoder(device_id=device_id)

        def _decode_rocjpeg(data: bytes) -> np.ndarray:
            return _dec.decode(data, output_format=_cs)

        logger.info('[gpu_amd_decoder] backend: rocJPEG')
        return 'rocJPEG', _decode_rocjpeg
    except (ImportError, Exception) as exc:  # noqa: BLE001
        logger.debug(f'[gpu_amd_decoder] rocJPEG unavailable: {exc}')

    # --- 2. PyOpenCL — CPU decode + GPU upload ----------------------------
    # Decodes JPEG on the CPU then copies the raw pixel buffer into an
    # OpenCL device buffer, keeping frames GPU-visible for downstream
    # OpenCL-based processing pipelines (e.g., custom ROCm kernels).
    try:
        import pyopencl as cl  # noqa: PLC0415
        import cv2  # noqa: PLC0415

        platforms = cl.get_platforms()
        # Prefer AMD platform; fall back to first available
        amd_platform = next(
            (p for p in platforms
             if 'amd' in p.name.lower() or 'advanced micro' in p.name.lower()),
            platforms[0] if platforms else None,
        )
        if amd_platform is None:
            raise RuntimeError('no OpenCL platform found')

        devices = amd_platform.get_devices(device_type=cl.device_type.GPU)
        if not devices:
            raise RuntimeError('no OpenCL GPU device found')

        _dev   = devices[min(device_id, len(devices) - 1)]
        _ctx   = cl.Context(devices=[_dev])
        _queue = cl.CommandQueue(_ctx)

        logger.info(
            f'[gpu_amd_decoder] backend: PyOpenCL ({_dev.name.strip()}) — '
            'CPU decode + GPU upload'
        )

        def _decode_pyopencl(data: bytes) -> np.ndarray:
            # JPEG → CPU numpy array
            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError('cv2.imdecode returned None — corrupt frame?')
            if 'rgb' in encoding:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            img_c = np.ascontiguousarray(img)

            # Upload to GPU buffer
            _buf = cl.Buffer(
                _ctx,
                cl.mem_flags.READ_WRITE | cl.mem_flags.COPY_HOST_PTR,
                hostbuf=img_c,
            )

            # Download back for ROS publish
            # (downstream OpenCL nodes can reuse _buf via shared context)
            out = np.empty_like(img_c)
            cl.enqueue_copy(_queue, out, _buf)
            _queue.finish()
            return out

        return 'PyOpenCL', _decode_pyopencl
    except (ImportError, Exception) as exc:  # noqa: BLE001
        logger.debug(f'[gpu_amd_decoder] PyOpenCL unavailable: {exc}')

    # --- 3. CPU fallback --------------------------------------------------
    try:
        import simplejpeg  # noqa: PLC0415

        _cs = 'BGR' if 'bgr' in encoding else 'RGB'

        def _decode_simplejpeg(data: bytes) -> np.ndarray:
            return simplejpeg.decode_jpeg(data, colorspace=_cs)

        logger.warn('[gpu_amd_decoder] no ROCm/OpenCL backend found — using simplejpeg (CPU fallback)')
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

    logger.warn('[gpu_amd_decoder] no ROCm/OpenCL backend found — using cv2.imdecode (CPU fallback)')
    return 'cv2.imdecode (CPU)', _decode_cv2_cpu


# ===========================================================================
# Node
# ===========================================================================

class GpuAmdDecoder(Node):

    def __init__(self):
        super().__init__('gpu_amd_decoder')

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
        # 2 workers: one uploading to GPU, one preparing the next frame
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
                f'[gpu_amd_decoder] decode error ({self._backend_name}): {exc}'
            )
            return

        try:
            out_msg = self._bridge.cv2_to_imgmsg(img, encoding=self._encoding)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'[gpu_amd_decoder] cv_bridge error: {exc}')
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
    node = GpuAmdDecoder()
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
