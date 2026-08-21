# Components sub-package: standalone ROS 2 decoder nodes for
# hardware-accelerated image decompression.
#
# Available nodes:
#   cpu_image_decoder   — ThreadPoolExecutor + libjpeg-turbo (simplejpeg/turbojpeg/cv2)
#   gpu_nvidia_decoder  — NVIDIA/CUDA:  nvjpeg → cv2.cuda → CPU fallback
#   gpu_amd_decoder     — AMD/ROCm:    rocJPEG → PyOpenCL  → CPU fallback
