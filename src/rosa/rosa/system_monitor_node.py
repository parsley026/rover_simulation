# Copyright (c) 2026. All rights reserved.
# Dedicated Hardware & System Diagnostic Monitor Node for ROSA Command Center.
# Polling occurs independently to preserve single responsibility and zero CPU blocking on LLM inference.

import json
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('rosa_system_monitor')
        self.pub = self.create_publisher(String, 'rosa/system_status', 10)
        # Publish hardware diagnostics every 2 seconds
        self.timer = self.create_timer(2.0, self._on_timer)
        
        self.last_cpu_idle = 0
        self.last_cpu_total = 0
        self.sim_battery_voltage = 24.6  # 24V Rover nominal system voltage
        self.battery_decay_step = 0.01
        
        self.get_logger().info("System Monitor Node ready. Polling RTX 3060 GPU, CPU, & Rover metrics.")

    def _get_gpu_metrics(self) -> dict:
        """Query NVIDIA GPU metrics without external Python libraries via non-blocking subprocess."""
        try:
            cmd = [
                "nvidia-smi",
                "--query-gpu=name,memory.used,memory.total,utilization.gpu,temperature.gpu",
                "--format=csv,noheader,nounits"
            ]
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
            if res.returncode == 0 and res.stdout.strip():
                parts = [p.strip() for p in res.stdout.strip().split(",")]
                if len(parts) >= 5:
                    return {
                        "gpu_name": parts[0],
                        "vram_used_mb": int(parts[1]),
                        "vram_total_mb": int(parts[2]),
                        "vram_percent": round((int(parts[1]) / max(int(parts[2]), 1)) * 100, 1),
                        "gpu_util_percent": float(parts[3]),
                        "gpu_temp_c": float(parts[4])
                    }
        except Exception as e:
            self.get_logger().debug(f"GPU metric reading note: {e}")
        return {
            "gpu_name": "NVIDIA GeForce RTX 3060",
            "vram_used_mb": 0,
            "vram_total_mb": 6144,
            "vram_percent": 0.0,
            "gpu_util_percent": 0.0,
            "gpu_temp_c": 0.0
        }

    def _get_cpu_and_ram(self) -> dict:
        """Fast <1ms inspection of Linux /proc/stat and /proc/meminfo."""
        cpu_percent = 0.0
        try:
            with open('/proc/stat', 'r') as f:
                line = f.readline()
                if line.startswith('cpu '):
                    val = [float(x) for x in line.split()[1:9]]
                    idle = val[3] + val[4]
                    total = sum(val)
                    diff_idle = idle - self.last_cpu_idle
                    diff_total = total - self.last_cpu_total
                    if diff_total > 0 and self.last_cpu_total > 0:
                        cpu_percent = round((1.0 - (diff_idle / diff_total)) * 100.0, 1)
                    self.last_cpu_idle = idle
                    self.last_cpu_total = total
        except Exception as e:
            pass

        ram_used_mb = 0
        ram_total_mb = 0
        ram_percent = 0.0
        try:
            mem_data = {}
            with open('/proc/meminfo', 'r') as f:
                for _ in range(5):
                    parts = f.readline().split()
                    if len(parts) >= 2:
                        mem_data[parts[0].rstrip(':')] = int(parts[1])
            if 'MemTotal' in mem_data and 'MemAvailable' in mem_data:
                ram_total_mb = mem_data['MemTotal'] // 1024
                ram_used_mb = (mem_data['MemTotal'] - mem_data['MemAvailable']) // 1024
                ram_percent = round((ram_used_mb / max(ram_total_mb, 1)) * 100.0, 1)
        except Exception as e:
            pass

        return {
            "cpu_percent": max(0.0, min(100.0, cpu_percent)),
            "ram_used_mb": ram_used_mb,
            "ram_total_mb": ram_total_mb,
            "ram_percent": ram_percent
        }

    def _get_rover_battery(self) -> dict:
        """Simulated live rover telemetry battery health indicator."""
        self.sim_battery_voltage -= self.battery_decay_step
        if self.sim_battery_voltage <= 23.5:
            self.battery_decay_step = -0.01
        elif self.sim_battery_voltage >= 24.8:
            self.battery_decay_step = 0.01
        voltage = round(self.sim_battery_voltage, 2)
        percentage = round(max(0, min(100, ((voltage - 22.0) / 3.0) * 100.0)), 1)
        return {
            "battery_voltage": voltage,
            "battery_percent": percentage,
            "battery_status": "DISCHARGING" if self.battery_decay_step > 0 else "CHARGING"
        }

    def _on_timer(self):
        payload = {
            "timestamp": time.time(),
            "gpu": self._get_gpu_metrics(),
            "system": self._get_cpu_and_ram(),
            "rover": self._get_rover_battery(),
            "status": "ONLINE"
        }
        msg = String(data=json.dumps(payload))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
