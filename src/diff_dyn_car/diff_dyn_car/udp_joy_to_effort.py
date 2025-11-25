#!/usr/bin/env python3
import socket
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class UdpJoyToEffort(Node):
    def __init__(self):
        super().__init__('udp_joy_to_effort')

        # 参数
        self.declare_parameter('port', 9999)
        self.declare_parameter('tau_forward_max', 0.5)       # 前进最大力矩
        self.declare_parameter('tau_turn_max', 0.3)          # 转向最大力矩
        self.declare_parameter('tau_reverse_max', 0.3)       # 倒车最大力矩

        port = self.get_parameter('port').value
        self.tau_forward_max = self.get_parameter('tau_forward_max').value
        self.tau_turn_max    = self.get_parameter('tau_turn_max').value
        self.tau_reverse_max = self.get_parameter('tau_reverse_max').value

        # 发布器
        self.pub_effort = self.create_publisher(
            Float64MultiArray,
            '/wheel_effort_controller/commands',
            10
        )

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

        self.get_logger().info(f"🎮 UdpJoyToEffort listening on UDP 0.0.0.0:{port}")

        # ===== 校准 =====
        self.calib_needed = True
        self.calib_samples = 0
        self.lx_center = 0.0
        self.lt_center = 0.0
        self.rt_center = 0.0

        self.deadzone_lx = 0.08
        self.deadzone_trigger = 0.05

        self.get_logger().info("✨ Joystick calibration started... 请不要动手柄")

        # 100Hz 轮询
        self.timer = self.create_timer(0.01, self._poll_udp)

    def _poll_udp(self):
        try:
            data, addr = self.sock.recvfrom(1024)
        except BlockingIOError:
            return

        try:
            msg = json.loads(data.decode())
        except Exception:
            return

        # 读取原始值
        lx_raw = -float(msg.get("lx", 0.0))
        lt_raw = float(msg.get("lt", 0.0))
        rt_raw = float(msg.get("rt", 0.0))

        # ========== 校准阶段 ==========
        if self.calib_needed:
            self.lx_center += lx_raw
            self.lt_center += lt_raw
            self.rt_center += rt_raw
            self.calib_samples += 1

            # 累积 100 次（约 1 秒）
            if self.calib_samples >= 100:
                self.lx_center /= self.calib_samples
                self.lt_center /= self.calib_samples
                self.rt_center /= self.calib_samples

                self.calib_needed = False

                self.get_logger().info(
                    f"🎯 Joystick calibration finished:"
                    f" lx_center={self.lx_center:.3f},"
                    f" lt_center={self.lt_center:.3f},"
                    f" rt_center={self.rt_center:.3f}\n"
                )

            return  # 校准时不发输出

        # ========== 减去校准偏移 ==========
        lx = lx_raw - self.lx_center
        lt = lt_raw - self.lt_center
        rt = rt_raw - self.rt_center

        # ========== 死区处理 ==========
        if abs(lx) < self.deadzone_lx:
            lx = 0.0
        if abs(lt) < self.deadzone_trigger:
            lt = 0.0
        if abs(rt) < self.deadzone_trigger:
            rt = 0.0

        # ========== 限幅 ==========
        lx = max(-1.0, min(1.0, lx))
        lt = max(0.0, min(1.0, lt))
        rt = max(0.0, min(1.0, rt))

        # ========== Forza 风格动力分配 ==========
        tau_forward = rt * self.tau_forward_max - lt * self.tau_reverse_max
        tau_turn = lx * self.tau_turn_max

        tau_L = tau_forward - tau_turn
        tau_R = tau_forward + tau_turn

        # 输出力矩
        cmd = Float64MultiArray()
        cmd.data = [tau_L, tau_L, tau_R, tau_R]
        self.pub_effort.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = UdpJoyToEffort()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
