#!/usr/bin/env python3
import socket
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JoyServer(Node):
    def __init__(self):
        super().__init__("joy_server")

        # 发布四个轮子的速度
        self.pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_velocity_controller/commands",
            10
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 9999))
        self.sock.settimeout(0.0)

        self.get_logger().info("🎮 Joy server started.")

        self.timer = self.create_timer(0.01, self.loop)

    def loop(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            msg = json.loads(data.decode())

            lx = msg["lx"]      # 转向
            lt = msg["lt"]      # 后退扳机
            rt = msg["rt"]      # 前进扳机

            throttle = rt - lt          # [-1, +1]
            left_speed  = 10 * throttle * (1 + lx)
            right_speed = 10 * throttle * (1 - lx)

            # 准备消息
            arr = Float64MultiArray()
            arr.data = [
                left_speed,
                right_speed,
                left_speed,
                right_speed
            ]
            print(f"L={left_speed:+.2f}  R={right_speed:+.2f}", end="\r")

            self.pub.publish(arr)

        except Exception:
            pass


def main():
    rclpy.init()
    rclpy.spin(JoyServer())


if __name__ == "__main__":
    main()
