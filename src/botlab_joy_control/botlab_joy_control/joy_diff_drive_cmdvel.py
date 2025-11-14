#!/usr/bin/env python3
import sys
import os
import time
import pygame

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


"""
wsl不支持直接接收xbox手柄的信号，需要socket连接
这个代码只能在本机是linux系统时才可以使用
"""

class JoyDiffDrive(Node):
    def __init__(self):
        super().__init__('joy_diff_drive')

        # ---- ROS publisher ----
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)

        # ---- control parameters ----
        self.max_wheel_norm = 1.0     # v_left / v_right are in [-1, 1]
        self.k_turn = 1.0             # turn sensitivity for lx

        # map normalized wheel power to real robot speed
        self.max_lin_vel = 0.5        # m/s  (you can tune)
        self.max_ang_vel = 1.5        # rad/s (you can tune)

        # init pygame & joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("❌ No joystick detected. Please connect Xbox controller.")
            sys.exit(1)

        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        print(f"✅ Joystick detected: {self.js.get_name()}")

        os.system("cls" if os.name == "nt" else "clear")

    def print_refresh(self, text: str):
        print(text, end="\r", flush=True)

    def step(self):
        # pump pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                sys.exit(0)

        pygame.event.pump()

        # ----- axes -----
        lx = self.js.get_axis(0)   # left stick X: right is +1
        ly = self.js.get_axis(1)   # left stick Y: down is +1 (not used for now)

        # ----- triggers -----
        lt = self.js.get_axis(4)
        rt = self.js.get_axis(5)
        lt_val = (lt + 1.0) / 2.0   # [0, 1]
        rt_val = (rt + 1.0) / 2.0   # [0, 1]

        # ===========================
        #      power mapping
        # ===========================
        # 1) overall forward/backward power from triggers
        throttle = rt_val - lt_val              # [-1, 1]
        base_speed = throttle * self.max_wheel_norm

        # 2) left-right ratio from lx
        left_raw = 1.0 + self.k_turn * lx
        right_raw = 1.0 - self.k_turn * lx

        max_factor = max(abs(left_raw), abs(right_raw), 1e-6)
        left_factor = left_raw / max_factor
        right_factor = right_raw / max_factor

        # 3) final wheel normalized power
        v_left = base_speed * left_factor
        v_right = base_speed * right_factor

        # ===========================
        #  map to /cmd_vel
        # ===========================
        # average controls forward speed
        # difference controls angular speed
        lin = (v_left + v_right) / 2.0 * self.max_lin_vel
        ang = (v_right - v_left) / 2.0 * self.max_ang_vel

        msg = Twist()
        msg.linear.x = float(lin)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(ang)

        self.pub_cmd.publish(msg)

        # debug print
        self.print_refresh(
            f"LX={lx:+.2f}  LT={lt_val:.2f} RT={rt_val:.2f}  "
            f"|  Left={v_left:+.2f} Right={v_right:+.2f}  "
            f"|  lin={lin:+.2f} ang={ang:+.2f}"
        )


def main():
    rclpy.init()
    node = JoyDiffDrive()

    try:
        rate_hz = 60.0
        dt = 1.0 / rate_hz
        while rclpy.ok():
            node.step()
            # process ROS callbacks (we don't have subscribers, but keep structure)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
