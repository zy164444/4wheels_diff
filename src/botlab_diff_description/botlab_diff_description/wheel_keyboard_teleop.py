#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WheelKeyboardTeleop(Node):
    """
    键盘输入四个数 -> 持续发布到 /wheel_velocity_controller/commands

    joints 顺序必须和 YAML 一致：
      0: left_front_wheel_joint
      1: right_front_wheel_joint
      2: left_behind_wheel_joint
      3: right_behind_wheel_joint
    """

    def __init__(self):
        super().__init__('wheel_keyboard_teleop')

        # 发布频率参数（Hz），默认 10，相当于 ros2 topic pub -r 10
        self.declare_parameter('publish_rate', 10.0)
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )

        # 当前四个轮子的目标速度，初始为 0
        self.current_cmd = [0.0, 0.0, 0.0, 0.0]

        # 定时器：按固定频率发布当前命令
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        # 开一个单独线程读键盘，不阻塞 rclpy.spin()
        self.input_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info(
            f'WheelKeyboardTeleop started. publish_rate = {rate} Hz\n'
            '输入格式: 四个浮点数，用空格分隔，例如:\n'
            '  5 5 5 5      -> 四个轮子一起转\n'
            '  5 -5 5 -5    -> 左前/左后正转，右前/右后反转\n'
            '输入 q 回车退出（或 Ctrl+C）。'
        )

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = list(self.current_cmd)
        self.publisher.publish(msg)

    def keyboard_loop(self):
        """
        在单独线程里循环读键盘输入。
        """
        while rclpy.ok():
            try:
                line = input("\n请输入四个轮子速度 [lf rf lb rb]，或 q 回车退出：\n> ").strip()
            except EOFError:
                # 终端被关了之类的，直接退出循环
                break

            if not line:
                continue

            if line.lower() in ('q', 'quit', 'exit'):
                self.get_logger().info("收到退出指令，当前节点将停止发布。按 Ctrl+C 结束进程。")
                # 置零，让小车停下
                self.current_cmd = [0.0, 0.0, 0.0, 0.0]
                break

            parts = line.split()
            if len(parts) != 4:
                print("❌ 需要输入 4 个数，例如: 2 2 2 2")
                continue

            try:
                vals = [float(p) for p in parts]
            except ValueError:
                print("❌ 解析失败，请输入数字，例如: 1.0 -1.0 0 0")
                continue

            self.current_cmd = vals
            self.get_logger().info(
                f"更新四轮速度: lf={vals[0]:.3f}, rf={vals[1]:.3f}, "
                f"lb={vals[2]:.3f}, rb={vals[3]:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WheelKeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停车再退出
        node.current_cmd = [0.0, 0.0, 0.0, 0.0]
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
