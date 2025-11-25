#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray
import csv
import time
import math


class CarStateLogger(Node):
    def __init__(self):
        super().__init__('car_state_logger')

        self.car_name = 'diff_dyn_car'

        # 创建 CSV 文件
        timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self.filename = f"car_log_{timestamp}.csv"
        self.file = open(self.filename, "w", newline='')
        self.writer = csv.writer(self.file)

        # 写表头（加上左右轮 effort）
        self.writer.writerow([
            "t",
            "x", "y", "z",
            "yaw",
            "vx", "vy", "vz",
            "wx", "wy", "wz",
            "tau_L", "tau_R"
        ])

        # 订阅 model_states
        self.sub_states = self.create_subscription(
            ModelStates,
            '/model_states',
            self.cb_states,
            10
        )

        # 订阅轮子 effort 命令
        self.sub_effort = self.create_subscription(
            Float64MultiArray,
            '/wheel_effort_controller/commands',
            self.cb_effort,
            10
        )

        # 保存最近一次的左右轮力矩
        self.last_tau_L = 0.0
        self.last_tau_R = 0.0

        self.start_time = self.get_clock().now()

        self.get_logger().info(f"🚗 CarStateLogger started, saving to {self.filename}")

    def quat_to_yaw(self, qx, qy, qz, qw):
        # ZYX 欧拉角，只取 yaw
        siny = 2.0 * (qw*qz + qx*qy)
        cosy = 1.0 - 2.0 * (qy*qy + qz*qz)
        return math.atan2(siny, cosy)

    def cb_effort(self, msg: Float64MultiArray):
        # 你的控制节点发的是 [tau_L, tau_L, tau_R, tau_R]
        if len(msg.data) >= 4:
            self.last_tau_L = float(msg.data[0])
            self.last_tau_R = float(msg.data[2])
        elif len(msg.data) == 2:
            # 如果以后改成只发 [tau_L, tau_R] 也兼容
            self.last_tau_L = float(msg.data[0])
            self.last_tau_R = float(msg.data[1])
        # else: 长度奇怪就先忽略，不更新

    def cb_states(self, msg: ModelStates):
        if self.car_name not in msg.name:
            return

        idx = msg.name.index(self.car_name)

        pose = msg.pose[idx]
        twist = msg.twist[idx]

        # 位置
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        # 姿态 yaw
        yaw = self.quat_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # 线速度
        vx = twist.linear.x
        vy = twist.linear.y
        vz = twist.linear.z

        # 角速度
        wx = twist.angular.x
        wy = twist.angular.y
        wz = twist.angular.z

        # 时间戳（秒）
        now = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # 当前左右轮 effort（由 cb_effort 维护）
        tau_L = self.last_tau_L
        tau_R = self.last_tau_R

        # 写入 CSV
        self.writer.writerow([
            now,
            x, y, z,
            yaw,
            vx, vy, vz,
            wx, wy, wz,
            tau_L, tau_R
        ])

    def destroy_node(self):
        # 关节点之前把文件关掉
        try:
            self.file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarStateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
