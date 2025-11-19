import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time


class WheelOdomMonitor(Node):
    def __init__(self):
        super().__init__('wheel_odom_monitor')

        # 你车的参数
        self.R = 0.035  # 轮半径 [m]
        self.L = 0.11   # 轮距 [m]

        # 关节名字（按 joint_states 里的 name 顺序来）
        self.left_joints = ['left_behind_wheel_joint', 'left_front_wheel_joint']
        self.right_joints = ['right_behind_wheel_joint', 'right_front_wheel_joint']

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_cb,
            10
        )

        self.last_print_time = time.time()

    def joint_cb(self, msg: JointState):
        # 把 name -> velocity 做个字典
        vel_map = {name: vel for name, vel in zip(msg.name, msg.velocity)}

        # 左右轮角速度（取平均）
        w_ls = [vel_map.get(j, 0.0) for j in self.left_joints]
        w_rs = [vel_map.get(j, 0.0) for j in self.right_joints]

        if len(w_ls) == 0 or len(w_rs) == 0:
            return

        w_L = sum(w_ls) / len(w_ls)
        w_R = sum(w_rs) / len(w_rs)

        # 线速度
        v_L = w_L * self.R
        v_R = w_R * self.R

        # 车体线速度 / 角速度
        v = 0.5 * (v_L + v_R)
        omega = (v_R - v_L) / self.L  # 绕 z 轴角速度

        # 每 0.1 s 打印一次
        now = time.time()
        if now - self.last_print_time > 0.1:
            self.last_print_time = now
            self.get_logger().info(
                f"v = {v:.3f} m/s, omega = {omega:.3f} rad/s "
                f"(w_L = {w_L:.2f}, w_R = {w_R:.2f} rad/s)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
