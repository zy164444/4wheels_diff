import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class DiffEffortController(Node):
    def __init__(self):
        super().__init__('diff_effort_controller')

        # 车的参数（和 URDF 一致）
        self.L = 0.11   # 轮距
        self.R = 0.035  # 轮子半径

        # 发布到 effort 控制器
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_effort_controller/commands',
            10
        )

        # 订阅整体速度指令
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_cb,
            10
        )

    def cmd_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # 差速模型：车的线速度、角速度 → 左右轮线速度
        v_r = v + 0.5 * w * self.L
        v_l = v - 0.5 * w * self.L

        # 很粗暴的“力矩控制”：力矩 ∝ 期望线速度
        k_tau = 0.5
        tau_r = k_tau * v_r
        tau_l = k_tau * v_l

        cmd = Float64MultiArray()
        # 顺序对应 yaml 里的 joints 列表
        cmd.data = [tau_l, tau_l, tau_r, tau_r]

        self.effort_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DiffEffortController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
