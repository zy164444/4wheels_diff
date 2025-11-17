import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class ForwardDemo(Node):
    def __init__(self):
        super().__init__("forward_demo")

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/wheel_effort_controller/commands",
            10
        )

        self.timer = self.create_timer(0.05, self.run)  # 20Hz

    def run(self):
        msg = Float64MultiArray()

        # 力矩大小，先小一点避免车炸飞
        torque = 0.4

        # 顺序要和 YAML 里 joints 顺序一致！
        msg.data = [torque, torque, torque, torque]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardDemo()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
