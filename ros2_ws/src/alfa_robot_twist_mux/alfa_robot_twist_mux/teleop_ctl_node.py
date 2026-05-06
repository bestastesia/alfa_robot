import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty


class HoldKeyTeleop(Node):
    def __init__(self):
        super().__init__('hold_key_teleop')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel_teleop',
            10
        )

        # 当前按键状态
        self.key = None

        # 参数
        self.linear = 0.3
        self.angular = 0.8

        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)

        # 10Hz持续发布
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info(
            "Hold-key teleop started\n"
            "W/S: forward/back\n"
            "A/D: rotate\n"
            "release key = stop"
        )

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def update(self):
        # 每次循环更新键状态
        k = self.get_key()

        if k is not None:
            self.key = k
        else:
            # ❗关键：没有按键 → 自动归零
            self.key = None

        twist = Twist()

        # 根据当前按键生成速度
        if self.key == 'w':
            twist.linear.x = self.linear
        elif self.key == 's':
            twist.linear.x = -self.linear
        elif self.key == 'a':
            twist.angular.z = self.angular
        elif self.key == 'd':
            twist.angular.z = -self.angular
        else:
            # 松手 / 无输入 → stop
            twist = Twist()

        self.pub.publish(twist)

    def destroy(self):
        self.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rclpy.init()
    node = HoldKeyTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()
