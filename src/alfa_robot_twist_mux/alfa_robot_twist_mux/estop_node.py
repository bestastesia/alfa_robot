import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class EstopNode(Node):
    def __init__(self):
        super().__init__('estop_node')

        self.estop_active = False

        # 输出给 mux
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel_estop',
            10
        )

        # 订阅急停信号
        self.sub = self.create_subscription(
            Bool,
            '/estop',
            self.estop_callback,
            10
        )
        
        self.stoppub = self.create_publisher(
            Bool,
            '/stop',
            10
        )

        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info("E-Stop node started")

    def estop_callback(self, msg):
        self.estop_active = msg.data

    def update(self):
        if self.estop_active:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.pub.publish(msg)
            
            stop_msg = Bool()
            stop_msg.data = True
            self.stoppub.publish(stop_msg)
        stop_msg = Bool()
        stop_msg.data = self.estop_active
        self.stoppub.publish(stop_msg)
            

def main():
    rclpy.init()
    node = EstopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
