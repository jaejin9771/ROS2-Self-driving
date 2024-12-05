import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        # 초기화
        self.twist_publisher_ = None

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'start_car',
            self.car_info_listener_callback,
            10)

        self.vel_info_subscription = self.create_subscription(
            Twist,
            'vel_info',
            self.vel_info_listener_callback,
            10
        )

    def car_info_listener_callback(self, msg: String):
        # 지정 차량
        car = msg.data
        self.get_logger().info('I heard: "%s"' % msg.data)

        time.sleep(5)

        # 차량에 속도 정보를 전달할 publisher
        self.twist_publisher_ = self.create_publisher(Twist, f'/{car}/cmd_demo', 10)

        # 차량 출발
        twist = Twist()
        for i in range(200):
            twist.linear.x = 2.0
            self.twist_publisher_.publish(twist)
            time.sleep(0.01)

    def vel_info_listener_callback(self, twist: Twist):
        self.twist_publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()