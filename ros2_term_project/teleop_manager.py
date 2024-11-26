import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class TeleopManager(Node):
    PUB_RATE = 10.0
    TARGET_X1 = 35.0
    TARGET_Y1 = -77.0
    TARGET_X2 = 35.0
    TARGET_Y2 = -63.0

    def __init__(self):
        super().__init__('teleop_manager')
        self.publisher = self.create_publisher(Twist, 'teleop_cmd_vel', 1)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/demo/odom_demo',
            self.odom_callback,
            100
        )
        timer_period = 1 / TeleopManager.PUB_RATE
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.position_timer = self.create_timer(0.5, self.position_callback)  # 새로운 타이머 추가
        self.target_linear_y = -2.0
        self.target_angular_z = float(0)
        self.msg = Twist()

        self.current_x = 35.0
        self.current_y = -63.0
        self.target = (TeleopManager.TARGET_X1, TeleopManager.TARGET_Y1)
        self.reached_target_2 = False

    def odom_callback(self, msg: Odometry):
        """Callback to get robot's current position from Odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        distance_to_target = math.sqrt((self.current_x - self.target[0]) ** 2 +
                                       (self.current_y - self.target[1]) ** 2)

        # Check if the robot has reached the current target
        if distance_to_target < 0.5:
            self.change_target()  # Change target when reached

    def change_target(self):
        """Change the target coordinates once the current target is reached."""
        if self.target == (TeleopManager.TARGET_X1, TeleopManager.TARGET_Y1):
            # Change target from Target 1 to Target 2
            self.target = (TeleopManager.TARGET_X2, TeleopManager.TARGET_Y2)
            self.target_linear_y = 2.0  # Move towards Target 2
            self.target_angular_z = 0.0  # No rotation needed
            #self.get_logger().info("Reached Target 1, moving to Target 2")
        elif self.target == (TeleopManager.TARGET_X2, TeleopManager.TARGET_Y2):
            # Change target from Target 2 to Target 1
            self.target = (TeleopManager.TARGET_X1, TeleopManager.TARGET_Y1)
            self.target_linear_y = -2.0  # Move towards Target 1
            self.target_angular_z = 0.0  # No rotation needed
            self.get_logger().info("Reached Target 2, moving to Target 1")

    def pub_callback(self):
        # Publish the velocity message to keep the robot moving towards the current target
        self.msg.linear.y = self.target_linear_y
        self.msg.angular.z = self.target_angular_z

        # Publish the velocity command
        self.publisher.publish(self.msg)

    def position_callback(self):
        """Print current position of the robot every 0.5 seconds."""
        #self.get_logger().info(f"Current Position: x={self.current_x}, y={self.current_y}")


def main(args=None):
    rclpy.init(args=args)
    manager = TeleopManager()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass

    # Gracefully stop the robot on shutdown
    manager.msg.linear.y = 0.0
    manager.msg.angular.z = float(0)
    manager.publisher.publish(manager.msg)

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
