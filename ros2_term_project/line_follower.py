import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros2_term_project.stopLine_tracker import StopLineTracker
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .line_tracker import LineTracker
import cv_bridge


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker, stop_line_tracker: StopLineTracker):
        super().__init__('line_follower')

        self.stop_timer = None
        self.line_tracker = line_tracker
        self.stop_line_tracker = stop_line_tracker
        self.bridge = cv_bridge.CvBridge()

        self.image_subscription_ = None
        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'start_car',
            self.car_info_listener_callback,
            10
        )

        self.vel_info_publisher = self.create_publisher(
            Twist,
            "vel_info",
            10
        )
        
        self.stopped = False  # 차량 정지 상태를 추적
        self.stop_duration = 5.0  # 정지 후 대기 시간 (초)

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        self.get_logger().info("car name : %s" % car)
        self.image_subscription_ = self.create_subscription(
            Image,
            f'/{car}/{car}_camera/image_raw',
            self.image_callback,
            10)

    def image_callback(self, image: Image):
        if self.stopped:
            return
        self.get_logger().info("image_callback method execute!!")
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        self.stop_line_tracker.process(img)

        twist = Twist()
        # 정지선이 감지되면 차량을 정지
        if self.stop_line_tracker.stop_line_detected and not self.stopped:
            self.get_logger().info("Stop line detected! Stopping the car.")
            self.stop_car()
            return

        # 정지선이 없을 경우 정상적인 주행
        self.line_tracker.process(img)
        twist.angular.z = (-1) * self.line_tracker.delta / 300
        if abs(twist.angular.z) > 0.15:  # 회전 값이 크면 속도를 줄임
            twist.linear.x = 3.0
        else:
            twist.linear.x = 6.0

        self.vel_info_publisher.publish(twist)

    def stop_car(self):
        """차량을 정지시키고 3초 후에 다시 출발하게 설정"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.get_logger().info("Publishing stop command.")

        # 차량을 멈추는 명령을 반복적으로 퍼블리시하여 확실히 전달
        for _ in range(10):
            self.vel_info_publisher.publish(twist)
            self.get_logger().info("Stop command published.")

        # 정지 상태 설정 및 타이머 시작
        self.stopped = True
        self.stop_timer = self.create_timer(self.stop_duration, self.resume_driving)

    def resume_driving(self):
        self.get_logger().info("Resuming driving.")
        self.stopped = False
        self.stop_timer.cancel()  # 타이머 취소


def main(args=None):
    rclpy.init(args=args)

    tracker = LineTracker()
    stop_line_tracker = StopLineTracker()
    follower = LineFollower(tracker, stop_line_tracker)

    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
