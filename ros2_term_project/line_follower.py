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

        self.obstacle_subscription = None
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

        self.stopped = False
        self.stop_duration = 5.0  # 정지선 감지 시 정지 시간
        self.obstacle_detected = False
        self.safe_distance = 6.0  # 장애물과의 안전 거리 (m)

        self.obstacle_detected = False
        self.obstacle_timer = None  # 장애물 제거 후 타이머

    def obstacle_callback(self, msg: String):
        """장애물 상태 메시지를 처리하는 콜백"""
        if msg.data.startswith('distance:'):  # 거리 정보를 포함한 메시지
            try:
                distance = float(msg.data.split(':')[1])
                if distance < self.safe_distance and not self.stopped:
                    # 안전거리 내에 장애물이 있으면 정지
                    self.obstacle_detected = True
                    self.stop_car()
                    self.get_logger().info(f"장애물 감지 (거리: {distance}m): 차량 정지")
                elif distance >= self.safe_distance and self.stopped:
                    # 안전거리 이상이면 주행 재개
                    self.obstacle_detected = False
                    self.resume_driving()
                    self.get_logger().info(f"안전거리 확보 (거리: {distance}m): 주행 재개")
            except ValueError:
                self.get_logger().error("거리 정보 처리 중 오류 발생")

    def stop_for_obstacle(self):
        """장애물 감지로 인한 차량 정지"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.vel_info_publisher.publish(twist)
        self.get_logger().info("장애물 감지: 차량 정지")

    def resume_from_obstacle(self):
        """장애물 제거 후 주행 재개"""
        self.get_logger().info("3초 경과: 주행 재개")
        self.obstacle_detected = False  # 이 플래그를 False로 설정하여 image_callback이 다시 실행되도록 함
        if self.obstacle_timer is not None:
            self.obstacle_timer.cancel()
            self.obstacle_timer = None

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        self.get_logger().info("car name : %s" % car)
        self.image_subscription_ = self.create_subscription(
            Image,
            f'/{car}/{car}_camera/image_raw',
            self.image_callback,
            10)

        # 장애물 감지 상태를 구독
        self.obstacle_subscription = self.create_subscription(
            String,
            f'/{car}/obstacle_status',
            self.obstacle_callback,
            10
        )

    def image_callback(self, image: Image):
        # 장애물이 감지되면 차량 제어를 중단
        if self.obstacle_detected:
            return

        if self.stopped:
            return

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
        """차량을 정지시키고 5초 후에 다시 출발하게 설정"""
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
        self.obstacle_detected = False
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
