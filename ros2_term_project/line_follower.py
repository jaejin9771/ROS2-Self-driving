import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros2_term_project.end_line_tracker import EndLineTracker
from ros2_term_project.stopLine_tracker import StopLineTracker
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .line_tracker import LineTracker
import cv_bridge


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker, stop_line_tracker: StopLineTracker,
                 end_line_tracker: EndLineTracker):
        super().__init__('line_follower')

        self.stop_timer = None
        self.line_tracker = line_tracker
        self.stop_line_tracker = stop_line_tracker
        self.end_line_tracker = end_line_tracker

        self.stop_speed = 2.0  # 초기 속도
        self.deceleration_rate = 0.1  # 속도 감소율
        self.deceleration_timer = None  # 감속 타이머
        self.bridge = cv_bridge.CvBridge()

        self.image_subscription_ = None
        self.deceleration_timer = None  # 감속타이머

        self.odom_subscription = None

        # 추가 상태 변수
        self.ignore_line_tracker = False  # LineTracker를 무시할지 여부
        self.ignore_timer = None  # LineTracker 무시 타이머

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
        self.angular_scale = None

        # 경사 관련 변수
        self.slope_threshold = 0.37  # z 값이 이 값 이상이면 경사로로 간주
        self.on_slope = False  # 경사로 상태

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

        self.odom_subscription = self.create_subscription(
            Odometry,
            f'/{car}/odom_demo',
            self.odom_callback,
            10
        )
        if car == "PR001":
            self.angular_scale = 300
        elif car == "PR002":
            self.angular_scale = 200

    def odom_callback(self, msg: Odometry):
        """Odom 데이터를 처리하여 z 값으로 경사로 상태 관리"""
        current_z = msg.pose.pose.position.z

        # z 값이 경사로 임계값 이상이면 경사로 상태 활성화
        if current_z >= self.slope_threshold:
            self.get_logger().info(f"Current Z value: {current_z:.3f}")
            self.on_slope = True
            self.ignore_line_tracker = True
            self.get_logger().warn("Vehicle is on a slope! Ignoring LineTracker.")
        else:
            self.on_slope = False
            self.ignore_line_tracker = False

    def image_callback(self, image: Image):
        # 장애물이 감지되면 차량 제어를 중단
        if self.obstacle_detected:
            return

        # 이미 정지된 상태라면 추가 처리를 하지 않음
        if self.stopped:
            return

        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        twist = Twist()

        # End line 감지 처리
        self.end_line_tracker.process(img)
        if self.end_line_tracker.end_line_detected:
            self.get_logger().info("End line detected! Stopping the car permanently.")
            self.stop_permanently()  # 영구 정지 호출
            self.stopped = True  # 정지 상태로 전환
            return  # 모든 추가 동작 중단

        # Stop line 감지 처리
        self.stop_line_tracker.process(img)
        if self.stop_line_tracker.stop_line_detected:
            # 정지 상태 설정
            self.stopped = True
            # LineTracker를 3초 동안 무시하도록 설정
            self.ignore_line_tracker = True
            self.get_logger().info("Stop line detected! Stopping the car.")
            self.stop_car()
            return

        # LineTracker 무시 상태일 경우 직진만 수행
        if self.ignore_line_tracker:
            twist.linear.x = 6.0
            twist.angular.z = 0.0
            self.vel_info_publisher.publish(twist)
            self.get_logger().info("Ignoring LineTracker, moving straight.")
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
        twist = Twist()
        """차량을 정지시키고, 타이머를 통해 주행 재개"""
        if self.on_slope:
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.get_logger().info("Publishing stop command.")

        # 차량을 멈추는 명령을 반복적으로 퍼블리시하여 확실히 전달
        for _ in range(10):
            self.vel_info_publisher.publish(twist)

        # 정지 상태 설정
        self.stopped = True

        # Stop Line 감지 시에는 타이머를 설정해 다시 주행
        self.stop_timer = self.create_timer(self.stop_duration, self.resume_driving)

    def resume_driving(self):
        """End Line 감지 상태일 경우 동작하지 않음"""
        if self.end_line_tracker.end_line_detected:
            self.get_logger().info("End Line detected! Preventing resume driving.")
            return  # End Line 상태에서는 주행 재개 금지

        self.get_logger().info("Resuming driving.")
        self.obstacle_detected = False
        self.stopped = False
        self.stop_timer.cancel()  # 타이머 취소

        if self.ignore_line_tracker:
            self.ignore_timer = self.create_timer(3.0, self.enable_line_tracker)

    def enable_line_tracker(self):
        """3초 후 LineTracker 처리 재개"""
        self.ignore_line_tracker = False
        if self.ignore_timer:
            self.ignore_timer.cancel()
        self.get_logger().info("LineTracker re-enabled.")

    def stop_permanently(self):
        """속도를 점진적으로 줄이며 차량을 영구적으로 정지시키는 함수"""
        if self.deceleration_timer is not None:
            self.deceleration_timer.cancel()  # 기존 타이머가 실행 중이라면 취소

        self.get_logger().info("Initiating gradual stop.")

        # 초기 속도로 시작하여 점진적으로 줄이기
        self.stop_speed = 2.0
        self.deceleration_timer = self.create_timer(0.25, self.gradual_stop)

    def gradual_stop(self):
        """속도를 점진적으로 줄이는 함수"""
        if self.stop_speed <= 0.0:  # 속도가 0 이하가 되면 멈춤
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.vel_info_publisher.publish(twist)

            self.get_logger().info("Car stopped permanently.")

            # 타이머 종료
            if self.deceleration_timer is not None:
                self.deceleration_timer.cancel()
                self.deceleration_timer = None
            return

        # 현재 속도로 이동
        twist = Twist()
        twist.linear.x = self.stop_speed
        twist.angular.z = 0.0
        self.vel_info_publisher.publish(twist)

        self.get_logger().info(f"Decelerating... Current speed: {self.stop_speed:.2f} m/s")

        # 속도를 감소
        self.stop_speed -= self.deceleration_rate


def main(args=None):
    rclpy.init(args=args)

    tracker = LineTracker()
    stop_line_tracker = StopLineTracker()
    end_line_tracker = EndLineTracker()
    follower = LineFollower(tracker, stop_line_tracker, end_line_tracker)

    rclpy.spin(follower)

    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
