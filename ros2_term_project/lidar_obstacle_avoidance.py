import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ros2_term_project import line_follower
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class LidarObstacleAvoidance(Node):
    def __init__(self):
        try:
            super().__init__('lidar_obstacle_avoidance')

            # QoS 설정을 SYSTEM_DEFAULT로 지정
            self.qos_profile = QoSProfile(depth=10)
            # 지정된 차량 정보를 받아올 subscription
            self.car_info_subscription_ = self.create_subscription(
                String,
                'start_car',
                self.car_info_listener_callback,
                10
            )

            self.twist_publisher = None

            # Lidar 데이터를 구독하여 장애물 감지를 위한 subscription 생성
            self.lidar_subscription = None

            self.get_logger().info('Lidar 구독자가 생성되었습니다.')

            # 기본 속도 설정
            self.current_speed = 2.0  # 기본 속도 (단위: m/s)
            self.min_safe_distance = 8.0  # 안전 정지 거리를 8m로 설정
            self.safe_resume_distance = 6.0  # 재출발을 위한 최소 거리를 6m로 설정
            self.obstacle_detected = False
            self.previous_distance = float('inf')

            # 경사도 계산을 위한 이전 위치 저장
            self.prev_position = None
            self.slope_threshold = 5.0  # 경사도 임계값 (단위: degrees)

            self.odom_subscription = None
            # Lidar 데이터 사용 여부
            self.ignore_lidar = False

            # 차선 감지와 정지선 감지를 위한 publisher 생성
            self.pause_publisher = self.create_publisher(String, f'/PR001/pause_lane_following', self.qos_profile)

            # 장애물 상태를 알리는 publisher 추가
            self.obstacle_publisher = None

            self.get_logger().info('LidarObstacleAvoidance 노드가 성공적으로 초기화되었습니다.')

        except Exception as e:
            print(f'노드 초기화 중 오류 발생: {e}')

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량에 속도 정보를 전달할 publisher 생성 및 로그 출력
        self.twist_publisher = self.create_publisher(Twist, f'/{car}/cmd_demo', self.qos_profile)
        self.get_logger().info('Twist 퍼블리셔가 생성되었습니다.')
        # 장애물 상태를 알리는 publisher 추가
        self.obstacle_publisher = self.create_publisher(
            String,
            f'/{car}/obstacle_status',
            10
        )
        # Lidar 데이터를 구독하여 장애물 감지를 위한 subscription 생성
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            f'/{car}/scan',  # 실제 토픽 이름으로 수정
            self.lidar_callback,
            self.qos_profile
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            f'/{car}/odom',
            self.odom_callback,
            10
        )

    def lidar_callback(self, msg):
        if self.ignore_lidar:
            self.get_logger().info("Ignoring Lidar data due to steep slope.")
            return
        try:
            # Lidar 데이터에서 가장 가까운 거리 탐색
            valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > 0.1]

            if len(valid_ranges) > 0:
                min_distance = min(valid_ranges)
            else:  # 아무것도 인식되지 않음
                return

            self.get_logger().info(f'Lidar 데이터 수신 중... 최소 거리: {min_distance}m')

            # 장애물이 안전 거리 이내에 있는지 여부를 업데이트
            if 1 <= min_distance < self.min_safe_distance:
                self.get_logger().info(f'장애물 감지! 최소 거리: {min_distance}m. 속도 감소.')
                self.obstacle_detected = True

                # 장애물 감지 상태와 거리 정보 발행
                status_msg = String()
                status_msg.data = f'distance:{min_distance}'
                self.obstacle_publisher.publish(status_msg)

                # 차량 정지
                twist = Twist()
                twist.linear.x = 0.0
                self.twist_publisher.publish(twist)

            # 장애물이 감지된 상태에서, 거리가 1m 이상 증가했을 때
            elif self.obstacle_detected and (min_distance - self.previous_distance) > 1.0:
                self.get_logger().info(f'장애물과의 거리가 충분히 증가함. 차량 출발.')
                self.obstacle_detected = False

                # 장애물 제거 상태와 거리 정보 발행
                status_msg = String()
                status_msg.data = f'distance:{min_distance}'
                self.obstacle_publisher.publish(status_msg)

            # 장애물이 감지되지 않는 상태
            elif min_distance > self.min_safe_distance:
                if self.obstacle_detected:
                    self.get_logger().info('장애물이 더 이상 감지되지 않음.')
                    self.obstacle_detected = False

                    # 장애물 제거 상태와 거리 정보 발행
                    status_msg = String()
                    status_msg.data = f'distance:{min_distance}'
                    self.obstacle_publisher.publish(status_msg)

            self.previous_distance = min_distance

        except Exception as e:
            self.get_logger().error(f'Lidar 데이터 처리 중 오류 발생: {e}')

    def publish_pause(self, pause):
        try:
            pause_msg = String()
            pause_msg.data = 'pause' if pause else 'resume'
            self.pause_publisher.publish(pause_msg)
            self.get_logger().info(f'차선 감지 및 정지선 감지 제어 신호 발행: {pause_msg.data}')
        except Exception as e:
            self.get_logger().error(f'차선 감지 제어 신호 발행 중 오류 발생: {e}')

    def status_check(self):
        # 상태 확인 로그를 출력하여 노드가 정상 작동 중인지 확인
        self.get_logger().info('노드 상태 점검 중...')


def main(args=None):
    rclpy.init(args=args)

    try:
        lidar_obstacle_avoidance = LidarObstacleAvoidance()
        rclpy.spin(lidar_obstacle_avoidance)
    except Exception as e:
        print(f"노드 실행 중 오류 발생: {e}")
    finally:
        if 'lidar_obstacle_avoidance' in locals():
            lidar_obstacle_avoidance.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()