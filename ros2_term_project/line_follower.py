import rclpy
from rclpy.node import Node
from .ilne_tracker import LineTracker
import cv_bridge
from custom_interface.msg import CarInfo


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker):
        super().__init__('line_follower')

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            CarInfo,
            'start_car',
            self.car_info_listener_callback,
            10)