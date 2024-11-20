import os

import rclpy
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity
import tf_transformations as tf


class PriusSpawner(Node):
    def __init__(self):
        super().__init__('prius_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        # 차량들을 미리 생성
        self.spawn('PR001', (90.0, -12.0, 0.2))  # PR001
        self.spawn('PR002', (90.0, -16.0, 0.2))  # PR002

    def spawn(self, name, position):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        roll, pitch, yaw = 0.0, 0.0, -1.57
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion

        package_share_directory = get_package_share_directory('ros2_term_project')
        sdf_path = os.path.join(package_share_directory, 'worlds', 'model.sdf')

        if not os.path.exists(sdf_path):
            self.get_logger().error(f"SDF file not found: {sdf_path}")
            return
        with open(sdf_path, "r") as sdf_file:
            sdf_content = sdf_file.read()

        sdf_content = sdf_content.replace("__model__", name)
        sdf_content = sdf_content.replace("__plugin__", f"{name}_ackermann_drive")

        request = SpawnEntity.Request()
        request.name = name
        # request.xml = open("/home/ros2/.gazebo/models/prius_hybrid/model.sdf").read()
        request.xml = sdf_content
        request.robot_namespace = f"/{name}"  # set namespace
        request.initial_pose = pose

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Spawning {name} with namespace {request.robot_namespace}")

        if future.result() is not None:
            self.get_logger().info(f'Successfully spawned {name}')
        else:
            self.get_logger().error(f'Failed to spawn {name}')


def main(args=None):
    rclpy.init(args=args)
    spawner = PriusSpawner()
    rclpy.spin(spawner)
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
