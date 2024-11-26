import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import os
from ament_index_python.packages import get_package_share_directory


class BoxSpawn(Node):

    def __init__(self):
        super().__init__('spawn_entity')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()
        self.future = None

    def send_request(self):
        self.req.name = "box"
        model_file = os.path.join(get_package_share_directory('ros2_term_project'), 'worlds', 'box.sdf')
        model_xml = open(model_file).read()
        self.req.xml = model_xml
        self.req.initial_pose.position.x = 35.0
        self.req.initial_pose.position.y = -63.0
        self.req.initial_pose.position.z = 1.2
        quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        self.future = self.client.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    client = BoxSpawn()
    client.send_request()
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                print('response status = ', response.status_message)
            except Exception as e:
                client.get_logger().info('Service call failed %s' % e)
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()