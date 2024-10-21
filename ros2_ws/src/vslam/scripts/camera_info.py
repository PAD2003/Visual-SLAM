import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

class CameraInfoSubscriber(Node):
    def __init__(self):
        super().__init__('camera_info_subscriber')
        self.client = self.create_client(GetParameters, '/rviz2/get_parameters')
        self.request = GetParameters.Request()
        self.request.names = ['view_controller']

    def get_camera_info(self):
        self.client.call_async(self.request)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoSubscriber()
    node.get_camera_info()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()