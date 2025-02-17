import rclpy
from rclpy.node import Node
from sensor_msgs.srv import SetCameraInfo
from sensor_msgs.msg import CameraInfo

class SetCameraInfoClient(Node):

    def __init__(self):
        super().__init__('set_camera_info_client')
        self.cli = self.create_client(SetCameraInfo, 'camera/set_camera_info')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetCameraInfo.Request()

    def send_request(self, camera_info):
        self.req.camera_info = camera_info
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)

    node = SetCameraInfoClient()
    camera_info = CameraInfo()
    # Populate camera_info with appropriate values
    # camera_info.header.frame_id = 'camera_frame'
    # camera_info.width = 640
    # camera_info.height = 480
    # camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    # camera_info.d = [k1, k2, p1, p2, k3]

    node.send_request(camera_info)

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info('Result: %r' % (response.success,))
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()