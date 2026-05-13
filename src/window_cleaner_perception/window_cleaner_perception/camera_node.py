import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.bridge = CvBridge()
        self._frame_count = 0
        self._last_log_time = time.time()

        self.sub = self.create_subscription(
            Image,
            '/robot/camera/image_raw',
            self._image_callback,
            10
        )
        self.pub = self.create_publisher(Image, '/perception/debug_image', 10)
        self.get_logger().info('camera_node started — subscribing to /robot/camera/image_raw')

    def _image_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.putText(
            cv_img,
            f'frame {self._frame_count}',
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2
        )

        out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)

        self._frame_count += 1
        now = time.time()
        if self._frame_count % 100 == 0:
            elapsed = now - self._last_log_time
            fps = 100.0 / elapsed if elapsed > 0 else 0.0
            self.get_logger().info(f'frames={self._frame_count}  fps={fps:.1f}')
            self._last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
