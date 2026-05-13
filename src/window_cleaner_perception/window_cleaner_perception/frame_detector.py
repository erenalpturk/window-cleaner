import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from cv_bridge import CvBridge
import cv2
import numpy as np


class FrameDetector(Node):
    """
    Detects glass frame boundary from camera image using Canny + Hough.
    Publishes /perception/glass_boundary and /perception/debug_frame.
    """

    def __init__(self):
        super().__init__('frame_detector')

        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hough_min_length', 50)
        self.declare_parameter('hough_max_gap', 20)
        self.declare_parameter('blur_kernel', 5)

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/robot/camera/image_raw', self._image_callback, 10)
        self.pub_boundary = self.create_publisher(
            PolygonStamped, '/perception/glass_boundary', 10)
        self.pub_debug = self.create_publisher(
            Image, '/perception/debug_frame', 10)

        self.get_logger().info('frame_detector started')

    def _image_callback(self, msg: Image):
        p = self._get_params()
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = cv_img.shape[:2]

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (p['blur_kernel'], p['blur_kernel']), 0)
        edges = cv2.Canny(blurred, p['canny_low'], p['canny_high'])

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,
            minLineLength=p['hough_min_length'],
            maxLineGap=p['hough_max_gap']
        )

        debug = cv_img.copy()
        corners = self._estimate_boundary(lines, w, h, debug)

        self._publish_boundary(corners, msg.header)
        self._publish_debug(debug, msg.header)

    def _estimate_boundary(self, lines, w, h, debug_img):
        """
        Groups detected lines into H/V, picks outermost → glass boundary corners.
        Falls back to image edges when detection fails.
        """
        h_lines, v_lines = [], []

        if lines is not None:
            for seg in lines:
                x1, y1, x2, y2 = seg[0]
                angle = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
                if angle < 20 or angle > 160:
                    h_lines.append((x1, y1, x2, y2))
                elif 70 < angle < 110:
                    v_lines.append((x1, y1, x2, y2))
                cv2.line(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 1)

        top = min((min(y1, y2) for x1, y1, x2, y2 in h_lines), default=0)
        bottom = max((max(y1, y2) for x1, y1, x2, y2 in h_lines), default=h)
        left = min((min(x1, x2) for x1, y1, x2, y2 in v_lines), default=0)
        right = max((max(x1, x2) for x1, y1, x2, y2 in v_lines), default=w)

        corners = [(left, top), (right, top), (right, bottom), (left, bottom)]
        pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(debug_img, [pts], isClosed=True, color=(255, 0, 0), thickness=2)
        return corners

    def _publish_boundary(self, corners, header):
        msg = PolygonStamped()
        msg.header = header
        for x, y in corners:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            msg.polygon.points.append(p)
        self.pub_boundary.publish(msg)

    def _publish_debug(self, cv_img, header):
        out = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out.header = header
        self.pub_debug.publish(out)

    def _get_params(self):
        k = self.get_parameter('blur_kernel').value
        if k % 2 == 0:
            k += 1
        return {
            'canny_low': self.get_parameter('canny_low').value,
            'canny_high': self.get_parameter('canny_high').value,
            'hough_min_length': self.get_parameter('hough_min_length').value,
            'hough_max_gap': self.get_parameter('hough_max_gap').value,
            'blur_kernel': k,
        }


def main(args=None):
    rclpy.init(args=args)
    node = FrameDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
