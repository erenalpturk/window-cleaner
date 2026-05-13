import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


class DirtSegmenter(Node):
    """
    Segments dirty regions from camera image using HSV thresholding.
    Publishes /perception/dirty_regions (PoseArray) + /perception/dirty_areas (Float32MultiArray).
    Debug image on /perception/debug_dirt.
    """

    def __init__(self):
        super().__init__('dirt_segmenter')

        # HSV thresholds — tunable via perception_params.yaml
        self.declare_parameter('hsv_h_low', 0)
        self.declare_parameter('hsv_h_high', 30)
        self.declare_parameter('hsv_s_low', 20)
        self.declare_parameter('hsv_s_high', 150)
        self.declare_parameter('hsv_v_low', 20)
        self.declare_parameter('hsv_v_high', 120)
        self.declare_parameter('morph_kernel', 7)
        self.declare_parameter('min_area_px', 50)

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/robot/camera/image_raw', self._image_callback, 10)
        self.pub_poses = self.create_publisher(
            PoseArray, '/perception/dirty_regions', 10)
        self.pub_areas = self.create_publisher(
            Float32MultiArray, '/perception/dirty_areas', 10)
        self.pub_debug = self.create_publisher(
            Image, '/perception/debug_dirt', 10)

        self.get_logger().info('dirt_segmenter started')

    def _image_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug = cv_img.copy()

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        h_low = self.get_parameter('hsv_h_low').value
        h_high = self.get_parameter('hsv_h_high').value
        s_low = self.get_parameter('hsv_s_low').value
        s_high = self.get_parameter('hsv_s_high').value
        v_low = self.get_parameter('hsv_v_low').value
        v_high = self.get_parameter('hsv_v_high').value
        morph_k = self.get_parameter('morph_kernel').value
        min_area = self.get_parameter('min_area_px').value

        lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
        upper = np.array([h_high, s_high, v_high], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morph_k, morph_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centers, areas = [], []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
            centers.append((cx, cy))
            areas.append(float(area))
            cv2.drawContours(debug, [cnt], -1, (0, 255, 0), 2)
            cv2.circle(debug, (int(cx), int(cy)), 5, (0, 0, 255), -1)

        self._publish_regions(centers, msg.header)
        self._publish_areas(areas, msg.header)
        self._publish_debug(debug, msg.header)

        if len(centers) > 0:
            self.get_logger().debug(f'dirty regions detected: {len(centers)}')

    def _publish_regions(self, centers, header):
        msg = PoseArray()
        msg.header = header
        for cx, cy in centers:
            p = Pose()
            p.position.x = cx
            p.position.y = cy
            p.position.z = 0.0
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.pub_poses.publish(msg)

    def _publish_areas(self, areas, header):
        msg = Float32MultiArray()
        msg.data = areas
        self.pub_areas.publish(msg)

    def _publish_debug(self, cv_img, header):
        out = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out.header = header
        self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DirtSegmenter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
