import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class GlassSurfaceMarker(Node):
    """Publishes a static, translucent blue CUBE in `map` frame that visualises
    the glass surface for Foxglove's 3D panel. Foxglove does not read the SDF
    world, so the `glass_surface` model defined in glass_basic.sdf is invisible
    there; this node mirrors its dimensions as a ROS Marker."""

    def __init__(self):
        super().__init__('glass_surface_marker')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('size_x', 5.0)
        self.declare_parameter('size_y', 3.0)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('z_offset', -0.005)
        self.declare_parameter('alpha', 0.35)
        self.declare_parameter('rate_hz', 1.0)

        self.pub = self.create_publisher(
            Marker, '/perception/glass_surface_marker', 1)

        period = 1.0 / float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info('glass_surface_marker started')

    def _tick(self):
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').value
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'glass_surface'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD

        m.pose.position.x = float(self.get_parameter('center_x').value)
        m.pose.position.y = float(self.get_parameter('center_y').value)
        m.pose.position.z = float(self.get_parameter('z_offset').value)
        m.pose.orientation.w = 1.0

        m.scale.x = float(self.get_parameter('size_x').value)
        m.scale.y = float(self.get_parameter('size_y').value)
        m.scale.z = 0.005

        m.color.r = 0.25
        m.color.g = 0.55
        m.color.b = 0.85
        m.color.a = float(self.get_parameter('alpha').value)

        m.frame_locked = True
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = GlassSurfaceMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
