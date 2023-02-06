import rclpy
from rclpy.node import Node


from uwb_interfaces.msg import PointPair
from uwb_interfaces.msg import Point as Point_msg
from geometry_msgs.msg import PointStamped

from gps.ka_utils import Point

class TopicTranslator(Node):
    def __init__(self):
        super().__init__('topic_translator')
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.gps_position = Point(0.0, 0.0)
        self.calculated_position = Point(0.0, 0.0)
        self.anchor_A = Point(0.0, 0.0)
        self.anchor_A = Point(0.0, 0.0)
        self.gps_subscription = self.create_subscription(
            Point_msg,
            'gps',
            self.gps_callback,
            10)
        self.gps_subscription
        self.anchors_subscription = self.create_subscription(
            PointPair,
            'anchors',
            self.anchors_callback,
            10)
        self.anchors_subscription
        self.calculated_subscription = self.create_subscription(
            Point_msg,
            'calculated_position',
            self.calculated_callback,
            10)
        self.calculated_subscription
        self.calculated_publisher = self.create_publisher(PointStamped, 'rviz_calculated', 10)
        self.gps_publisher = self.create_publisher(PointStamped, 'rviz_gps', 10)

    def gps_callback(self, msg):
        self.gps_position = read_point_from_message(msg)

    def anchors_callback(self, msg):
        self.anchor_A = read_point_from_message(msg.nearest)
        self.anchor_A = read_point_from_message(msg.second)

    def calculated_callback(self, msg):
        self.calculated_position = read_point_from_message(msg)

    def timer_callback(self):
        print(self.calculated_position.x)
        calculated = self.transform_to_dg_point(self.calculated_position)
        self.calculated_publisher.publish(calculated)
        gps = self.transform_to_dg_point(self.gps_position)
        self.gps_publisher.publish(gps)

    def transform_to_dg_point(self, point):
        result = PointStamped()
        result.point.x = (point.x - 50) * 10
        result.point.y = (point.y - 18) * 10
        result.header.frame_id = "calculated"
        result.header.stamp = self.get_clock().now().to_msg()
        return result



def read_point_from_message(point_msg):
    return Point(point_msg.x, point_msg.y, point_msg.address)

def main(args=None):
    rclpy.init(args=args)
    topic_translator = TopicTranslator()
    rclpy.spin(topic_translator)
    topic_translator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
