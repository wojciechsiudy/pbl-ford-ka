import rclpy
from rclpy.node import Node

import json
import csv

from uwb_interfaces.msg import PointPair
from uwb_interfaces.msg import Point as Point_msg, UwbMessage
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
        self.anchor_B = Point(0.0, 0.0)
        self.uwb_l = 0
        self.uwb_r = 0
        self.uwb_subscription = self.create_subscription(
            UwbMessage,
            'uwb',
            self.uwb_callback,
            10)
        self.uwb_subscription
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
        self.save_counter = 0


    def uwb_callback(self, msg):
        self.uwb_l = msg.l
        self.uwb_r = msg.r

    def gps_callback(self, msg):
        self.gps_position = read_point_from_message(msg)

    def anchors_callback(self, msg):
        self.anchor_A = read_point_from_message(msg.nearest)
        self.anchor_B = read_point_from_message(msg.second)

    def calculated_callback(self, msg):
        self.calculated_position = read_point_from_message(msg)

    def make_csv_row(self):
        result = []
        result.append(self.gps_position.x)
        result.append(self.gps_position.y)
        result.append(self.anchor_A.address)
        result.append(self.anchor_B.address)
        result.append(self.uwb_l)
        result.append(self.uwb_r)
        return result

    def timer_callback(self):
        #print(self.calculated_position.x)
        #calculated = self.transform_to_dg_point(self.calculated_position)
        #self.calculated_publisher.publish(calculated)
        #gps = self.transform_to_dg_point(self.gps_position)
        #self.gps_publisher.publish(gps)
        #with open("/home/wojtek/pbl/bag-json/" + str(self.save_counter) + ".json", "w+") as outfile:
        #    outfile.write(json.dumps({'gps': {
        #                                        'x': self.gps_position.x,
        #                                        'y': self.gps_position.y},
        #                              'anchors': {
        #                                        'a': self.anchor_A.address,
        #                                        'b': self.anchor_B.address
        #                              }}, indent=3))
        #self.save_counter += 1
        with open("/home/wojtek/pbl/data.csv", "a+", newline="") as file:
            csv_writer = csv.writer(file)
            csv_writer.writerow(self.make_csv_row())


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
