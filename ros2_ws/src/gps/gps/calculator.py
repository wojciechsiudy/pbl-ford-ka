# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copyright 2022 Wojciech Siudy and Maciej Bolesta
# Silesian University of Technology

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from uwb_interfaces.msg import Point as Point_msg, PointPair, UwbMessage

from gps.ka_utils import Point, calculate_position
import gps.pointsDB


class PositionCalculator(Node):

    def __init__(self):
        super().__init__('calculator')
        self.distance_l = 0.0
        self.distance_r = 0.0
        self.distance_p = 0.0
        self.publisher_ = self.create_publisher(Point_msg, 'calculated_position', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.anchor_A = Point(0.0, 0.0)
        self.anchor_B = Point(0.0, 0.0)
        self.gps_position = Point(0.0, 0.0)
        self.gps_position = self.create_subscription(
            Point_msg,
            'gps',
            self.gps_callback,
            10
        )
        self.anchors_subscription = self.create_subscription(
            PointPair,
            'anchors',
            self.anchors_callback,
            10)
        self.anchors_subscription  # prevent unused variable warning
        self.uwb_subscription = self.create_subscription(
            UwbMessage,
            'uwb',
            self.uwb_callback,
            10)
        self.uwb_subscription  # prevent unused variable warning

    def gps_callback(self, msg):
        self.gps_position = read_point_from_message(msg)

    def anchors_callback(self, msg):
        self.anchor_A = read_point_from_message(msg.nearest)
        self.anchor_B = read_point_from_message(msg.second)

    def uwb_callback(self, msg):
        self.distance_l = msg.l
        self.distance_r = msg.r
        self.distance_t = msg.t

    def timer_callback(self):
        position = calculate_position(self.anchor_A, self.anchor_B, self.gps_position, self.distance_l, self.distance_r)
        print("CALCULATED POSITION:", position.x, position.y)
        msg = Point_msg()
        msg.x = position.x
        msg.y = position.y
        msg.address = "tag"
        self.publisher_.publish(msg)


def read_point_from_message(point_msg):
    return Point(point_msg.x, point_msg.y, point_msg.address)


def main(args=None):
    rclpy.init(args=args)

    position_subscriber = PositionCalculator()

    rclpy.spin(position_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
