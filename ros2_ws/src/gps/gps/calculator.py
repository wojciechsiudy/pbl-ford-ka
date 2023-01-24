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
from uwb_interfaces.msg import Point as Point_msg, UwbMessage

from gps.ka_utils import Point
import gps.pointsDB


class PositionSubscriber(Node):

    def __init__(self):
        super().__init__('calculator')
        self.anchor = Point(0.0, 0.0, "none")
        self.publisher_ = self.create_publisher(Point_msg, 'calculated_position', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.anchor = Point()
        self.distance_l, self.distance_r, self.distance_t = 0.0
        self.anchor_subscription = self.create_subscription(
            Point_msg,
            'anchor',
            self.anchor_callback,
            10)
        self.anchor_subscription  # prevent unused variable warning
        self.uwb_subscription = self.create_subscription(
            UwbMessage,
            'uwb',
            self.uwb_callback,
            10)
        self.uwb_subscription  # prevent unused variable warning

    def anchor_callback(self, msg):
        self.anchor = Point(msg.x, msg.y, msg.z, msg.address)

    def uwb_callback(self, msg):
        self.distance_l = msg.l
        self.distance_r = msg.r
        self.distance_t = msg.t

    def timer_callback(self):
        msg = Point_msg()
        msg.x = self.anchor.x
        msg.y = self.anchor.y
        msg.address = self.anchor.address
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    position_subscriber = PositionSubscriber()

    rclpy.spin(position_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
