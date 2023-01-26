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
from uwb_interfaces.msg import PointPair
from uwb_interfaces.msg import Point as Point_msg

from gps.ka_utils import Point
import gps.pointsDB


class PositionSubscriber(Node):

    def __init__(self):
        super().__init__('position_subscriber')
        self.anchor = Point(0.0, 0.0, "none")
        self.anchor_second = Point(0.0, 0.0, "none")
        self.publisher_ = self.create_publisher(PointPair, 'anchors', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            Point_msg,
            'gps',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        coordinates = Point(msg.x, msg.y)
        points_around = []
        for point in gps.pointsDB.getPoints(1):
            distance = coordinates.get_distance_to(point)
            print(point.address, distance)
            if distance < float('inf'):
                points_around.append((point, coordinates.get_distance_to(point)))
        points_around.sort(key=lambda x: x[1])
        for tpl in points_around:
            print("SORT TEST", tpl[0].address, tpl[1])

        self.anchor = points_around[0][0]
        self.anchor_second = points_around[1][0]
        self.get_logger().info(
            'Position: %f; %f. Nearest anchor: %s' % (coordinates.x, coordinates.y, self.anchor.address))

    def timer_callback(self):
        msg = PointPair()
        msg.nearest = create_point_msg(self.anchor.x, self.anchor.y, self.anchor.address)
        msg.second = create_point_msg(self.anchor_second.x, self.anchor_second.y, self.anchor_second.address)
        self.publisher_.publish(msg)


def create_point_msg(x_val, y_val, address_val):
    msg = Point_msg()
    msg.x = x_val
    msg.y = y_val
    msg.address = address_val
    return msg


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
