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

from gps.ka_utils import Point
import gps.pointsDB

class PositionSubscriber(Node):

    def __init__(self):
        super().__init__('position_subscriber')
        self.anchor = "none"
        self.publisher_ = self.create_publisher(String, 'anchor', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            String,
            'gps',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        raw_coordinates = msg.data.split(';')
        coordinates = Point(float(raw_coordinates[0]), float(raw_coordinates[1]))
        for point in gps.pointsDB.getPoints(1):
            if point.is_around(coordinates):
                self.anchor = point.address
                #todo nearest
        self.get_logger().info('Position: %f; %f. Nearest anchor: %s' % (coordinates.x, coordinates.y, self.anchor))

    def timer_callback(self):
        msg = String()
        msg.data = '%s' % (self.anchor)
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
