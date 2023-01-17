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

from gps.ka_utils import Point, get_position

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_pseudo_publisher')
        self.publisher_ = self.create_publisher(String, 'gps', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.node_first = True
        self.node_ticks = 0

    def timer_callback(self):
        self.node_ticks += 1
        msg = String()
        position = Point(0, 0)
        if self.node_first == True:
            position = Point(50.0, 18.0)
        else:
            position = Point(52.0, 20.0)
        msg.data = '%f;%f' % (position.x, position.y)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        if (self.node_ticks == 10):
            self.node_ticks = 0
            self.node_first = not self.node_first


def main(args=None):
    rclpy.init(args=args)

    position_publisher = PositionPublisher()

    rclpy.spin(position_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
