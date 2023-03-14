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
from uwb_interfaces.msg import Point as Point_msg

from pbl_utils.mapping import Point, get_position

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('position_publisher')
        self.publisher_ = self.create_publisher(Point_msg, 'gps', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Point_msg()
        position = Point(0, 0)
        position = get_position()
        msg.x = position.x
        msg.y = position.y
        print(position.x, position.y)
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)


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
