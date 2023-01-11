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

from uwb.uwb_utils import getAverage, getDistanceArray

class PositionPublisher(Node):

    def __init__(self):
        super().__init__('uwb_publisher')
        self.publisher_ = self.create_publisher(String, 'uwbT', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.distance = 0
        self.address = "none"
        self.subscription = self.create_subscription(
            String,
            'anchor',
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        self.address = msg.data

    def timer_callback(self):
        msg = String()
        self.distance = getAverage(getDistanceArray("/dev/UWBt", self.address, 5))
        msg.data = '%f;%f' % (self.distance)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


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
