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
from builtins import float
from uwb_interfaces.msg import UwbMessage, Point, PointPair

from uwb.uwb_utils import UwbSerialConnection


class UwbNode(Node):

    def __init__(self):
        super().__init__('uwb_publisher')
        self.publisher_ = self.create_publisher(UwbMessage, 'uwb', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.address_nearest = "none"
        self.address_second = "none"
        self.connection_l = UwbSerialConnection("/dev/UWBl")
        self.connection_r = UwbSerialConnection("/dev/UWBr")
        self.subscription = self.create_subscription(
            PointPair,
            'anchors',
            self.listener_callback,
            10)
        self.connection_l.begin()
        self.connection_r.begin()
        self.get_logger().info(
            "Setting up uwb publisher. No data will be logged here to keep performance. If you want to see data in "
            "console toggle 'DEBUG' value in uwb_utils.py to True.")

    def listener_callback(self, msg):
        changed = False
        if not self.address_nearest == msg.nearest.address:
            self.address_nearest = msg.nearest.address
            changed = True
        if not self.address_second == msg.second.address:
            self.address_second = msg.second.address
            changed = True
        if changed:
            self.connection_l.set_address(self.address_nearest)
            self.connection_r.set_address(self.address_second)


    def timer_callback(self):
        distances = UwbMessage()
        distances.l = float(self.connection_l.get_distance())
        distances.r = float(self.connection_r.get_distance())
        distances.t = 1.0
        self.publisher_.publish(distances)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    uwb_node = UwbNode()

    rclpy.spin(uwb_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    uwb_node.connection_l.end()
    uwb_node.connection_r.end()
    uwb_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
