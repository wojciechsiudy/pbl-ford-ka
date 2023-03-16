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
import time
from rclpy.node import Node
from builtins import float
from uwb_interfaces.msg import UwbPair, Point, PointPair, UwbMessage

from pbl_utils.ranging import UwbBluetoothConnection, UwbFatalError, UwbIncorrectData, UwbData

def make_uwb_message(data: UwbData) -> UwbMessage:
    message = UwbMessage()
    message.address = data.tag_address
    message.distance = data.distance
    message.power = data.power
    return message

def make_uwb_pair_message(nearest_data: UwbData, second_data: UwbData) -> UwbPair:
    message = UwbPair()
    message.nearest = make_uwb_message(nearest_data)
    message.second = make_uwb_message(second_data)
    return message

class UwbNode(Node):
    """
    Class holding ROS UWB node
    """
    def __init__(self):
        super().__init__('uwb_publisher')
        self.publisher_ = self.create_publisher(UwbPair, 'uwb', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        try:
            self.connection = UwbBluetoothConnection()
        except UwbFatalError:
            self.get_logger().info(
            "Fatal connection error. Exiting!")
            raise SystemExit
        self.connection.debug_level = 3
        self.subscription = self.create_subscription(
            PointPair,
            'anchors',
            self.anchors_callback,
            10)
        self.connection.connect()
        self.get_logger().info(
            "Node is set up.")

    def anchors_callback(self, msg):
        """
        This method is involved when "anchors" topic is fed with data
        """
        self.get_logger().info(
            "Anchors ticked, so we do.")
        try: # read data and publish it
            uwb_data_nearest = self.connection.read_uwb_data(msg.nearest.address)
            uwb_data_second = self.connection.read_uwb_data(msg.second.address)
            # if uwb_data_nearest is int or uwb_data_second is int:
            #     self.get_logger().info(
            # "Strange anwser recived Dropping frame.")
            #     return
            message = make_uwb_pair_message(uwb_data_nearest, uwb_data_second)
            self.publisher_.publish(message)
        except ConnectionError:
            self.connection.restart()
            self.get_logger().info(
            "Restarting.")
        except UwbIncorrectData: # if data was wrong dismiss publishing
            self.get_logger().info(
            "Wrong data recived.")


def main(args=None):
    rclpy.init(args=args)

    uwb_node = UwbNode()

    rclpy.spin(uwb_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    uwb_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
