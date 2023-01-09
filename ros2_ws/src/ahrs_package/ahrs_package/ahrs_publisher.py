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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from ahrs_package.ahrs_utils import *


class AhrsPublisher(Node):

    def __init__(self):
        super().__init__('ahrs_publisher')
        self.publisher_magn = self.create_publisher(String, 'ahrs_magn', 10)
        self.publisher_gyro = self.create_publisher(String, 'ahrs_gyro', 10)
        self.publisher_accel = self.create_publisher(String, 'ahrs_accel', 10)
        timer_period = 0.5  # seconds ???
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg_magn = String()
        msg_gyro = String()
        msg_accel = String()
        measure = get_data()
        msg_magn.data = 'Magn: X: %s Y: %s Z: %s' % (measure.magn.x, measure.magn.y, measure.magn.z)
        msg_gyro.data = 'Gyro: X: %s Y: %s Z: %s' % (measure.gyro.x, measure.gyro.y, measure.gyro.z)
        msg_accel.data = 'Accel: X: %s Y: %s Z: %s' % (measure.accel.x, measure.accel.y, measure.accel.z)
        self.publisher_magn.publish(msg_magn)
        self.publisher_gyro.publish(msg_gyro)
        self.publisher_accel.publish(msg_accel)
        #self.get_logger().info('Publishing: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)

    ahrs_publisher = AhrsPublisher()

    rclpy.spin(ahrs_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ahrs_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
