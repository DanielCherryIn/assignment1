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

import os
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class ControllerRobot2(Node):

    def __init__(self):
        super().__init__('controller_robot2')
        
        self.use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
        if self.use_twist_stamped:
            self.publisher_ = self.create_publisher(TwistStamped, '/robot2/cmd_vel', 10)
        else:
            self.publisher_ = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        
        self.subscribtion = self.create_subscription(LaserScan, '/robot2/scan', self.scan_callback, 10)

    def scan_callback(self, msg):        
        # Calculate desired speeds based on laserscan
        min_range = min(msg.ranges)
        if min_range < 0.5:
            v = 0.0
            w = 0.5
        else:
            v = 0.15
            w = 0.0
        
        # Publish desired speeds
        if self.use_twist_stamped:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.twist.linear.x = v
            cmd.twist.angular.z = w
            self.publisher_.publish(cmd)
        else:
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.publisher_.publish(cmd)
            
        self.get_logger().info('Robot2 Publishing - v:"%f" w:"%f"' % (v, w))


def main(args=None):
    rclpy.init(args=args)

    controller_robot2 = ControllerRobot2()

    rclpy.spin(controller_robot2)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_robot2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
