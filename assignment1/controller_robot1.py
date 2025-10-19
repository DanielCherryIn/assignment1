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
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

class ControllerRobot1(Node):

    def __init__(self):
        super().__init__('controller_robot1')
        self.publisher_ = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        self.subscribtion = self.create_subscription(LaserScan, '/robot1/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        cmd = TwistStamped()
        
        # Calculate desired speeds based on laserscan
        min_range = min(msg.ranges)
        if min_range < 0.5:
            cmd.twist.angular.z = 0.5
        else:
            cmd.twist.linear.x = 0.15
        
        # Publish desired speeds
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(cmd)
        self.get_logger().info('Robot1 Publishing - v:"%f" w:"%f"' % (cmd.twist.linear.x, cmd.twist.angular.z))


def main(args=None):
    rclpy.init(args=args)

    controller_robot1 = ControllerRobot1()

    rclpy.spin(controller_robot1)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_robot1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
