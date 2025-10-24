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

from std_msgs.msg import Bool 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan

from assignment1.wall_follower_lidar_controller import WallFollowerLidarController

class ControllerRobot1(Node):
    def __init__(self):
        super().__init__('controller_robot1')
        
        self.wall_follower_lidar_controller = WallFollowerLidarController(0.25)
        
        self.use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
        if self.use_twist_stamped:
            self.publisher_ = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        else:
            self.publisher_ = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
            
        self.laser_sub = self.create_subscription(LaserScan, '/robot1/scan', self.scan_callback, 10)
        self.start_sub = self.create_subscription(Bool, '/start_robots', self.start_callback, 10)
        self.started = False
        
    def start_callback(self, msg):
        self.started = msg.data
        if self.started:
            self.get_logger().info('Robots STARTED')
        else:
            self.get_logger().info('Robots STOPPED')
        
    def scan_callback(self, msg):
        if not self.started:
            return
        
        # Calculate desired speeds based on laserscan
        v, w = self.wall_follower_lidar_controller.compute_velocity(msg)
        
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
            
        self.get_logger().info('Robot1 Publishing - v:"%f" w:"%f"' % (v, w))


def main(args=None):
    rclpy.init(args=args)

    controller_robot1 = ControllerRobot1()
    
    rclpy.spin(controller_robot1)

    controller_robot1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
