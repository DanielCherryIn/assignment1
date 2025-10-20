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

from assignment1.base_lidar_controller import BaseLidarController

class WallFollowerLidarController(BaseLidarController):
    def compute_velocity(self, scan_msg: LaserScan) -> tuple[float, float]:
        linear_velocity = 0.2
        angular_velocity = 0.5

        return (linear_velocity, angular_velocity)

class ControllerRobot1(Node):
    def __init__(self, controllers: list[BaseLidarController]):
        super().__init__('controller_robot1')

        self.controllers = controllers

        self.publisher_ = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        self.subscribtion = self.create_subscription(LaserScan, '/robot1/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        cmd = TwistStamped()

        linear_sum = 0.0
        angular_sum = 0.0
        num_controllers = len(self.controllers)

        for controller in self.controllers:
            linear, angular = controller.compute_velocity(msg)
            linear_sum += linear
            angular_sum += angular

        if num_controllers > 0:
            linear_avg = linear_sum / num_controllers
            angular_avg = angular_sum / num_controllers
        else:
            linear_avg = 0.0
            angular_avg = 0.0
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear_avg
        cmd.twist.angular.z = angular_avg
        
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(cmd)
        self.get_logger().info('Robot1 Publishing - v:"%f" w:"%f"' % (cmd.twist.linear.x, cmd.twist.angular.z))


def main(args=None):
    rclpy.init(args=args)

    wall_follower = WallFollowerLidarController()

    controller_robot1 = ControllerRobot1(
        controllers=[wall_follower]
    )

    rclpy.spin(controller_robot1)

    controller_robot1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
