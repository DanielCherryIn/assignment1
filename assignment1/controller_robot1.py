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
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.min_robot_distance = 1.2    # Minimum acceptable distance to robot behind
        self.obstacle_threshold = 0.25   # Emergency stop threshold

        # ========== VISUAL TEST MARKER ==========
        print("\n" + "="*60)
        print("🤖 ROBOT 1 CONTROLLER LOADED SUCCESSFULLY! 🤖")
        print("="*60 + "\n")
        self.get_logger().info('⭐⭐⭐ ROBOT1 IS ALIVE AND RUNNING! ⭐⭐⭐')
        # ========================================


    def scan_callback(self, msg):
        cmd = TwistStamped()

        ranges = msg.ranges

        # Front: detect obstacles ahead
        front_ranges = ranges[0:30] + ranges[-30:]
        valid_front = [r for r in front_ranges if r > 0.1 and r < 10.0]
        front_min = min(valid_front) if valid_front else 10.0
        
        # Rear: detect Robot2 behind 
        rear_ranges = ranges[150:210]
        valid_rear = [r for r in rear_ranges if r > 0.1 and r < 10.0]
        rear_min = min(valid_rear) if valid_rear else 10.0

        self.get_logger().info(f'🔍 FRONT: {front_min:.2f}m | REAR: {rear_min:.2f}m | Total ranges: {len(ranges)}')

        if front_min < self.obstacle_threshold:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.5
            self.get_logger().info('Obstacle detected! Turning...')

        elif rear_min > self.min_robot_distance:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.get_logger().info('Waiting for Robot2 (distance: %.2f m)' % rear_min)

        else:
            cmd.twist.linear.x = 0.15
            cmd.twist.angular.z = 0.0
            self.get_logger().info('Moving forward | Robot2 distance: %.2f m' % rear_min)
    
        
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
