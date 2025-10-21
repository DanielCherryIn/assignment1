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
import math

class ControllerRobot2(Node):

    def __init__(self):
        super().__init__('controller_robot2')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        # Timer for sinusoidal speed
        self.start_time = self.get_clock().now()

        # Speed parameters
        self.base_speed = 0.15
        self.speed_amplitude = 0.08  
        self.frequency = 0.5 

        # ========== VISUAL TEST MARKER ==========
        print("\n" + "="*60)
        print("🎯 ROBOT 2 CONTROLLER LOADED SUCCESSFULLY! 🎯")
        print("="*60 + "\n")
        self.get_logger().info('⭐⭐⭐ ROBOT2 WITH SINUSOIDAL SPEED! ⭐⭐⭐')
        # ========================================

    def scan_callback(self, msg):
        cmd = TwistStamped()

        # Calculate sinusoidal speed
        current_time = self.get_clock().now()
        elapsed_sec = (current_time - self.start_time).nanoseconds / 1e9
        linear_speed = self.base_speed + self.speed_amplitude * math.sin(2 * math.pi * self.frequency * elapsed_sec)
        
        ranges = msg.ranges
        front_ranges = ranges[0:30] + ranges[-30:]  # Front 60 degrees
        valid_ranges = [r for r in front_ranges if r > 0.1 and r < 10.0]  # Filter valid readings
    
        if valid_ranges:
            min_range = min(valid_ranges)  
        else:
            min_range = 10.0 

        if min_range < 0.3:  
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.5
            self.get_logger().info('🚨 Robot2: Obstacle at %.2fm! Turning...' % min_range)
        else:
            cmd.twist.linear.x = linear_speed
            cmd.twist.angular.z = 0.0
            self.get_logger().info('🌊 Robot2: SINE WAVE speed=%.3f m/s | Front: %.2fm' % (linear_speed, min_range))
        
        # Publish desired speeds
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(cmd)
        self.get_logger().info('Robot2 Publishing - v:"%f" w:"%f"' % (cmd.twist.linear.x, cmd.twist.angular.z))


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
