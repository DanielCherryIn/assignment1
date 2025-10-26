import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from assignment1.wall_follower_lidar_controller import WallFollowerLidarController
import math


class ControllerRobot2(Node):
    def __init__(self):
        super().__init__('controller_robot2')

        # Wall-following controller
        self.wall_follower_lidar_controller = WallFollowerLidarController(0.25)

        # Publisher for velocity commands
        self.use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
        if self.use_twist_stamped:
            self.publisher_ = self.create_publisher(TwistStamped, '/robot2/cmd_vel', 10)
        else:
            self.publisher_ = self.create_publisher(Twist, '/robot2/cmd_vel', 10)

        # Subscriptions
        self.laser_sub = self.create_subscription(LaserScan, '/robot2/scan', self.scan_callback, 10)
        self.start_sub = self.create_subscription(Bool, '/start_robots', self.start_callback, 10)

        # State variables
        self.started = False
        
        # Sinusoidal speed parameters
        self.base_speed = 0.06
        self.speed_amplitude = 0.04 
        self.frequency = 0.5 

    def start_callback(self, msg):
        """Callback for the /start_robots topic."""
        self.started = msg.data
        if self.started:
            self.get_logger().info('Robot2 STARTED')
        else:
            self.get_logger().info('Robot2 STOPPED')

    def scan_callback(self, msg):
        """Callback for the laser scan data."""
        if not self.started:
            return

        # Wall-following logic
        v, w = self.wall_follower_lidar_controller.compute_velocity(msg)

        # Add sinusoidal speed variation
        time_elapsed = self.get_clock().now().nanoseconds * 1e-9
        v = self.base_speed + self.speed_amplitude * math.sin(2 * math.pi * self.frequency * time_elapsed)
        
        # Publish the velocity commands
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
        #self.get_logger().info(f'Robot2 Publishing - v: {v:.3f}, w: {w:.3f}')


def main(args=None):
    rclpy.init(args=args)

    controller_robot2 = ControllerRobot2()

    rclpy.spin(controller_robot2)

    controller_robot2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()