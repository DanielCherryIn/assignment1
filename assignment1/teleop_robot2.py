import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class TeleopRobot2(Node):

    def __init__(self):
        super().__init__('teleop_robot2')
        self.publisher_ = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        self.get_logger().info("Use W/A/S/D keys to control robot2. Press Q to quit.")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):

        move_bindings = {
            'w': (0.2, 0.0),  
            's': (-0.2, 0.0), 
            'a': (0.0, 0.5),  
            'd': (0.0, -0.5), 
        }

        try:
            while True:
                key = self.get_key()
                if key == 'q':  
                    break

                cmd = Twist()
                if key in move_bindings:
                    cmd.linear.x, cmd.angular.z = move_bindings[key]
                else:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                self.publisher_.publish(cmd)
                self.get_logger().info(f"Publishing: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

        finally:
           
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    teleop_robot2 = TeleopRobot2()
    teleop_robot2.run()
    teleop_robot2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()