# ...existing code...
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from sensor_msgs.msg import LaserScan
import math

class ControllerRobot1(Node):

    def __init__(self):
        super().__init__('controller_robot1')
        # publish plain Twist (robot base expects geometry_msgs/msg/Twist)
        self.publisher_ = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.subscribtion = self.create_subscription(LaserScan, '/robot1/scan', self.scan_callback, 10)

        # monitor robot2 scan to detect follower approaching
        self.sub_robot2 = self.create_subscription(LaserScan, '/robot2/scan', self.robot2_scan, 10)
        self.robot2_min_range = None
        self.robot2_lastDetect = 0.0

        self.prev_robot2_mia = None
        self.prev_robot2_oor = None

        # Tunable parameters
        self.follow_distance = 0.6    # desired gap to keep (m)
        self.lost_distance = 1.2      # if guest reports > this, leader should stop and wait
        self.guest_timeout = 0.5      # seconds without guest scan -> consider lost

        # Repulsion control (robot1 linear caused by robot2)
        self.repulse_gain = 1.2       # k in v = k*(follow_distance - guest_range)
        self.max_repulse = 0.30       # max forward speed caused by repulsion (>= follower speed)
        self.front_stop_dist = 0.35   # if front obstacle closer than this, stop immediately

    def robot2_scan(self, msg: LaserScan):
        # use robot2's forward sector to detect robot1 (when follower approaches leader)
        angle = msg.angle_min
        seen = []
        forward_half_angle = math.pi / 3.0  # 60 degrees forward window
        for r in msg.ranges:
            # guard invalid readings
            if r is None or math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue
            if abs(angle) <= forward_half_angle and r > 0.0:
                seen.append(r)
            angle += msg.angle_increment

        if seen:
            self.robot2_min_range = min(seen)
            self.robot2_lastDetect = time.time()
            #self.get_logger().debug('Robot1 Scan - robot2 forward min range: %.3f' % self.robot2_min_range)
        #else:
            # keep previous value; timeout logic will mark as lost
            #self.get_logger().debug('Robot1 Scan - robot2 not detected (forward sector)')

    def scan_callback(self, msg: LaserScan):
        cmd = Twist()

        # split scan into front (for collision) and side (for wall following)
        angle = msg.angle_min
        forward_half_angle = math.pi / 6.0  # 30 degrees forward window
        front_vals = []
        side_vals = []
        for r in msg.ranges:
            if r is None or math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue
            if abs(angle) <= forward_half_angle:
                if r > 0.0:
                    front_vals.append(r)
            else:
                if r > 0.0:
                    side_vals.append(r)
            angle += msg.angle_increment

        front_min = min(front_vals) if front_vals else float('inf')
        side_min = min(side_vals) if side_vals else float('inf')

        #self.get_logger().debug('Robot1 Scan - front_min: %.3f side_min: %.3f' % (front_min, side_min))

        # orientation to wall: use side_min so side walls/corners drive angular correction
        if side_min < 0.3:
            cmd.angular.z = 0.3
        else:
            cmd.angular.z = 0.0

        # front collision: only stop if something is actually in front
        #if front_min < self.front_stop_dist:
        #    self.get_logger().info('robot1 front collision stop (front_min=%.2f)' % front_min)
        #    cmd.linear.x = 0.0
        #    cmd.angular.z = 0.0
        #    self.publisher_.publish(cmd)
        #    return

        # repulsion / push behavior based on robot2 detection
        robot2_age = time.time() - self.robot2_lastDetect if self.robot2_lastDetect else float('inf')
        robot2_mia = (self.robot2_min_range is None) or (robot2_age > self.guest_timeout)
        robot2_oor = (self.robot2_min_range is not None) and (self.robot2_min_range > self.lost_distance)

        # Log only if the state of mia or oor changes
        if robot2_mia != self.prev_robot2_mia or robot2_oor != self.prev_robot2_oor:
            self.get_logger().debug('robot1: robot2 missing/too_far (mia=%s oor=%s age=%.2f)' %
                                    (robot2_mia, robot2_oor, robot2_age))
            self.prev_robot2_mia = robot2_mia
            self.prev_robot2_oor = robot2_oor

        if robot2_mia or robot2_oor:
            cmd.linear.x = 0.0
        else:
            if self.robot2_min_range < self.follow_distance:
                v = self.repulse_gain * (self.follow_distance - self.robot2_min_range)
                v = max(0.0, min(v, self.max_repulse))
                cmd.linear.x = v
                self.get_logger().info('robot1: approaching robot2, moving forward at %.3f' % v)
            else:
                cmd.linear.x = 0.0

        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    controller_robot1 = ControllerRobot1()

    rclpy.spin(controller_robot1)

    controller_robot1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# ...existing code...