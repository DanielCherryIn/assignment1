import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment1.wall_follower_lidar_controller import WallFollowerLidarController
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import Bool

import time

class ControllerRobot1(Node):

    def __init__(self):
        super().__init__('controller_robot1')
        self.publisher_ = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.subscribtion = self.create_subscription(LaserScan, '/robot1/scan', self.scan_callback, 10)
        self.start_sub = self.create_subscription(Bool, '/start_robots', self.start_callback, 10)

        self.started = False 

        self.wall_follower_lidar_controller = WallFollowerLidarController(0.25)

        self.follow_distance = 0.8
        self.lost_distance = 1.5
        self.guest_timeout = 1.0
        self.max_speed = 2.0

        self.cluster_distance_threshold = 0.1
        self.robot_size_min = 0.03
        self.robot_size_max = 0.6

        self.robot2_last_detect = 0.0
        self.robot2_distance = None

        self.last_robot2_distance = None
        self.robot2_last_update_time = None
        self.closing_speed = 0.0
        self.smoothing_factor = 0.4

        self.escape_threshold = 0.1     # aggressive
        self.adaptive_gain = 3.5

    def start_callback(self, msg):
        self.started = msg.data
        #if self.started:
        #    self.get_logger().info('Robot1 STARTED')
        #else:
        #    self.get_logger().info('Robot1 STOPPED')

    def scan_callback(self, msg: LaserScan):
        if not self.started:
            return
        cmd = Twist()
        clusters = self.find_clusters(msg)
        valid_clusters = self.filter_clusters(clusters)

        if valid_clusters:
            closest_cluster = min(valid_clusters, key=lambda c: c['avg_distance'])
            self.robot2_distance = closest_cluster['avg_distance']
            now = time.time()
            if self.last_robot2_distance is not None and self.robot2_last_update_time:
                dt = now - self.robot2_last_update_time
                if dt > 0:
                    raw_speed = (self.last_robot2_distance - self.robot2_distance) / dt
                    self.closing_speed = (1 - self.smoothing_factor) * self.closing_speed + self.smoothing_factor * raw_speed
            self.last_robot2_distance = self.robot2_distance
            self.robot2_last_update_time = now
            self.robot2_last_detect = now
        else:
            self.robot2_distance = None


        self.get_logger().info(
            f"SCAN: R2 Dist={self.robot2_distance if self.robot2_distance is not None else 'N/A'}, "
            f"Closing Speed={self.closing_speed:.3f} m/s, "
            f"Threshold={self.escape_threshold:.3f} m/s"
        )

        self.control_robot(cmd, msg)

    def find_clusters(self, msg):
        clusters = []
        current_cluster = []
        prev_distance = None

        sector_min_angle = -math.pi
        sector_max_angle = math.pi

        angle = msg.angle_min
        for r in msg.ranges:
            if r is None or math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue
            if not (sector_min_angle <= angle <= sector_max_angle):
                angle += msg.angle_increment
                continue
            if prev_distance is not None and abs(r - prev_distance) > self.cluster_distance_threshold:
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = []
            current_cluster.append({'distance': r, 'angle': angle})
            prev_distance = r
            angle += msg.angle_increment

        if current_cluster:
            clusters.append(current_cluster)
        return clusters

    def filter_clusters(self, clusters):
        valid_clusters = []
        for cluster in clusters:
            distances = [p['distance'] for p in cluster]
            min_distance = min(distances)
            max_distance = max(distances)

            if max_distance - min_distance > self.cluster_distance_threshold:
                continue

            avg_distance = sum(distances) / len(distances)
            angle_span = abs(cluster[-1]['angle'] - cluster[0]['angle'])
            cluster_size = avg_distance * angle_span

            if angle_span > math.pi / 4:
                continue

            if self.robot_size_min <= cluster_size <= self.robot_size_max:
                valid_clusters.append({
                    'avg_distance': avg_distance,
                    'size': cluster_size,
                    'points': cluster
                })

        return valid_clusters

    def control_robot(self, cmd, msg):
        robot2_age = time.time() - self.robot2_last_detect if self.robot2_last_detect else float('inf')
        robot2_missing = self.robot2_distance is None or robot2_age > self.guest_timeout
        robot2_too_far = self.robot2_distance is not None and self.robot2_distance > self.lost_distance

        if robot2_missing or robot2_too_far:
            cmd.linear.x = 0.0
            
        else:
            v_wall, w_wall = self.wall_follower_lidar_controller.compute_velocity(msg)

            if self.closing_speed > self.escape_threshold:
 
                self.get_logger().error(
                    f"!!! ESCAPE EMERGENCY TRIGGERED !!! "
                    f"Closing Speed ({self.closing_speed:.3f}) > Threshold ({self.escape_threshold:.3f}). "
                    f"Setting v_final={self.max_speed:.3f}"
                )
                # ---------------------------------------------------------------------------------
                adjusted_v = self.max_speed
            else:
                adjusted_v = v_wall + self.adaptive_gain * self.closing_speed

            adjusted_v = max(0.0, min(self.max_speed, adjusted_v))
     
            
            cmd.linear.x = adjusted_v
            cmd.angular.z = w_wall

        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    controller_robot1 = ControllerRobot1()
    rclpy.spin(controller_robot1)
    controller_robot1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()