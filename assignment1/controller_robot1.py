import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from assignment1.wall_follower_lidar_controller import WallFollowerLidarController
from assignment1.pid_controller import PIDController
from sensor_msgs.msg import LaserScan
import math
from std_msgs.msg import Bool

class ControllerRobot1(Node):

    def __init__(self):
        super().__init__('controller_robot1')
        
        self.use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
        if self.use_twist_stamped:
            self.publisher_ = self.create_publisher(TwistStamped, '/robot1/cmd_vel', 10)
        else:
            self.publisher_ = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
            
        self.subscribtion = self.create_subscription(LaserScan, '/robot1/scan', self.scan_callback, 10)
        
        self.start_sub = self.create_subscription(Bool, '/start_robots', self.start_callback, 10)
        self.started = False 

        self.wall_follower_lidar_controller = WallFollowerLidarController(0.25)
    
        self.follow_distance = 0.5
        self.lost_distance = 1.5
        
        self.pid = PIDController(kp=1.5, ki=0.0, kd=0.0, setpoint=self.follow_distance)
        
        # Lidar point clustering parameters
        self.cluster_distance_threshold = 0.2
        self.robot_size_min = 0.01
        self.robot_size_max = 0.2

        self.robot2_distance = None

    def start_callback(self, msg):
        self.started = msg.data
        if self.started:
            self.get_logger().info('Robot1 STARTED')
        else:
            self.get_logger().info('Robot1 STOPPED')

    def scan_callback(self, msg: LaserScan):
        if not self.started:
            return
        
        clusters = self.find_clusters(msg)
        valid_clusters = self.filter_clusters(clusters, msg.angle_increment)

        if valid_clusters:
            closest_cluster = min(valid_clusters, key=lambda c: c['avg_distance'])
            self.robot2_distance = closest_cluster['avg_distance']
        else:
            self.robot2_distance = None


        self.get_logger().info(
            f"SCAN: R2 Dist={self.robot2_distance if self.robot2_distance is not None else 'N/A'}, "
        )

        self.control_robot(msg)

    def find_clusters(self, msg):
        clusters = []
        current_cluster = []
        prev_distance = None

        sector_min_angle = -math.pi
        sector_max_angle = math.pi
    
        angle = msg.angle_min
        for r in msg.ranges:
            # when there is a nan/inf, make a new cluster
            if r is None or math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = []
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
            
        # merge wrap-around clusters if they belong together
        if len(clusters) > 1:
            first_cluster = clusters[0]
            last_cluster = clusters[-1]
            first_dist = first_cluster[0]['distance']
            last_dist = last_cluster[-1]['distance']
            
            angle_diff = (abs((first_cluster[0]['angle'] - last_cluster[-1]['angle']) - 2*math.pi))
            
            '''
            self.get_logger().info(
                f"Merge: angle_diff={angle_diff}, dist={abs(first_dist-last_dist)}"
            )
            '''
            
            if abs(first_dist - last_dist) < self.cluster_distance_threshold and angle_diff <= abs(msg.angle_increment):
                merged_cluster = last_cluster + first_cluster
                clusters = [merged_cluster] + clusters[1:-1]
                '''
                self.get_logger().info(
                    "BORDER CLUSTERS MERGED"
                )
                '''

        
        return clusters

    def filter_clusters(self, clusters, angle_increment):
        valid_clusters = []
        for cluster in clusters:
            distances = [p['distance'] for p in cluster]
            min_distance = min(distances)
            max_distance = max(distances)
            avg_distance = sum(distances) / len(distances)
            angle_span = len(cluster)*abs(angle_increment)
            cluster_size = avg_distance * angle_span

            '''
            # LOGGING: Track cluster info
            self.get_logger().info(
                f"points: {len(cluster)}, "
                f"angles: {cluster[0]['angle']:.3f} > {cluster[-1]['angle']:.3f}, "
                f"min: {min_distance:.3f}, max: {max_distance:.3f}, "
                f"avg: {avg_distance:.3f}, angle_span: {angle_span:.3f}, "
                f"cluster_size: {cluster_size:.3f}"
            )
            '''

            # Filtering checks
            if max_distance - min_distance > self.cluster_distance_threshold or len(cluster) < 3:
                continue


            if self.robot_size_min <= cluster_size <= self.robot_size_max:
                valid_clusters.append({
                    'avg_distance': avg_distance,
                    'size': cluster_size,
                    'points': cluster
                })

        return valid_clusters


    def control_robot(self, msg):
        robot2_missing = self.robot2_distance is None
        robot2_too_far = self.robot2_distance is not None and self.robot2_distance > self.lost_distance

        if robot2_missing or robot2_too_far:
            v = 0.0
            w = 0.0
            
        else:
            v_wall, w_wall = self.wall_follower_lidar_controller.compute_velocity(msg)
            v = self.pid.compute(self.robot2_distance)
            w = w_wall

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


def main(args=None):
    rclpy.init(args=args)
    controller_robot1 = ControllerRobot1()
    rclpy.spin(controller_robot1)
    controller_robot1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()