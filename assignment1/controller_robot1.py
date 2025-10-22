import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time

class ControllerRobot1(Node):

    def __init__(self):
        super().__init__('controller_robot1')
        self.publisher_ = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.subscribtion = self.create_subscription(LaserScan, '/robot1/scan', self.scan_callback, 10)


        self.follow_distance = 0.6    
        self.lost_distance = 1.2      
        self.guest_timeout = 0.5      
        self.repulse_gain = 1.2       # tenho q variar estes parameteros de velocidade qnd tiver o wall following
        self.max_repulse = 0.30       
        self.front_stop_dist = 0.35   # var preciso de wall following para mudar o parametro
        self.cluster_distance_threshold = 0.3  # Max distance between points in a cluster
        self.robot_size_min = 0.03    #Isto e para cluster vals
        self.robot_size_max = 0.6   

        self.robot2_last_detect = 0.0
        self.robot2_distance = None

    def scan_callback(self, msg: LaserScan):
        cmd = Twist()
        clusters = self.find_clusters(msg)
        valid_clusters = self.filter_clusters(clusters)
        if valid_clusters:
            closest_cluster = min(valid_clusters, key=lambda c: c['avg_distance'])
            self.robot2_distance = closest_cluster['avg_distance']
            self.robot2_last_detect = time.time()
        else:
            self.robot2_distance = None

        self.control_robot(cmd)

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

       
        self.get_logger().info(f'Clusters detected: {len(clusters)}')
        return clusters
    
    def filter_clusters(self, clusters): # aqui preciso do wall following por causa da disparity distance
        valid_clusters = []
        for cluster in clusters:
            distances = [p['distance'] for p in cluster]
            min_distance = min(distances)
            max_distance = max(distances)

           
            if max_distance - min_distance > self.cluster_distance_threshold:
                self.get_logger().info(f'Discarded cluster due to large disparity: {max_distance - min_distance}')
                continue

          
            avg_distance = sum(distances) / len(distances)
            angle_span = abs(cluster[-1]['angle'] - cluster[0]['angle'])
            cluster_size = avg_distance * angle_span  

            self.get_logger().info(f'Cluster avg_distance: {avg_distance}, angle_span: {angle_span}, size: {cluster_size}')

          
            if angle_span > math.pi / 4:  # discard nas paredes
                self.get_logger().info(f'Discarded cluster due to wide angle span: {angle_span}')
                continue

          
            if self.robot_size_min <= cluster_size <= self.robot_size_max:
                valid_clusters.append({
                    'avg_distance': avg_distance,
                    'size': cluster_size,
                    'points': cluster
                })
            else:
                self.get_logger().info(f'Discarded cluster due to size: {cluster_size}')

      
        self.get_logger().info(f'Valid clusters: {len(valid_clusters)}')
        return valid_clusters

    def control_robot(self, cmd):
       
        robot2_age = time.time() - self.robot2_last_detect if self.robot2_last_detect else float('inf')
        robot2_missing = self.robot2_distance is None or robot2_age > self.guest_timeout
        robot2_too_far = self.robot2_distance is not None and self.robot2_distance > self.lost_distance

        if robot2_missing or robot2_too_far:
            cmd.linear.x = 0.0
            # self.get_logger().info('robot2 missing or too far')
        else:
           #dps meter aqui o wall following
            error = self.follow_distance - self.robot2_distance
            v = self.repulse_gain * error
            cmd.linear.x = max(0.0, min(v, self.max_repulse))
            # self.get_logger().info(f'robot1 moving forward: v={cmd.linear.x:.3f}')

        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    controller_robot1 = ControllerRobot1()
    rclpy.spin(controller_robot1)
    controller_robot1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()