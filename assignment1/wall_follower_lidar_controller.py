import math
from sensor_msgs.msg import LaserScan
from assignment1.base_lidar_controller import BaseLidarController
from assignment1.pid_controller import PIDController


class WallFollowerLidarController(BaseLidarController):
    def __init__(self, desired_distance=0.5):
        self.desired_distance = desired_distance

        self.pid = PIDController(kp=6.0, ki=0.0, kd=0, setpoint=desired_distance)

    def compute_velocity(self, scan_msg: LaserScan) -> tuple[float, float]:
        right_indices = range(230,250)
        valid_readings = [
            scan_msg.ranges[i] for i in right_indices
            if i < len(scan_msg.ranges)
            and not math.isinf(scan_msg.ranges[i])
            and not math.isnan(scan_msg.ranges[i])
        ]

        if valid_readings:
            right_distance = sum(valid_readings) / len(valid_readings)
            angular_velocity = self.pid.compute(right_distance)
        else:
            #TODO maybe make proportional to linear_velocity? (take input from previously caculated linear_velocity from either robot follower or sine velocity calc)
            angular_velocity = -1.0

        linear_velocity = 0.1

        return (linear_velocity, angular_velocity)