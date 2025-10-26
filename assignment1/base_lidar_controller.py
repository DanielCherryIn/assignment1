from abc import ABC, abstractmethod
from sensor_msgs.msg import LaserScan

class BaseLidarController(ABC):

    @abstractmethod
    def compute_velocity(self, scan_msg: LaserScan) -> tuple[float, float]:
        """
        Process LIDAR and return (linear_velocity, angular_velocity)
        """
        pass
