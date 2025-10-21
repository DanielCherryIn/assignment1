import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math

class PIDController:
    """
    A simple PID controller class for Python.
    """
    def __init__(self, Kp, Ki, Kd, output_limits=None, sample_time=0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limits = output_limits if output_limits else (None, None)
        self.sample_time = sample_time

        self.last_time = time.time()
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, current_value, setpoint):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt < self.sample_time:
            return None 
            
        error = setpoint - current_value

        P_out = self.Kp * error

        self.integral += error * dt
        
        derivative = (error - self.previous_error) / dt

        D_out = self.Kd * derivative

        output = P_out + (self.Ki * self.integral) + D_out

        min_output, max_output = self.output_limits
        if min_output is not None and output < min_output:
            output = min_output
            if self.Ki != 0 and (output < max_output):
                self.integral -= error * dt 
        elif max_output is not None and output > max_output:
            output = max_output
            if self.Ki != 0 and (output > min_output):
                self.integral -= error * dt 

        # Update state
        self.previous_error = error
        self.last_time = current_time
        
        return output
