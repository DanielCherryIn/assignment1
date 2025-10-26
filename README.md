# Intelligent Robotics - Assignment 1

## Requirements
- Ros2
- Webots
- webots_ros2 package

## Directory Structure
```
├── README.md                   # Project documentation
├── setup.py                     # Python package setup
├── setup.cfg                    # Python package configuration
├── package.xml                  # ROS 2 package manifest
├── assignment1                  # Main Python code for robot controllers
│   ├── base_lidar_controller.py         # Base class for LIDAR-based robot control
│   ├── controller_robot1.py             # Robot 1 specific controller
│   ├── controller_robot2.py             # Robot 2 specific controller
│   ├── pid.py                           # PID control implementation
│   ├── pid_controller.py                # PID controller class
│   └── wall_follower_lidar_controller.py  # Wall-following controller using LIDAR
├── launch                       # ROS 2 launch scripts
│   ├── robot_launch.py                 # Launches robots with custom controllers
│   └── robot_launch_og.py              # Original or alternate launch file
├── resource                     # Resource and configuration files
│   ├── assignment1/                     # Additional assignment-specific assets
│   ├── ros2control.yml                  # ROS 2 control configuration
│   ├── turtlebot1_webots.urdf          # TurtleBot 1 URDF model
│   └── turtlebot2_webots.urdf          # TurtleBot 2 URDF model
├── test                         # Unit and style tests
│   ├── test_copyright.py               # Copyright header checks
│   ├── test_flake8.py                  # Python style checks (Flake8)
│   └── test_pep257.py                  # Docstring convention checks
└── worlds                       # Webots simulation world files
    ├── tilde.obj                        # 3D model for the simulation
    └── tilde_world.wbt                  # Webots world with robots and environment
```

## To run simulation
- create 'ros2_ws' folder
- create 'src' folder indside 'ros2_ws'
- clone repository into 'src' folder
- in commandline run:
```
cd ros2_ws
colcon build
source  install/setup.bash
ros2 launch assignment1 robot_launch.py
```
- wait for launch to finish, then to start robot movement, run in another commandline window:
```
ros2 topic pub /start_robots std_msgs/msg/Bool "data: true"
```

## Useful commands
- Launch Keyboard teleoperation node (replace 'robot1' with 'robot2' for other robot)
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/robot1/cmd_vel
```
- Launch rviz
```
ros2 run rviz2 rviz2
```

## Idea for robot controllers
Wall follower (both robots)
- first test with constant linear velocity (v)
- pid controller for angular velocity (w)
   - proportional first until fast approach and oscillating, half value
   - integral to eliminate SS error
   - derivative to eliminate overshoot and oscillations
- use error between desired dist and distance from wall to eight
- distance from wall is lidar point to the right and slightly ahead (maybe average of several points?)
- if distance from wall is inf. (going around corner and no wall detected) set distance at max value and use PID OR hard coded turning movement until wall is found
- change v so that it oscillates sinusoidally around desired speed (for robot2)


Robot follower (robot1)
- assume robot2 is behind
- find cluster of points behind
- search for points where distance changes more than a threshold value, and stays similar for an angle that corresponds more or less the size of the robot (wont work because depends on distance from other robot, calculate estimated real size of cluster based on distance of those rays and the angle span)
   - step 1 - for laserscan ranges behind robot1 (maybe also a bit in front for when robot turns), find clusters of angles separated by sharp changes in distance (set distance variation threshold for cluster borders, convert inf. Values to high number). One of the clusters should be robot2
   - step 2 - discard clusters where there is a large disparity between its smallest and largest distance values (discard wall clusters; robot is small and will have similar range values for all points)
   - step 3 - for remaining clusters, calculate real size estimate based on the angle span and range values (arc length -  radius: avg cluster distance; angle: angle span). Discard clusters that aren't within expected values for size of turtlebot.
   - step 4 - if there are still multiple valid clusters, select the closest to robot1
   - step 5 - get minimum distance from the cluster ranges OR middle range value OR average distance value and use it as feedback for forward velocity (v) PID controller (error = setpoint - distance)


