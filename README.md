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
├── assignment1              
│   ├── base_lidar_controller.py         # Base class for LIDAR-based robot control
│   ├── controller_robot1.py             # Robot 1 specific controller
│   ├── controller_robot2.py             # Robot 2 specific controller
│   ├── pid.py                           # PID control implementation
│   ├── pid_controller.py                # PID controller class
│   └── wall_follower_lidar_controller.py  # Wall-following controller using LIDAR
├── launch                    
│   ├── robot_launch.py                 # Launches robots with custom controllers
│   └── robot_launch_og.py              # Original or alternate launch file
├── resource                     # Resource and configuration files
│   ├── assignment1/                     # Additional assignment-specific assets
│   ├── ros2control.yml                  # ROS 2 control configuration
│   ├── turtlebot1_webots.urdf          # TurtleBot 1 URDF model
│   └── turtlebot2_webots.urdf          # TurtleBot 2 URDF model
├── test                        
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
