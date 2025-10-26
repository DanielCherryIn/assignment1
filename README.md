# Intelligent Robotics - Assignment 1
Group A1_Friday_30
- Daniel Cherry
- João Sequeira
- João Tomás Teixeira
- Luís Tavares

## Directory structure
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
│   ├── assignment1/                 
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

## Requirements
- Ros2 Kilted/Humble (other versions may work too)
- Webots 2025
- webots_ros2 package

## To compile
- create ros2 workspace folder if not present
- create 'src' folder indside workspace folder
- clone repository into 'src' folder
- in commandline, change directory to workspace folder and run:
```
colcon build
source install/setup.bash
```
## To start simulation
```
ros2 launch assignment1 robot_launch.py
```
- wait until simulation is initialized, and until the commandline logs stop
- to start robot movement, open another commandline window and run:
```
ros2 topic pub /start_robots std_msgs/msg/Bool "data: true"
```



