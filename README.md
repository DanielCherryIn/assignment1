# Intelligent Robotics - Assignment 1
Group A1_Friday_30
- Daniel Cherry
- João Sequeira
- João Tomás Teixeira
- Luís Tavares

## Directory structure
```
├── assignment1                              <---- nodes and scripts
│   ├── base_lidar_controller.py
│   ├── controller_robot1.py
│   ├── controller_robot2.py
│   ├── __init__.py
│   ├── pid_controller.py
│   └── wall_follower_lidar_controller.py
├── launch                                   <---- ros2 launch files
│   └── robot_launch.py
├── resource                      
│   ├── assignment1
│   ├── ros2control.yml
│   ├── turtlebot1_webots.urdf
│   └── turtlebot2_webots.urdf
├── test
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
└── worlds 
    ├── tilde.obj
    ├── .tilde_world.wbproj
    ├── .tilde_world.jpg
    └── tilde_world.wbt
├── package.xml
├── README.md
├── setup.cfg
├── setup.py
```

## Requirements
- Ros2 Kilted/Humble
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

## To start simulation
```
ros2 launch assignment1 robot_launch.py
```
- wait until simulation is initialized, and until the commandline logs stop
- to start robot movement, open another commandline window and run:
```
ros2 topic pub /start_robots std_msgs/msg/Bool "data: true"
```



