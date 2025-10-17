# Intelligent Robotics - Assignment 1

## Requirements
- Ros2
- Webots
- webots_ros2 package

## To run simulation
- create 'ros2_ws' folder
- create 'src' folder indside 'ros2_ws'
- clone repository into 'src' folder
- in commandline run:
```
colcon build
source  install/setup.bash
ros2 launch assignment1 robot_launch.py
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
