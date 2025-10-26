#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots TurtleBot3 Burger driver."""

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
import yaml
import tempfile


def generate_launch_description():
    package_dir = get_package_share_directory('assignment1')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    robot1_description_path = os.path.join(package_dir, 'resource', 'turtlebot1_webots.urdf')
    robot2_description_path = os.path.join(package_dir, 'resource', 'turtlebot2_webots.urdf')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        mode=mode,
        ros2_supervisor=True
    )

    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace="robot1",
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
            'frame_prefix': "robot1/"
        }],
    )
    
    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace="robot2",
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
            'frame_prefix': "robot2/"
        }],
    )   
    
    # TF publishers to have relative robot positions in a map frame
    static_tf_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.5', '0.5', '0', '0', '0', '0', 'map', 'robot1/odom']
    )

    static_tf_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.85', '0.5', '0', '0', '0', '0', 'map', 'robot2/odom']
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner1 = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
        namespace="robot1"
    )
    joint_state_broadcaster_spawner1 = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        namespace="robot1"
    )
    ros_control_spawners1 = [diffdrive_controller_spawner1, joint_state_broadcaster_spawner1]
    
    diffdrive_controller_spawner2 = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
        namespace="robot2"
    )
    joint_state_broadcaster_spawner2 = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
        namespace="robot2"
    )
    ros_control_spawners2 = [diffdrive_controller_spawner2, joint_state_broadcaster_spawner2]

    # Load and namespace ros2_control parameters
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    def _create_namespaced_params_file(src, namespace):
        with open(src, 'r') as f:
            data = yaml.safe_load(f) or {}
        namespaced = {}
        for key, val in data.items():
            key_name = key.lstrip('/')
            namespaced[f'/{namespace}/{key_name}'] = val
        tmp = tempfile.NamedTemporaryFile(delete=False, suffix='.yml', mode='w')
        yaml.safe_dump(namespaced, tmp)
        tmp.close()
        return tmp.name
    
    ros2_control_params1 = _create_namespaced_params_file(ros2_control_params, 'robot1')
    ros2_control_params2 = _create_namespaced_params_file(ros2_control_params, 'robot2')

    # Remove diffdrive_controller prefix and set remappings based on ROS distro
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['rolling', 'jazzy', 'kilted'])
    if use_twist_stamped:
        mappings = [
            ('diffdrive_controller/cmd_vel', 'cmd_vel'),
            ('diffdrive_controller/odom', 'odom')
        ]
    else:
        mappings = [
            ('diffdrive_controller/cmd_vel_unstamped', 'cmd_vel'),
            ('diffdrive_controller/odom', 'odom')
        ]

    # Add these remappings for robot1
    robot1_remappings = [
        ('Robot1/compass/bearing', 'compass/bearing'),
        ('Robot1/compass/north_vector', 'compass/north_vector'),
        ('/imu', 'imu'),
        ('/scan', 'scan'),
        ('/scan/point_cloud', 'scan/point_cloud')
    ] + mappings
    
    # Add these remappings for robot2
    robot2_remappings = [
        ('Robot2/compass/bearing', 'compass/bearing'),
        ('Robot2/compass/north_vector', 'compass/north_vector'),
        ('/imu', 'imu'),
        ('/scan', 'scan'),
        ('/scan/point_cloud', 'scan/point_cloud')
    ] + mappings

    # TurtleBot3 Webots controllers
    turtlebot_driver1 = WebotsController(
        robot_name='Robot1',
        parameters=[
            {'robot_description': robot1_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params1
        ],
        namespace="robot1",
        remappings=robot1_remappings,
        respawn=True
    )
    
    turtlebot_driver2 = WebotsController(
        robot_name='Robot2',
        parameters=[
            {'robot_description': robot2_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params2
        ],
        namespace="robot2",
        remappings=robot2_remappings,
        respawn=True
    )
    
    ## ADD MORE NODES IF NEEDED ##
    controller_robot1 = Node(
        package='assignment1',
        executable='controller_robot1',
        name='controller_robot1'
    )
    
    controller_robot2 = Node(
        package='assignment1',
        executable='controller_robot2',
        name='controller_robot2'
    )

    # Wait for the simulation to be ready to start spawner nodes
    waiting_nodes1 = WaitForControllerConnection(
        target_driver=turtlebot_driver1,
        nodes_to_start=ros_control_spawners1
    )
    waiting_nodes2 = WaitForControllerConnection(
        target_driver=turtlebot_driver2,
        nodes_to_start=ros_control_spawners2
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='tilde_world.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        webots,
        webots._supervisor,

        robot_state_publisher1,
        robot_state_publisher2,
        
        static_tf_robot1,
        static_tf_robot2,

        turtlebot_driver1,
        turtlebot_driver2,
        waiting_nodes1,
        waiting_nodes2,
        
        controller_robot1,
        controller_robot2,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        ),
    ])
