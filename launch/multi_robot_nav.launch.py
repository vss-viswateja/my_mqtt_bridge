# Copyright (c) 2024 - for information on the respective copyright owner
# see the NOTICE file or the repository https://github.com/boschresearch/remroc/.
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

"""
Simplified Multi-Robot Navigation Launch File

This launch file spawns multiple robots in Gazebo and makes them navigation-ready.
Robots can be commanded via Nav2 action interface:
  ros2 action send_goal /<robot_name>/navigate_to_pose nav2_msgs/action/NavigateToPose ...

Or via goal pose topic:
  ros2 topic pub /<robot_name>/goal_pose geometry_msgs/msg/PoseStamped ...
"""

import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, RegisterEventHandler, GroupAction, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


# ============================================================================
# CONFIGURATION - Modify these settings for your application
# ============================================================================

# Define your robots here: [id, type, x, y, yaw]
# Each robot will be named as: <type>_<id>
ROBOTS = [
    {'id': 0, 'type': 'SmallDeliveryRobot', 'x': 3.635, 'y': 4.0, 'yaw': -1.57},
    {'id': 1, 'type': 'SmallDeliveryRobot', 'x': 2.175, 'y': 4.0, 'yaw': -1.57},
    {'id': 2, 'type': 'SmallDeliveryRobot', 'x': -2.175, 'y': 4.0, 'yaw': -1.57},
    {'id': 3, 'type': 'SmallDeliveryRobot', 'x': -3.635, 'y': 4.0, 'yaw': -1.57},
]

# World configuration
world_name_default = 'simple'           # The world name (simple or depot)
map_name_default = 'simple'             # The map YAML file name (e.g., simple.yaml)
number_of_humans_default = '0'          # Number of humans in the world (0 for empty world)
sample_default = '0'                    # Sample number for world variations

# ============================================================================


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use simulation time'),
    DeclareLaunchArgument('world', default_value=world_name_default,
                          description='World name (simple or depot)'),
    DeclareLaunchArgument('map', default_value=map_name_default,
                          description='Map file name (without extension)'),
    DeclareLaunchArgument('number_of_humans', default_value=number_of_humans_default,
                          description='Number of humans in the world (0, 5, 10, 20)'),
    DeclareLaunchArgument('sample', default_value=sample_default,
                          description='World sample number (0-19)'),
]


def generate_launch_description():

    # Launch configuration variables
    world_name = LaunchConfiguration('world')
    map_name = LaunchConfiguration('map')
    number_of_humans = LaunchConfiguration('number_of_humans')
    sample = LaunchConfiguration('sample')

    # Package paths
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_remroc = get_package_share_directory('remroc')
    pkg_remroc_robots = get_package_share_directory('remroc_robots')

    # Set Ignition resource path for robot models
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            Path(pkg_remroc).joinpath('worlds').as_posix() + ':' + 
            Path(pkg_remroc_robots).joinpath('robots').as_posix()
        ])

    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])

    # Launch Ignition Gazebo
    # World file format: worldname_numhumans_sample.sdf (e.g., simple_0_0.sdf)
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('gz_args', [
                'sdfs/', world_name, '_', number_of_humans, '_', sample, '.sdf',  # World file path
                ' -v 4',             # Verbose mode
                ' -r',               # Start unpaused
                ' --headless-rendering',  # Better performance
            ])
        ]
    )

    # Initialize launch description
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ignition_gazebo)

    # Bridge Ignition clock to ROS2 clock
    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge_clock',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                ['/world/', world_name, '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            ],
            remappings=[
                (['/world/', world_name, '/clock'], '/clock'),
            ],
        )
    )

    # Launch map server
    ld.add_action(
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'yaml_filename': [Path(pkg_remroc).joinpath('worlds', 'maps').as_posix(), '/', map_name, '.yaml']}
            ]
        )
    )

    # Lifecycle manager for map server
    ld.add_action(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapserver',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'node_names': ['map_server']},
                {'autostart': True}
            ]
        )
    )

    # ========================================================================
    # Launch nodes for each robot
    # ========================================================================
    
    for robot in ROBOTS:
        robot_id = robot['id']
        robot_type = robot['type']
        robot_x = robot['x']
        robot_y = robot['y']
        robot_yaw = robot['yaw']
        robot_name = f'{robot_type}_{robot_id}'

        # TF remappings for namespaced robots
        remappings = [
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]

        # ------------------------------------------------------------------------
        # Spawn robot in Gazebo
        # ------------------------------------------------------------------------
        robot_spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_name}',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                '-world', world_name,
                '-file', Path(pkg_remroc_robots).joinpath('robots', robot_type, 'model.sdf').as_posix(),
                '-name', robot_name,
                '-x', str(robot_x),
                '-y', str(robot_y),
                '-Y', str(robot_yaw)
            ]
        )
        ld.add_action(robot_spawn_node)

        # ------------------------------------------------------------------------
        # Setup sensor transforms
        # ------------------------------------------------------------------------
        with open(Path(pkg_remroc_robots).joinpath('config', robot_type, 'sensor_positions.yaml')) as file:
            sensor_positions = yaml.safe_load(file)

        for sensor_dict in sensor_positions.values():
            ld.add_action(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    namespace=robot_name,
                    output='screen',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    arguments=[
                        '--x', sensor_dict['x'],
                        '--y', sensor_dict['y'],
                        '--z', sensor_dict['z'],
                        '--yaw', sensor_dict['yaw'],
                        '--pitch', sensor_dict['pitch'],
                        '--roll', sensor_dict['roll'],
                        '--frame-id', 'base_link',
                        '--child-frame-id', f'{robot_name}/{sensor_dict["sensor_frame_id"]}',
                    ],
                    remappings=remappings
                )
            )

        # ------------------------------------------------------------------------
        # ROS-Gazebo bridge for sensors and control
        # ------------------------------------------------------------------------
        ld.add_action(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=robot_name,
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                arguments=[
                    ['/model/', robot_name, '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
                    ['/model/', robot_name, '/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
                    ['/world/', world_name, '/model/', robot_name, '/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
                    ['/world/', world_name, '/model/', robot_name, '/link/scan_omni/sensor/scan_omni/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                    ['/world/', world_name, '/model/', robot_name, '/link/scan_omni/sensor/scan_omni/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
                ],
                remappings=[
                    (['/model/', robot_name, '/cmd_vel'], ['/', robot_name, '/cmd_vel']),
                    (['/model/', robot_name, '/odometry'], ['/', robot_name, '/odometry']),
                    (['/world/', world_name, '/model/', robot_name, '/link/imu_link/sensor/imu/imu'], ['/', robot_name, '/imu']),
                    (['/world/', world_name, '/model/', robot_name, '/link/scan_omni/sensor/scan_omni/scan'], ['/', robot_name, '/laser_scan']),
                    (['/world/', world_name, '/model/', robot_name, '/link/scan_omni/sensor/scan_omni/scan/points'], ['/', robot_name, '/point_cloud']),
                ],
            )
        )

        # ------------------------------------------------------------------------
        # Navigation stack for this robot
        # ------------------------------------------------------------------------
        navigation_nodes = GroupAction(
            actions=[
                # EKF for state estimation
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    namespace=robot_name,
                    respawn=True,
                    respawn_delay=2.0,
                    name='ekf_node',
                    output='screen',
                    parameters=[
                        Path(pkg_remroc_robots).joinpath('config', robot_type, 'ekf.yaml').as_posix(),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=remappings
                ),
                
                # AMCL for localization
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    namespace=robot_name,
                    respawn=True,
                    respawn_delay=2.0,
                    name='amcl_node',
                    output='screen',
                    parameters=[
                        Path(pkg_remroc_robots).joinpath('config', robot_type, 'amcl.yaml').as_posix(),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'set_initial_pose': True},
                        {'initial_pose': {'x': robot_x, 'y': robot_y, 'yaw': robot_yaw}}
                    ],
                    remappings=remappings
                ),
                
                # Controller server (MPPI)
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    namespace=robot_name,
                    name='controller_server_node',
                    respawn=True,
                    respawn_delay=2.0,
                    output='screen',
                    parameters=[
                        Path(pkg_remroc_robots).joinpath('config', robot_type, 'controller_server.yaml').as_posix(),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    ],
                    remappings=remappings + [
                        ('/laser_scan', f'/{robot_name}/laser_scan'),
                        ('/point_cloud', f'/{robot_name}/point_cloud'),
                        ('/trajectories', f'/{robot_name}/trajectories'),
                    ]
                ),
                
                # Planner server (Hybrid A*)
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    namespace=robot_name,
                    name='planner_server_node',
                    respawn=True,
                    respawn_delay=2.0,
                    output='screen',
                    parameters=[
                        Path(pkg_remroc_robots).joinpath('config', robot_type, 'planner_server.yaml').as_posix(),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    ],
                    remappings=remappings + [
                        ('/laser_scan', f'/{robot_name}/laser_scan'),
                        ('/point_cloud', f'/{robot_name}/point_cloud'),
                    ]
                ),
                
                # Behavior server
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server_node',
                    namespace=robot_name,
                    respawn=True,
                    respawn_delay=2.0,
                    output='screen',
                    parameters=[
                        Path(pkg_remroc_robots).joinpath('config', robot_type, 'behavior_server.yaml').as_posix(),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    ],
                    remappings=remappings
                ),
                
                # Behavior tree navigator
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    namespace=robot_name,
                    name='bt_navigator_node',
                    respawn=True,
                    respawn_delay=2.0,
                    output='screen',
                    parameters=[
                        Path(pkg_remroc_robots).joinpath('config', robot_type, 'bt_navigator.yaml').as_posix(),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ],
                    remappings=remappings
                ),
                
                # Lifecycle manager for navigation nodes
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    namespace=robot_name,
                    name='lifecycle_manager_node',
                    output='screen',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'node_names': ['amcl_node', 'controller_server_node', 'planner_server_node', 
                                       'behavior_server_node', 'bt_navigator_node']},
                        {'autostart': True}
                    ],
                ),
            ]
        )

        # Robot state publisher (republishes odometry in map frame)
        robot_state_publisher_node = TimerAction(
            actions=[
                Node(
                    package='remroc',
                    executable='robot_state_publisher',
                    namespace=robot_name,
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    remappings=remappings,
                )
            ],
            period=5.0
        )

        # Launch navigation stack after robot is spawned
        launch_nav_after_spawn = RegisterEventHandler(
            OnProcessExit(
                target_action=robot_spawn_node,
                on_exit=TimerAction(
                    actions=[navigation_nodes, robot_state_publisher_node],
                    period=1.0
                )
            )
        )
        ld.add_action(launch_nav_after_spawn)

    return ld
