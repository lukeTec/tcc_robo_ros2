#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_local = get_package_share_directory('meu_pacote_navegacao')
    pkg_slam = get_package_share_directory('meu_pacote_slam')

    # Caminhos
    map_yaml = '/home/robo/ros2_ws/maps/meu_mapa_final.yaml'
    nav2_params = '/home/robo/ros2_ws/src/meu_pacote_navegacao/config/nav2_params.yaml'
    ekf_params = os.path.join(pkg_local, 'config', 'ekf_params.yaml')

    # Arg use_sim_time para consistência
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # TF estático base_link -> laser (ajuste z conforme seu hardware)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0.30', '0.30', '0.43', '0', '0', '0', 'base_link', 'laser']
    )

    # EKF (publica odom->base_link)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params],
    )

    # RPLIDAR (usa seu launch existente)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'rplidar.launch.py')
        )
    )

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml}]
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )

    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[
          ('/cmd_vel', '/cmd_vel_raw')
        ]
    )

    # Smoother server
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Velocity smoother (se você usa)
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
        remappings=[
          ('/cmd_vel', '/cmd_vel'),             # saída para driver
          ('/cmd_vel_raw', '/cmd_vel_raw'),      # entrada do controller
          ('/odom', '/odometry/filtered') #feedback
        ]
    )

    # Behavior server (spin, backup, wait, etc.)
#    behavior_server = Node(
#        package='nav2_behaviors',
#        executable='behavior_server',
#        name='behavior_server',
#        output='screen',
#        parameters=[nav2_params]
#    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator_custom',
        output='screen',
        parameters=[
            '/home/robo/ros2_ws/src/meu_pacote_navegacao/config/bt_nav_params.yaml',
            {
                'default_bt_xml_filename': '/home/robo/ros2_ws/behavior_trees/inline_test.xml',
                'default_nav_to_pose_bt_xml': '/home/robo/ros2_ws/behavior_trees/inline_test.xml',
                'default_nav_through_poses_bt_xml': '/home/robo/ros2_ws/behavior_trees/inline_test.xml',
                'always_reload_bt_xml': True
            }
        ]
    )

    # Lifecycle manager (ativa tudo)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'smoother_server',
                'bt_navigator_custom',
            #    'behavior_server',
            ]
        }]
    )

    return LaunchDescription([
        # Sensores e base
        ekf_node,
        static_tf,
        rplidar_launch,
        # Nav2 nodes standalone
        map_server,
        amcl,
        planner_server,
        controller_server,
        smoother_server,
        velocity_smoother,
#        behavior_server,
        bt_navigator,
        lifecycle_manager
    ])
