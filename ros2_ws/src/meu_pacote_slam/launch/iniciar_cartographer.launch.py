import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('meu_pacote_slam')
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    configuration_basename = 'cartographer.lua'

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        output='screen'
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=['-configuration_directory', cartographer_config_dir, '-configuration_basename', configuration_basename]
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )
    
    rviz_node = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2',
    )

    return LaunchDescription([
        rplidar_node,
        static_tf_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node
    ])
