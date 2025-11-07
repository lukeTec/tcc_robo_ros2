from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # 115200 para A1/A2, 256000 para A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'use_sim_time': False
            }]
        )
    ])
