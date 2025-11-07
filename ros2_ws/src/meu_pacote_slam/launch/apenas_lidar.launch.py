from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Nó do RPLIDAR, sem use_sim_time para gravação real
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

    # Nó da Transformada Estática (TF)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
    )

    return LaunchDescription([
        rplidar_node,
        static_tf_node
    ])
