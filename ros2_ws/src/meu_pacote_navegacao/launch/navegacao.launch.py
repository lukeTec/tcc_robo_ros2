import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Encontrar os diretórios dos nossos pacotes
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    meu_pacote_slam_dir = get_package_share_directory('meu_pacote_slam')
    meu_pacote_nav_dir = get_package_share_directory('meu_pacote_navegacao')

    # --- Argumentos de Launch ---
    # 1. O mapa que salvámos
    map_file = LaunchConfiguration('map')
    declare_map_arg = DeclareLaunchArgument(
        'map',
        # (ATENÇÃO: Este é o caminho padrão. Nós vamos substituí-lo na linha de comando)
        default_value=os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'), 
        description='Caminho completo para o ficheiro map.yaml'
    )
    
    # 2. Parâmetros do Nav2 (usamos os padrão por agora)
    nav2_params_file = LaunchConfiguration('params_file')
    declare_nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
        description='Caminho completo para o ficheiro de parâmetros do Nav2'
    )

    # --- NÓS (Os nossos 5 terminais) ---

    # Terminal 2: Driver Serial
    serial_driver_node = Node(
        package='robo_driver',
        executable='serial_driver_node',
        name='serial_driver_node',
        output='screen'
    )

    # Terminal 3: LIDAR (Do seu pacote de SLAM)
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(meu_pacote_slam_dir, 'launch', 'rplidar.launch.py')
        )
    )

    # Terminal 4: TF Estático (base_link -> laser)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # Terminal 5: EKF (Usando o nosso ficheiro .yaml CORRIGIDO)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(meu_pacote_nav_dir, 'config', 'robot_localization.yaml')]
    )

    # Terminal 6: Pilha Nav2 (AMCL, Map Server, etc.)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'False'
        }.items()
    )

    # --- Retorna a Descrição do Launch ---
    return LaunchDescription([
        declare_map_arg,
        declare_nav2_params_arg,
        
        serial_driver_node,    # T2
        rplidar_launch,        # T3
        static_tf_node,        # T4
        ekf_node,              # T5
        nav2_bringup_launch    # T6
    ])
