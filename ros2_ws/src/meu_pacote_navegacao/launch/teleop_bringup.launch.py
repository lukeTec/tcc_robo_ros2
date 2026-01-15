#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- 1. DEFINIÇÃO DE DIRETÓRIOS E ARQUIVOS (ORGANIZAÇÃO) ---
    pkg_nav = get_package_share_directory('meu_pacote_navegacao')
    pkg_slam = get_package_share_directory('meu_pacote_slam')
    pkg_driver = get_package_share_directory('robo_driver') # Seu pacote de driver

    # Configurações Navegação
    map_file = os.path.join(pkg_nav, 'maps', 'meu_mapa_final.yaml')
    nav2_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    ekf_config_file = os.path.join(pkg_nav, 'config', 'robot_localization.yaml')
    urdf_file = os.path.join(pkg_nav, 'urdf', 'meu_robo.urdf')
    rviz_config_file = os.path.join(pkg_nav, 'config', 'view_robot.rviz')
    
    # Configuração do Twist Mux (NOVO)
    mux_config_file = os.path.join(pkg_nav, 'config', 'twist_mux.yaml')

    # Leitura do URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # --- 2. NODES DE HARDWARE E CONTROLE (A Mágica Acontece Aqui) ---
    
    # 2.1 Publica TFs estáticas do robô
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
    )

    # 2.2 Driver do LIDAR
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'rplidar.launch.py')
        )
    )

    # 2.3 BLOCO DE CONTROLE DE MOTORES E SEGURANÇA (Substitui o antigo serial_driver simples)
    
    # A) Nó que fala com o Arduino
    serial_node = Node(
        package='robo_driver',
        executable='serial_node',
        name='serial_node',
        output='screen',
        # Ele obedece ao tópico final que sai do Mux
        remappings=[('/cmd_vel', '/cmd_vel')] 
    )

    # B) Nó de Segurança (Guarda-Costas)
    safety_node = Node(
        package='robo_driver',
        executable='safety_node',
        name='safety_node',
        output='screen'
        # Publica em /safety_stop
    )

    # C) Twist Mux (Gerente de Prioridades)
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[mux_config_file],
        remappings=[
            ('cmd_vel_out', 'cmd_vel') # Saída oficial para o Arduino
        ]
    )

    # --- 3. LOCALIZAÇÃO E ODOMETRIA ---
    
    # Robot Localization (EKF)
    ekf_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config_file]
            )
        ]
    )

    # Stack de Mapa (Map Server + AMCL)
    map_localization = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_file}]
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_params_file]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['map_server', 'amcl']}]
            )
        ]
    )

    return LaunchDescription([
        # 1. Hardware Básico
        robot_state_publisher,
        rplidar_launch,
        
        # 2. Trio de Controle (Arduino + Segurança + Mux)
        serial_node,
        safety_node,
        twist_mux,

        # 3. Inteligência (Localização)
        ekf_node,
        map_localization,
    ])
