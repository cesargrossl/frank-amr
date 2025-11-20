from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # --- 1) Launch do LIDAR C1 (reaproveita o launch do sllidar_ros2) ---
    sllidar_launch_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_c1_launch.py'
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_file),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'channel_type': 'serial',
            'serial_baudrate': '460800',
            'frame_id': 'laser',     # frame publicado pelo driver
        }.items()
    )

    # --- 2) TF estática: base_link -> laser ---
    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=[
            '0', '0', '0',           # x y z
            '0', '0', '0',           # roll pitch yaw
            'base_link',             # frame pai
            'laser'                  # frame filho (mesmo do frame_id do LIDAR)
        ],
        output='screen'
    )

    # --- 3) SLAM Toolbox (online, sem odometria) ---
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'odom_frame': 'base_link',   # por enquanto sem frame odom
            'base_frame': 'base_link',
            'map_frame': 'map',
            'scan_topic': '/scan',
        }]
        # se quiser, pode adicionar aqui um YAML com mais parâmetros
        # parameters=[os.path.join(
        #     get_package_share_directory('slam_toolbox'),
        #     'config', 'mapper_params_online_async.yaml')]
    )

    # --- 4) (Opcional) RViz já abrindo junto, com Fixed Frame = map ---
    # Se tiver um arquivo de config RViz, coloque aqui. Exemplo:
    # rviz_config = os.path.join(
    #     get_package_share_directory('frank_bringup'),
    #     'rviz', 'slam.rviz'
    # )
    #
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config]
    # )

    return LaunchDescription([
        lidar,
        static_tf_laser,
        slam_toolbox_node,
        # rviz,   # descomente se criar o arquivo de config do RViz
    ])
