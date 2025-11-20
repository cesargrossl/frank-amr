from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 1) LIDAR C1
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_c1_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'channel_type': 'serial',
            'serial_baudrate': '460800',
            'frame_id': 'laser',
        }.items()
    )

    # 2) TF estática base_link -> laser
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    # 3) SLAM Toolbox em modo mapeamento (online async)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'odom_frame': 'base_link',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'scan_topic': '/scan',
        }]
    )

    return LaunchDescription([
        sllidar_launch,
        static_tf,
        slam_node
    ])


#cd ~/ros2_ws
#colcon build
#source install/setup.bash


#para usar
#source /opt/ros/humble/setup.bash
#source ~/ros2_ws/install/setup.bash
#ros2 launch meu_slam slam_mapping.launch.py
