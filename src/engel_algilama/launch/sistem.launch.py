"""Tüm sistemi tek komutla başlatır: LiDAR sürücüsü + RViz + engel_node + ayar paneli.

  ros2 launch engel_algilama sistem.launch.py                 # canlı LiDAR
  ros2 launch engel_algilama sistem.launch.py surucu:=false   # rosbag oynatırken

Masaüstündeki "Engel Algılama" kısayolu da bunu çalıştırır.

NOT: LiDAR sürücüsü, rslidar_sdk'nın kendi start.py dosyasındakiyle AYNI şekilde
başlatılır (kendi config.yaml dosyasını kullanır). Sürücünün hiçbir ayarı
burada değiştirilmez. Sadece RViz, bizim hazır ayarlı ekranımızla açılır.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    paket = get_package_share_directory('engel_algilama')

    # Kayıt (rosbag) oynatırken sürücü açılmamalı: ikisi aynı topic'e yayın yapar
    surucu_arg = DeclareLaunchArgument(
        'surucu', default_value='true',
        description='LiDAR surucusunu ac (rosbag oynatirken false)')

    # 1) LiDAR sürücüsü — rslidar_sdk/launch/start.py ile birebir aynı ayarlar
    surucu = Node(
        namespace='rslidar_sdk',
        package='rslidar_sdk',
        executable='rslidar_sdk_node',
        output='screen',
        parameters=[{'config_path': ''}],  # boş = sürücünün kendi config.yaml'ı
        condition=IfCondition(LaunchConfiguration('surucu')),
    )

    # 2) RViz — ham bulut + filtrelenmiş noktalar + yelpaze/karar görselleri hazır ekli
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(paket, 'rviz', 'engel.rviz')],
    )

    # 3) engel_node + ayar paneli
    engel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(paket, 'launch', 'engel.launch.py')))

    return LaunchDescription([surucu_arg, surucu, rviz, engel])
