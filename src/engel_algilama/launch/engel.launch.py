"""engel_node'u (ve isteğe bağlı ayar panelini) başlatan launch dosyası.

Kullanım:
  ros2 launch engel_algilama engel.launch.py              # node + ayar paneli
  ros2 launch engel_algilama engel.launch.py panel:=false # sadece node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_dosyasi = os.path.join(
        get_package_share_directory('engel_algilama'), 'config', 'engel_params.yaml')

    # "panel:=false" yazılırsa ayar paneli açılmaz
    panel_arg = DeclareLaunchArgument(
        'panel', default_value='true', description='Ayar panelini de ac (true/false)')

    engel_node = Node(
        package='engel_algilama',
        executable='engel_node',
        name='engel_node',          # yaml'daki en üst anahtarla aynı olmalı
        output='screen',            # terminalde canlı satırlar görünsün
        parameters=[params_dosyasi],
    )

    ayar_paneli = Node(
        package='engel_algilama',
        executable='engel_ayar_paneli',
        name='engel_ayar_paneli',
        parameters=[{'hedef_node': 'engel_node'}],
        condition=IfCondition(LaunchConfiguration('panel')),
    )

    return LaunchDescription([panel_arg, engel_node, ayar_paneli])
