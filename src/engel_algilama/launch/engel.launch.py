"""engel_node'u (ve isteğe bağlı ayar panelini) başlatan launch dosyası.

Kullanım:
  ros2 launch engel_algilama engel.launch.py              # node + ayar paneli
  ros2 launch engel_algilama engel.launch.py panel:=false # sadece node
  ros2 launch engel_algilama engel.launch.py nesne:=false # nesne_node'u açma

nesne_node (kutu + mesafe + etiket + takip) config/nesne_params.yaml'dan yüklenir.

Parametreler şu sırayla yüklenir (sonraki öncekinin üstüne yazar):
  1) config/engel_params.yaml / config/nesne_params.yaml -> paketin varsayılan değerleri
  2) ~/.ros/engel_algilama/son_ayarlar.yaml -> panelde en son uygulanan ayarlar (varsa)
     (dosyada "engel_node:" ve "nesne_node:" bölümleri ayrı ayrı durur)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SON_AYARLAR = os.path.expanduser('~/.ros/engel_algilama/son_ayarlar.yaml')


def generate_launch_description():
    params_dosyasi = os.path.join(
        get_package_share_directory('engel_algilama'), 'config', 'engel_params.yaml')

    parametreler = [params_dosyasi]
    if os.path.exists(SON_AYARLAR):
        parametreler.append(SON_AYARLAR)
        bilgi = LogInfo(msg=f'Kayitli ayarlar yukleniyor: {SON_AYARLAR}')
    else:
        bilgi = LogInfo(msg='Kayitli ayar yok, varsayilan degerler kullaniliyor.')

    nesne_parametreler = [os.path.join(
        get_package_share_directory('engel_algilama'), 'config', 'nesne_params.yaml')]
    if os.path.exists(SON_AYARLAR):
        nesne_parametreler.append(SON_AYARLAR)  # dosyadaki "nesne_node:" bölümü kullanılır

    # "panel:=false" yazılırsa ayar paneli açılmaz
    panel_arg = DeclareLaunchArgument(
        'panel', default_value='true', description='Ayar panelini de ac (true/false)')

    # "nesne:=false" yazılırsa nesne algılama (kutular) açılmaz
    nesne_arg = DeclareLaunchArgument(
        'nesne', default_value='true', description='nesne_node (kutu/mesafe/takip) ac (true/false)')

    nesne_node = Node(
        package='engel_algilama',
        executable='nesne_node',
        name='nesne_node',          # nesne_params.yaml'daki en üst anahtarla aynı olmalı
        output='screen',
        # Panelde uygulanan son ayarlar (kutulama alanı) varsa varsayılanların üstüne yazar
        parameters=nesne_parametreler,
        condition=IfCondition(LaunchConfiguration('nesne')),
    )

    engel_node = Node(
        package='engel_algilama',
        executable='engel_node',
        name='engel_node',          # yaml'daki en üst anahtarla aynı olmalı
        output='screen',            # terminalde canlı satırlar görünsün
        parameters=parametreler,
    )

    ayar_paneli = Node(
        package='engel_algilama',
        executable='engel_ayar_paneli',
        name='engel_ayar_paneli',
        parameters=[{'hedef_node': 'engel_node'}],
        condition=IfCondition(LaunchConfiguration('panel')),
    )

    return LaunchDescription([panel_arg, nesne_arg, bilgi, engel_node, nesne_node, ayar_paneli])
