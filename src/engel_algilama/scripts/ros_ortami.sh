# shellcheck shell=bash
# =============================================================================
# Ortak ROS ortamı: masaüstü kısayollarıyla açılan tüm betikler bunu kullanır.
# (Kısayollar ~/.bashrc'yi okumadığı için ayarları burada tekrar veriyoruz.)
# Kullanım: source "$(dirname "$(readlink -f "$0")")/ros_ortami.sh"
# =============================================================================

source /opt/ros/jazzy/setup.bash
source "$HOME/lidar_ws/install/setup.bash"

# ~/.bashrc'deki ağ ayarlarının aynısı. setup.bash'lerden SONRA verilmeli,
# çünkü ROS'un setup.bash dosyası ROS_AUTOMATIC_DISCOVERY_RANGE'i eziyor.
export ROS_DOMAIN_ID=42                                   # Wi-Fi'deki diğer ROS bilgisayarlarından ayrı
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST            # sadece bu bilgisayar
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/lidar_ws/fastdds_buyuk_mesaj.xml"  # 1.5 MB'lık mesajlar için

KAYIT_KLASORU="$HOME/lidar_kayitlar"

# LiDAR sürücüsü çalışıyor mu?
surucu_calisiyor_mu() {
    pgrep -f "rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node" > /dev/null
}

# Hata/bitiş mesajından sonra terminal hemen kapanmasın
bekle() {
    local mesaj="$1"
    [ -z "$mesaj" ] && mesaj="Pencereyi kapatmak icin Enter'a basin..."
    echo
    read -r -p "$mesaj"
}
