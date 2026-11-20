cmake_minimum_required(VERSION 3.8)
project(engel_algilama)

# C++17 standardını kullan
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Derleyici uyarılarını aç (hataları erken görmek için faydalı)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıkları bul (package.xml'deki <depend> satırlarıyla aynı olmalı)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# engel_node adında çalıştırılabilir dosya oluştur
add_executable(engel_node src/engel_node.cpp)

# Node'u bağımlılıklara bağla (başlık dosyaları + kütüphaneler)
ament_target_dependencies(engel_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  rcl_interfaces
  geometry_msgs
)

# Çalıştırılabilir dosyayı "ros2 run" bulabilsin diye lib/<paket_adı> altına kur
install(TARGETS engel_node
  DESTINATION lib/${PROJECT_NAME}
)

# Python ayar panelini "engel_ayar_paneli" adıyla kur (derleme gerekmez)
install(PROGRAMS scripts/engel_ayar_paneli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME engel_ayar_paneli
)

# config ve launch klasörlerini share/<paket_adı> altına kur
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
