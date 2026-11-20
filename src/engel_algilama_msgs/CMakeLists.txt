cmake_minimum_required(VERSION 3.8)
project(engel_algilama_msgs)

# Sadece mesaj tanımları. Ayrı paket olmasının sebebi: ileride yazılacak
# füzyon node'u (C++ veya Python) node kodunu çekmeden bu mesajları kullanabilsin.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Nesne.msg"
  "msg/NesneDizisi.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
