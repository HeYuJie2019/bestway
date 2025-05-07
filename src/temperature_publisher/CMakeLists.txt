# filepath: /home/bestway/bestway_ws/src/temperature_publisher/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(temperature_publisher)

# 找到依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 添加 include 目录
include_directories(include)

# 添加库路径
link_directories(/home/bestway/SV_NET_SDK_ARM_250424/demo/consoleDemo/lib/x64/) # 替换为实际的库路径

# 添加可执行文件
add_executable(temperature_publisher src/temperature_publisher.cpp)

# 链接依赖项
ament_target_dependencies(temperature_publisher rclcpp std_msgs)

# 链接外部库
target_link_libraries(temperature_publisher libSV_NET_SDK.so) # 替换为实际的库名（如 libSVNETSDK.so）

# 安装目标
install(TARGETS
  temperature_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()