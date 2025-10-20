# filepath: /home/bestway/bestway_ws/src/temperature_publisher/CMakeLists.txt
cmake_minimum_required(VERSION 3.5)
project(temperature_publisher)

# 找到依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 生成服务已删除，改为依赖外部服务包

# 添加 include 目录
include_directories(include)

# SDK 库路径（如需修改，改这里或用 -DSDK_LIB_DIR=... 指定）
set(SDK_LIB_DIR "/home/bestway/SV_NET_SDK_ARM_250424/demo/consoleDemo/lib/x64/" CACHE PATH "Path to SDK shared libraries")
set(SDK_INCLUDE_DIR "/home/bestway/SV_NET_SDK_ARM_250424/demo/consoleDemo/include/" CACHE PATH "Path to SDK include headers")

# 添加 include 和库路径
include_directories(${SDK_INCLUDE_DIR})
link_directories(${SDK_LIB_DIR})

# 添加可执行文件
add_executable(temperature_publisher src/temperature_publisher.cpp)

# 链接依赖项
ament_target_dependencies(temperature_publisher rclcpp std_msgs)

# 链接外部库
target_link_libraries(temperature_publisher libSV_NET_SDK.so) # 替换为实际的库名（如 libSVNETSDK.so）

# 运行时可找到 SDK 动态库（RPATH）；也可配合 env hook 设置 LD_LIBRARY_PATH
set_target_properties(temperature_publisher PROPERTIES
  BUILD_RPATH ${SDK_LIB_DIR}
  INSTALL_RPATH ${SDK_LIB_DIR}
)

# 安装目标
install(TARGETS
  temperature_publisher
  DESTINATION lib/${PROJECT_NAME})

# 安装环境 hook，自动追加 SDK 路径到 LD_LIBRARY_PATH
set(dsv_file "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/temperature_publisher_library_path.dsv.in")
set(dsv_out  "${CMAKE_CURRENT_BINARY_DIR}/temperature_publisher_library_path.dsv")
configure_file(${dsv_file} ${dsv_out} @ONLY)
install(FILES ${dsv_out}
  DESTINATION share/${PROJECT_NAME}/environment
)

ament_package()