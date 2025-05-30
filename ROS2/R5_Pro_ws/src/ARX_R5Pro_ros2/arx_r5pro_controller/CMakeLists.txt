cmake_minimum_required(VERSION 3.8)
project(arx_r5pro_controller)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(arm_control REQUIRED)
find_package(arx5_arm_msg REQUIRED)

include_directories(include)

find_library(ARX_SRC_LIB
    NAMES arx_r5pro_src
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5pro_src
    NO_DEFAULT_PATH  # 禁止在其他路径中搜索
)

# 确保库文件被找到
if (NOT ARX_SRC_LIB)
    message(FATAL_ERROR "library not found!")
endif()

# 添加可执行文件
add_executable(R5ProController src/R5ProController.cpp)
target_include_directories(R5ProController PUBLIC lib/arx_hardware_interface/include lib/arx_r5pro_src/include)
target_link_libraries(R5ProController ${ARX_SRC_LIB})
ament_target_dependencies(R5ProController rclcpp std_msgs arx5_arm_msg arm_control)

add_executable(KeyBoard src/KeyBoard.cpp)
ament_target_dependencies(KeyBoard rclcpp std_msgs arx5_arm_msg)

install(TARGETS
    R5ProController
    KeyBoard
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${ARX_SRC_LIB}
  DESTINATION lib/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml")

ament_export_dependencies(rosidl_default_runtime)

ament_package()