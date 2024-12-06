cmake_minimum_required(VERSION 3.8)
project(arx_r5pro_controller)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(arm_control REQUIRED)
find_package(arx_r5pro_msg REQUIRED)

include_directories(include)

# 添加共享库路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 添加可执行文件
add_executable(R5ProController src/R5ProController.cpp)
target_include_directories(R5ProController PUBLIC lib/arx_hardware_interface/include lib/arx_r5pro_src/include)
target_link_libraries(R5ProController ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5pro_src/libarx_r5pro_src.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_hardware_interface/libarx_hardware_interface.so)
ament_target_dependencies(R5ProController rclcpp std_msgs arx_r5pro_msg)

add_executable(R5ProControllerVr src/R5ProControllerVr.cpp)
target_include_directories(R5ProControllerVr PUBLIC lib/arx_hardware_interface/include lib/arx_r5pro_src/include)
target_link_libraries(R5ProControllerVr ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5pro_src/libarx_r5pro_src.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_hardware_interface/libarx_hardware_interface.so)
ament_target_dependencies(R5ProControllerVr rclcpp std_msgs arx_r5pro_msg arm_control)

add_executable(KeyBoard src/KeyBoard.cpp)
ament_target_dependencies(KeyBoard rclcpp std_msgs arx_r5pro_msg)

install(TARGETS
    R5ProController
    R5ProControllerVr
    KeyBoard
  DESTINATION lib/${PROJECT_NAME})

install(FILES lib/arx_r5pro_src/libarx_r5pro_src.so lib/arx_hardware_interface/libarx_hardware_interface.so
  DESTINATION lib/
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()