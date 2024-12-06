cmake_minimum_required(VERSION 3.8)
project(arx_r5pro_controller)

find_package(catkin REQUIRED COMPONENTS
        arm_control
        arx_r5pro_msg
        roscpp
        std_msgs
        message_generation
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS arm_control arx_r5pro_msg roscpp std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# 添加共享库路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 添加可执行文件
add_executable(R5ProController src/R5ProController.cpp)
target_include_directories(R5ProController PUBLIC lib/arx_hardware_interface/include lib/arx_r5pro_src/include)
target_link_libraries(R5ProController ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5pro_src/libarx_r5pro_src.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_hardware_interface/libarx_hardware_interface.so)

add_dependencies(R5ProController ${catkin_EXPORTED_TARGETS})

add_executable(R5ProControllerVr src/R5ProControllerVr.cpp)
target_include_directories(R5ProControllerVr PUBLIC lib/arx_hardware_interface/include lib/arx_r5pro_src/include)
target_link_libraries(R5ProControllerVr ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5pro_src/libarx_r5pro_src.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_hardware_interface/libarx_hardware_interface.so)
add_dependencies(R5ProControllerVr ${catkin_EXPORTED_TARGETS})

add_executable(KeyBoard src/KeyBoard.cpp)
target_include_directories(KeyBoard PUBLIC lib/arx_hardware_interface/include lib/arx_r5pro_src/include)
target_link_libraries(KeyBoard ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_r5pro_src/libarx_r5pro_src.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/arx_hardware_interface/libarx_hardware_interface.so)
add_dependencies(KeyBoard ${catkin_EXPORTED_TARGETS})

install(TARGETS
        R5ProController
        R5ProControllerVr
        DESTINATION lib/${PROJECT_NAME})

install(FILES lib/arx_r5pro_src/libarx_r5pro_src.so lib/arx_hardware_interface/libarx_hardware_interface.so
        DESTINATION lib/
)
