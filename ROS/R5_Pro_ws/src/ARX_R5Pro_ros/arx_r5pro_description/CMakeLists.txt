cmake_minimum_required(VERSION 3.8)
project(arx_r5pro_description)

find_package(catkin REQUIRED COMPONENTS
        roscpp
)

catkin_package()

install(
        DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch/
)

install(
        DIRECTORY model/
        DESTINATION share/${PROJECT_NAME}/model/
)
