import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

params_file = os.path.join(
    get_package_share_directory('arx_r5_controller'), 'config', 'vr_double_arm.yaml')

arm_l_node =Node(
    package='arx_r5_controller',
    executable='R5Controller',
    name='vr_arm_l',
    output='screen',                             
    parameters=[params_file],
)

arm_r_node =Node(
    package='arx_r5_controller',
    executable='R5Controller',
    name='vr_arm_r',
    output='screen',
    parameters=[params_file],
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        arm_l_node,

        TimerAction(
            period=0.5,  # 设置间隔时间为2秒
            actions=[arm_r_node]
        )
    ])