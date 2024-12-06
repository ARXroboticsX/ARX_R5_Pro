import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory


from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
import xacro


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("gazebo_ros"),
                "/launch",
                "/gzserver.launch.py",
            ]
        ),
        launch_arguments={"pause": "true"}.items(),
    )

    pkg_dir = get_package_share_directory("arx_l5pro_description")
    simulation_description_path = os.path.join(pkg_dir)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [simulation_description_path, "model", "xacro", "L5Pro.xacro"]
            ),
        ]
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "arx_l5pro",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            gazebo,
            spawn_entity,
        ]
    )
