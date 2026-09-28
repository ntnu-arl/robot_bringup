from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("robot_bringup"), "config", "ros2", "agentic_uas_unipilot.yaml"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=config),
        DeclareLaunchArgument("log_level", default_value="info"),
        Node(
            package="agentic_uas", executable="agentic_uas_node", output="screen",
            parameters=[{"config_path": LaunchConfiguration("config_path")}],
            arguments=["--ros-args", "--log-level",
                       ["agentic_uas:=", LaunchConfiguration("log_level")]],
        ),
        Node(package="agentic_uas", executable="graph_candidates_node", output="screen"),
    ])
