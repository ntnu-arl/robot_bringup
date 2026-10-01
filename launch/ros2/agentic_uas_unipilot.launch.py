from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("robot_bringup"), "config", "ros2", "agentic_uas_unipilot.yaml"
    ])
    system_prompt = PathJoinSubstitution([
        FindPackageShare("robot_bringup"), "config", "ros2", "agentic_uas_system_prompt.txt"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("config_path", default_value=config),
        DeclareLaunchArgument("system_prompt_path", default_value=system_prompt),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("method", default_value=""),
        Node(
            package="agentic_uas", executable="agentic_uas_node", output="screen",
            parameters=[{
                "config_path": LaunchConfiguration("config_path"),
                "method": LaunchConfiguration("method"),
                "system_prompt_path": LaunchConfiguration("system_prompt_path"),
            }],
            arguments=["--ros-args", "--log-level",
                       ["agentic_uas:=", LaunchConfiguration("log_level")]],
        ),
        Node(package="agentic_uas", executable="graph_candidates_node", output="screen"),
    ])
