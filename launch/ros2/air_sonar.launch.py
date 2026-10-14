"""Launch only the PICASO sonar driver with the stack's hardware profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_driver(context):
    """Apply an explicit port override while preserving the YAML port by default.

    :param context: Resolved launch arguments.
    :return: The standalone sonar node.
    """
    parameters = [LaunchConfiguration("config_file")]
    port = LaunchConfiguration("port").perform(context)
    if port != "config":
        parameters.append({"port": ParameterValue("" if port == "auto" else port, value_type=str)})
    return [Node(package="picaso_driver", executable="picaso_driver", name="air_sonar",
                 output="screen", parameters=parameters,
                 arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")])]


def generate_launch_description() -> LaunchDescription:
    """Create the standalone driver launch.

    :return: Driver configuration and launch argument declarations.
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_bringup"), "config", "ros2", "air_sonar.yaml"]
                ),
                description="ROS parameter YAML for the air sonar driver.",
            ),
            DeclareLaunchArgument(
                "port", default_value="config",
                description="Serial path override; config keeps YAML, auto enables USB discovery."
            ),
            DeclareLaunchArgument(
                "log_level", default_value="info", description="ROS logging verbosity."
            ),
            OpaqueFunction(function=launch_driver),
        ]
    )
