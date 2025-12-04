from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare the config file argument
    declare_cfg_arg = DeclareLaunchArgument(
        'cfg',
        default_value='cbf_pc_selector_sim_lidar.yaml',
        description='Config file name (only filename, not full path)'
    )
    
    # Create the full path to the config file
    cfg_file = PathJoinSubstitution([
        get_package_share_directory('robot_bringup'),
        'config',
        'ros2',
        LaunchConfiguration('cfg')
    ])
    
    node = Node(
        package='cbf_pc_selector',
        executable='pc_selector_node',
        name='cbf_pc_selector',
        parameters=[
            cfg_file,
            {'use_sim_time': True}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_cfg_arg,
        node
    ])