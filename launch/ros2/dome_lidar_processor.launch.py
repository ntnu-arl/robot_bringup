from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/rslidar_points',
        description='Input point cloud topic'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='rslidar',
        description='Frame ID for output messages'
    )
    
    slice_thickness_arg = DeclareLaunchArgument(
        'slice_thickness',
        default_value='90.0',
        description='Vertical slice thickness in degrees'
    )
    
    center_elevation_arg = DeclareLaunchArgument(
        'center_elevation',
        default_value='0.0',
        description='Center elevation angle in degrees'
    )
    
    fill_value_arg = DeclareLaunchArgument(
        'fill_value',
        default_value='1.5',
        description='Fill value for padding'
    )
    
    stretch_to_full_arg = DeclareLaunchArgument(
        'stretch_to_full',
        default_value='false',
        description='Stretch image to full 512x128 without padding'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='10.0',
        description='Maximum range for range image (in meters)'
    )
    
    use_intermediate_upsampling_arg = DeclareLaunchArgument(
        'use_intermediate_upsampling',
        default_value='true',
        description='Use intermediate 128x64 upsampling step'
    )
    
    min_range_threshold_arg = DeclareLaunchArgument(
        'min_range_threshold',
        default_value='0.3',
        description='Minimum range threshold (values below are set to 0)'
    )
    
    # Node configuration
    dome_lidar_processor_node = Node(
        package='dome_lidar_processor',
        executable='dome_lidar_processor_node',
        name='dome_lidar_processor',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'slice_thickness': LaunchConfiguration('slice_thickness'),
            'center_elevation': LaunchConfiguration('center_elevation'),
            'fill_value': LaunchConfiguration('fill_value'),
            'stretch_to_full': LaunchConfiguration('stretch_to_full'),
            'max_range': LaunchConfiguration('max_range'),
            'use_intermediate_upsampling': LaunchConfiguration('use_intermediate_upsampling'),
            'min_range_threshold': LaunchConfiguration('min_range_threshold'),
        }],
        remappings=[
            ('slice_pointcloud', '/dome_lidar/slice_pointcloud'),
            ('range_128x64', '/dome_lidar/range_128x64'),
            ('range_256x128', '/dome_lidar/range_256x128'),
            ('range_512x128', '/dome_lidar/range_512x128'),
        ]
    )
    
    return LaunchDescription([
        input_topic_arg,
        frame_id_arg,
        slice_thickness_arg,
        center_elevation_arg,
        fill_value_arg,
        stretch_to_full_arg,
        max_range_arg,
        use_intermediate_upsampling_arg,
        min_range_threshold_arg,
        dome_lidar_processor_node,
    ])