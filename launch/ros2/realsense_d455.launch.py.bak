from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Standalone D455 driver: publishes color+depth (and compressed siblings via
    # image_transport, see Dockerfile.ros2_realsense) so topics exist for rviz/rosbag,
    # but nothing in this stack subscribes to them - not wired into mimosa/gbplanner/NMPC.
    rs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            # camera_namespace defaults to 'camera' just like camera_name, which doubles up
            # to /camera/camera/... - leave it empty so topics land under a plain /d455/...
            'camera_name': 'd455',
            'camera_namespace': '',
            'enable_color': 'true',
            'rgb_camera.color_profile': '640x480x15',
            'enable_depth': 'true',
            'depth_module.depth_profile': '640x480x15',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'publish_tf': 'true',
        }.items()
    )

    return LaunchDescription([rs_launch])
