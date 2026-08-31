from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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
            # Reset the camera's USB connection at launch - reduces (though does not
            # fully eliminate) the well-documented librealsense-on-Jetson
            # 'messenger-libusb.cpp control_transfer ... Resource temporarily
            # unavailable' warning burst seen at startup (see IntelRealSense/
            # librealsense#10902 and related issues). Confirmed low-risk: this
            # camera isn't consumed by mimosa/gbplanner/NMPC (see the comment
            # above), so a slightly slower/different startup here has no
            # downstream effect on navigation.
            'initial_reset': 'true',
        }.items()
    )

    # Static TF so the depth pointcloud/image can be visualized in rviz aligned
    # with the robot body frame - previously missing entirely (nothing published
    # mimosa_body -> the D455's own frame tree, so rviz had no way to place the
    # camera data relative to mimosa_body/mimosa_world). publish_tf=true above
    # already makes the D455 driver publish its OWN internal chain rooted at
    # d455_link (camera_name=d455 above) down to d455_depth_optical_frame -
    # this is the ONE missing link needed to connect that whole chain to the
    # robot's own TF tree, matching the exact static_transform_publisher
    # convention already used elsewhere in this stack (see uav_sim.launch.xml).
    #
    # TRANSLATION starts from the radar's own real, measured extrinsic
    # (mimosa_body -> mimosa_radar, see mimosa.yaml's T_B_S:
    # [0.080, -0.034, 0.016, ...], x/y/z) since the D455 is mounted directly
    # above the radar and axis-aligned with it in x/y - z is bumped up by an
    # UNMEASURED placeholder (5cm, a rough guess for mounted just above).
    #
    # ROTATION is identity (no rotation at all), NOT the radar's own
    # [0.6830, 0.1830, -0.1830, 0.6830] - the D455 points the same general
    # direction as the radar (forward, outside the robot) but is mounted
    # level/vertically-aligned, without the radar's ~30deg down-tilt. Since
    # the D455 driver's own d455_link frame already follows the standard
    # ROS camera-mount convention (x-forward, y-left, z-up - the same
    # convention mimosa_body itself uses), a level, untilted, unrolled mount
    # needs NO rotation here at all - the radar's own 90deg roll component
    # was specifically compensating for the RADAR's different native sensor-
    # frame convention, which doesn't apply to the D455.
    #
    # NEITHER translation nor rotation independently measured for the D455
    # itself yet - re-measure/finetune both once the robot is up and the
    # pointcloud can be checked against real geometry in rviz, per the plan
    # already discussed. In particular, double check there isn't a small
    # roll if the camera housing itself is mounted slightly rotated about
    # its own forward axis - identity assumes it is genuinely level.
    d455_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_mimosa_body_d455',
        arguments=[
            '0.080', '-0.034', '0.066',  # x y z (z = radar's 0.016 + 0.05 placeholder)
            '0.0', '0.0', '0.0', '1.0',  # qx qy qz qw - identity (level, no tilt/roll)
            'mimosa_body', 'd455_link',
        ],
    )

    return LaunchDescription([rs_launch, d455_static_tf])
