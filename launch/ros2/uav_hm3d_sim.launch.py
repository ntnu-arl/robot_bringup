"""Launch the existing Unipilot sensors and robot in a converted HM3D scene."""

import json
import math
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def scene_launch(context):
    """Read the imported scene's saved world and free-space spawn.

    :param context: Resolved launch context.
    :return: Existing simulation launch configured for the selected scene.
    """
    assets = Path(LaunchConfiguration('assets_dir').perform(context)).resolve()
    scene = LaunchConfiguration('scene').perform(context)
    if not re.fullmatch(r'[A-Za-z0-9_-]+', scene):
        raise ValueError('scene must be an HM3D scene directory id')
    model = 'hm3d_' + scene.replace('-', '_')
    world = assets / 'worlds' / f'{model}.sdf'
    manifest = world.with_suffix('.json')
    if not world.is_file() or not manifest.is_file():
        raise FileNotFoundError(f'Import the scene first; missing world/metadata at {world}')
    metadata = json.loads(manifest.read_text())
    if metadata['scene_id'] != scene or metadata['model_name'] != model:
        raise ValueError('Imported metadata does not match requested scene')
    position, heading = metadata['spawn']['position'], metadata['spawn']['yaw']
    if len(position) != 3 or not all(math.isfinite(value) for value in [*position, heading]):
        raise ValueError('Imported spawn must contain finite xyz and yaw')
    simulation = Path(get_package_share_directory('robot_bringup')) / 'launch/ros2/uav_sim_unipilot.launch.xml'
    return [IncludeLaunchDescription(
        AnyLaunchDescriptionSource(str(simulation)),
        launch_arguments={
            'world_package': 'gz_sim_worlds', 'world_file': str(world),
            'world_name': metadata['world_name'], 'extra_resource_path': str(assets / 'models'),
            'x': str(position[0]), 'y': str(position[1]), 'z': str(position[2]), 'Y': str(heading),
            'headless': LaunchConfiguration('headless'),
            'verbosity': LaunchConfiguration('verbosity'),
        }.items())]


def generate_launch_description():
    """Declare the imported assets and selected scene.

    :return: Launch description using the existing RGB-D and LiDAR robot.
    """
    return LaunchDescription([
        DeclareLaunchArgument('assets_dir', default_value='/hm3d'),
        DeclareLaunchArgument('scene', description='Imported scene directory id, e.g. 00001-UVdNNRcVyV1'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('verbosity', default_value='2'),
        OpaqueFunction(function=scene_launch),
    ])
