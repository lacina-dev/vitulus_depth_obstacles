"""D435 ground / obstacle clouds in one node (vitulus_depth_obstacles), drop-in for vitulus_nav2 rtabmap_obstacles.

Same inputs (aligned depth + colour camera_info of `camera2`) and the same output topics as the RTAB-Map chains:
/cloudXYZ, /obstacles_cloud, /ground_cloud and /mapping/{cloudXYZ,obstacles_cloud,ground_cloud}_wide."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    cam = LaunchConfiguration('camera2').perform(context)
    return [Node(package='vitulus_depth_obstacles', executable='depth_obstacles', name='depth_obstacles',
                 output='log',
                 parameters=[LaunchConfiguration('params_file').perform(context),
                             {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true',
                              'depth_topic': f'/{cam}/aligned_depth_to_color/image_raw',
                              'camera_info_topic': f'/{cam}/color/camera_info'}])]


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('vitulus_depth_obstacles'), 'config', 'd435.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('camera2', default_value='d435'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=cfg),
        OpaqueFunction(function=_setup)])
