"""The RTAB-Map chains depth_obstacles replaces, for A/B (tools/ab_rtabmap.py): 2x rtabmap_util point_cloud_xyz +
2x obstacles_detection with the values of rtabmap_docker obstacles_detection.launch (test/data), same input topics,
outputs as in that launch (or under `prefix`, e.g. prefix:=/ref so both can run at once).

ROS2 rtabmap declares the Grid/* parameters as strings; wait_for_transform is one double (seconds)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

XYZ = {'approx_sync': True, 'decimation': 8, 'voxel_size': 0.05, 'noise_filter_radius': 0.12,
       'noise_filter_min_neighbors': 5}
GRID = {'frame_id': 'base_link', 'wait_for_transform': 0.2, 'Grid/MinClusterSize': '2',
        'Grid/MaxGroundHeight': '0.15', 'Grid/MaxGroundAngle': '35', 'Grid/ClusterRadius': '0.15',
        'Grid/NormalsSegmentation': 'true', 'Grid/FlatObstacleDetected': 'true'}
CHAINS = {  # name: (max_depth, Grid/MaxObstacleHeight, cloud, obstacles, ground)
    '': (2.8, '0.5', '/cloudXYZ', '/obstacles_cloud', '/ground_cloud'),
    '_wide': (4.0, '2.0', '/mapping/cloudXYZ_wide', '/mapping/obstacles_cloud_wide', '/mapping/ground_cloud_wide'),
}


def _setup(context):
    cam = LaunchConfiguration('camera2').perform(context)
    pre = LaunchConfiguration('prefix').perform(context).rstrip('/')
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    out = []
    for sfx, (max_depth, max_h, cloud, obst, ground) in CHAINS.items():
        out.append(Node(package='rtabmap_util', executable='point_cloud_xyz', name='depth2cloud' + sfx, output='log',
                        parameters=[dict(XYZ, max_depth=max_depth, use_sim_time=sim)],
                        remappings=[('depth/image', '/%s/aligned_depth_to_color/image_raw' % cam),
                                    ('depth/camera_info', '/%s/color/camera_info' % cam), ('cloud', pre + cloud)]))
        out.append(Node(package='rtabmap_util', executable='obstacles_detection', name='obstacles_detection' + sfx,
                        output='log', parameters=[dict(GRID, **{'Grid/MaxObstacleHeight': max_h}, use_sim_time=sim)],
                        remappings=[('cloud', pre + cloud), ('obstacles', pre + obst), ('ground', pre + ground)]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera2', default_value='d435'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup)])
