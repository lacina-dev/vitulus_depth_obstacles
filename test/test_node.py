"""depth_obstacles node: config = the ROS1 obstacles_detection.launch values, topics, outputs on a synthetic scene,
runtime parameter changes and their validation."""
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
CFG = os.path.join(PKG, 'config', 'd435.yaml')
# rtabmap_docker obstacles_detection.launch (github.com/lacina-dev/rtabmap_docker 9caa739): the RTAB-Map chains replaced
ROS1 = os.path.join(HERE, 'data', 'obstacles_detection.launch')

# RTAB-Map parameter -> depth_obstacles parameter
MAP_XYZ = {'decimation': 'decimation', 'voxel_size': 'voxel_size', 'max_depth': 'max_depth',
           'noise_filter_radius': 'noise_filter_radius', 'noise_filter_min_neighbors': 'noise_filter_min_neighbors'}
MAP_GRID = {'Grid/MaxObstacleHeight': 'max_obstacle_height', 'Grid/MaxGroundHeight': 'max_ground_height',
            'Grid/MaxGroundAngle': 'max_ground_angle_deg', 'Grid/ClusterRadius': 'cluster_radius',
            'Grid/MinClusterSize': 'min_cluster_size', 'Grid/NormalsSegmentation': 'normals_segmentation',
            'Grid/FlatObstacleDetected': 'flat_obstacles_detected'}
IGNORED_BY_RTABMAP = {'Grid/MinObstacleHeight'}   # not an RTAB-Map 0.21/0.23 parameter


def typed(p):
    v, t = p.get('value'), p.get('type')
    return {'bool': lambda: v == 'true', 'int': lambda: int(v), 'double': lambda: float(v)}.get(t, lambda: v)()


def test_config_equals_ros1_launch():
    nodes = {n.get('name'): n for n in ET.parse(ROS1).getroot().iter('node')}
    prm = yaml.safe_load(open(CFG))['depth_obstacles']['ros__parameters']
    assert prm['frame_id'] == 'base_link'
    for prof, xyz, det in (('nav', 'depth2cloud', 'obstacles_detection'),
                           ('wide', 'depth2cloud_wide', 'obstacles_detection_wide')):
        p = prm[prof]
        for e in nodes[xyz].iter('param'):
            if e.get('name') in MAP_XYZ:
                assert p[MAP_XYZ[e.get('name')]] == typed(e), (prof, e.get('name'))
            else:
                assert e.get('name') == 'approx_sync' and prm['approx_sync'] == typed(e)
        remap = {r.get('from'): r.get('to') for r in nodes[det].iter('remap')}
        assert p['obstacles_topic'] == remap['obstacles'] and p['ground_topic'] == remap['ground']
        assert p['cloud_topic'] == remap['cloud']
        for e in nodes[det].iter('param'):
            n = e.get('name')
            if n in MAP_GRID:
                assert p[MAP_GRID[n]] == typed(e), (prof, n)
            elif n in IGNORED_BY_RTABMAP:
                assert p['min_obstacle_height'] == -1.0   # keep RTAB-Map's behaviour (floor = max_ground_height)
            else:
                assert n in ('frame_id', 'wait_for_transform'), n


# --- running node ---------------------------------------------------------------------------------------------

FX, CX, CY = 615.0, 319.5, 239.5


def render(box_h=0.3):
    """ground + a 0.3 m wide box 1.5 m ahead; camera 0.4 m high, 20 deg down (as test_segmentation.cpp)"""
    opt = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
    a = math.radians(20)
    rot_y = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    r = rot_y @ opt
    o = np.array([0.3, 0, 0.4])
    u, v = np.meshgrid(np.arange(640), np.arange(480))
    rays = np.stack([(u - CX) / FX, (v - CY) / FX, np.ones_like(u, float)], -1) @ r.T
    with np.errstate(divide='ignore', invalid='ignore'):
        s = np.where(rays[..., 2] < -1e-6, -o[2] / rays[..., 2], 1e9)
        sb = (1.5 - o[0]) / rays[..., 0]
        p = o + sb[..., None] * rays
        hit = (rays[..., 0] > 1e-6) & (np.abs(p[..., 1]) < 0.15) & (p[..., 2] >= 0) & (p[..., 2] <= box_h)
    s = np.where(hit & (sb < s), sb, s)
    img = np.where(s < 6, np.round(s * 1000), 0).astype(np.uint16)
    q = np.zeros(4)   # optical -> base quaternion (w, x, y, z) for the static TF
    m = r
    w = math.sqrt(max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    q = (w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w))
    return img, o, q


@pytest.fixture(scope='module')
def node():
    os.environ['ROS_DOMAIN_ID'] = os.environ.get('DEPTH_OBSTACLES_TEST_DOMAIN', '96')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    img, o, q = render()
    procs = [subprocess.Popen(['ros2', 'run', 'tf2_ros', 'static_transform_publisher', '--x', str(o[0]),
                               '--y', str(o[1]), '--z', str(o[2]), '--qx', str(q[1]), '--qy', str(q[2]),
                               '--qz', str(q[3]), '--qw', str(q[0]), '--frame-id', 'base_link',
                               '--child-frame-id', 'cam_optical'],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True),
             subprocess.Popen(['ros2', 'launch', 'vitulus_depth_obstacles', 'depth_obstacles.launch.py',
                               'camera2:=testcam'],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)]
    import rclpy
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    rclpy.init()
    n = rclpy.create_node('depth_obstacles_test')
    got = {}
    for t in ('/cloudXYZ', '/obstacles_cloud', '/ground_cloud', '/mapping/obstacles_cloud_wide',
              '/mapping/ground_cloud_wide', '/rest_cloud'):
        n.create_subscription(PointCloud2, t, lambda m, t=t: got.setdefault(t, []).append(m), 10)
    pd = n.create_publisher(Image, '/testcam/aligned_depth_to_color/image_raw', 2)
    pi = n.create_publisher(CameraInfo, '/testcam/color/camera_info', 2)
    t_end = time.time() + 30
    while time.time() < t_end and pd.get_subscription_count() < 1:
        rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(1.0)

    def frame():
        stamp = n.get_clock().now().to_msg()
        im = Image(height=480, width=640, encoding='16UC1', is_bigendian=0, step=1280, data=img.tobytes())
        im.header.stamp, im.header.frame_id = stamp, 'cam_optical'
        ci = CameraInfo(height=480, width=640)
        ci.header = im.header
        ci.k = [FX, 0.0, CX, 0.0, FX, CY, 0.0, 0.0, 1.0]
        ci.p = [FX, 0.0, CX, 0.0, 0.0, FX, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        got.clear()
        for _ in range(3):   # approx sync emits a pair once the next one arrives
            pd.publish(im)
            pi.publish(ci)
            t0 = time.time()
            while time.time() - t0 < 0.3:
                rclpy.spin_once(n, timeout_sec=0.05)
            im.header.stamp = ci.header.stamp = n.get_clock().now().to_msg()
        t0 = time.time()
        while time.time() - t0 < 3 and len(got) < 6:
            rclpy.spin_once(n, timeout_sec=0.05)
        return got

    yield n, frame
    n.destroy_node()
    rclpy.shutdown()
    for p in procs:
        os.killpg(p.pid, 2)
        p.wait(10)


def xyz(msg):
    from sensor_msgs_py import point_cloud2
    return np.asarray(point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z')), float).reshape(-1, 3)


def set_param(name, value, wait=60.0):
    """ros2 param set; retried while the CLI does not see the node yet (discovery is slow on a loaded PC)"""
    t_end = time.time() + wait
    while True:
        r = subprocess.run(['ros2', 'param', 'set', '/depth_obstacles', name, value], capture_output=True,
                           text=True, timeout=60)
        out = r.stdout + r.stderr
        if 'node not found' not in out.lower() or time.time() > t_end:
            return out
        time.sleep(1.0)


def test_outputs(node):
    _n, frame = node
    got = frame()
    assert set(got) == {'/cloudXYZ', '/obstacles_cloud', '/ground_cloud', '/mapping/obstacles_cloud_wide',
                        '/mapping/ground_cloud_wide', '/rest_cloud'}
    rest = got['/rest_cloud'][-1]
    assert [f.name for f in rest.fields] == ['x', 'y', 'z', 'intensity']   # intensity = reason 1..4
    assert rest.header.frame_id == 'cam_optical'
    m = got['/obstacles_cloud'][-1]
    assert m.header.frame_id == 'cam_optical'           # like RTAB-Map: input frame
    assert len(xyz(m)) > 3 and len(xyz(got['/ground_cloud'][-1])) > 100


def test_runtime_parameters(node):
    _n, frame = node
    assert 'successful' in set_param('nav.max_depth', '1.0').lower()
    got = frame()
    assert len(xyz(got['/obstacles_cloud'][-1])) == 0    # the box at 1.5 m is now out of range
    assert len(xyz(got['/mapping/obstacles_cloud_wide'][-1])) > 3   # wide profile unchanged
    assert 'successful' in set_param('nav.max_depth', '2.8').lower()
    assert len(xyz(frame()['/obstacles_cloud'][-1])) > 3
    out = set_param('nav.normal_k', '1')
    assert 'successful' not in out.lower() and 'normal_k' in out
