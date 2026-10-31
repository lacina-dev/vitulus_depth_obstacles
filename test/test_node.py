"""depth_obstacles node: config = the ROS1 obstacles_detection.launch values, topics, outputs on a synthetic scene,
runtime parameter changes and their validation. The node tests of the ROS version found in $ROS_VERSION run: ROS 2
(ros2 launch, own ROS_DOMAIN_ID), ROS 1 (live settings ~set / ~config, own roscore on port 11820, never another)."""
import json
import math
import os
import subprocess
import threading
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
ON_ROS1 = os.environ.get('ROS_VERSION') == '1'
only_ros2 = pytest.mark.skipif(ON_ROS1, reason='ROS 2 node')
only_ros1 = pytest.mark.skipif(not ON_ROS1, reason='ROS 1 node')


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


@only_ros2
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


@only_ros2
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


# --- ROS 1: live settings ---------------------------------------------------------------------------------------

MASTER = 'http://127.0.0.1:11820'
CFG1 = os.path.join(PKG, 'config', 'd435_ros1.yaml')


class Ros1Node:
    """the node on a private master, fed with the synthetic scene at 15 Hz"""

    def __init__(self, tmp):
        import rospy
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        from std_msgs.msg import String
        self.tmp = tmp
        self.store = os.path.join(tmp, 'settings.json')
        self.proc = None
        self.env = dict(os.environ)
        self.core = subprocess.Popen(['roscore', '-p', '11820'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                     start_new_session=True)
        time.sleep(3.0)
        rospy.init_node('depth_obstacles_test', disable_signals=True)
        self.rospy = rospy
        self.config = None
        self.lock = threading.Lock()
        self.clouds = {}
        self.stamps = []
        self.stats = None
        self.sets = rospy.Publisher('/depth_obstacles/set', String, queue_size=10)
        rospy.Subscriber('/depth_obstacles/config', String, self._on_config, queue_size=10)
        from diagnostic_msgs.msg import DiagnosticStatus
        rospy.Subscriber('/depth_obstacles/stats', DiagnosticStatus, self._on_stats, queue_size=2)
        for t in ('/cloudXYZ', '/obstacles_cloud', '/ground_cloud', '/rest_cloud', '/mapping/obstacles_cloud_wide',
                  '/mapping/ground_cloud_wide'):
            rospy.Subscriber(t, PointCloud2, lambda m, t=t: self._on_cloud(t, m), queue_size=5)
        img, o, q = render(0.45)
        self.o = o
        self.r = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
        a = math.radians(20)
        self.r = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]]) @ self.r
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id, t.child_frame_id = 'base_link', 'cam_optical'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = o
        t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = q
        self.tf = tf2_ros.StaticTransformBroadcaster()
        self.tf.sendTransform(t)
        self.pd = rospy.Publisher('/test/depth', Image, queue_size=2)
        self.pi = rospy.Publisher('/test/info', CameraInfo, queue_size=2)
        self.im = Image(height=480, width=640, encoding='16UC1', is_bigendian=0, step=1280, data=img.tobytes())
        self.ci = CameraInfo(height=480, width=640)
        self.ci.K = [FX, 0.0, CX, 0.0, FX, CY, 0.0, 0.0, 1.0]
        self.ci.P = [FX, 0.0, CX, 0.0, 0.0, FX, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.feeding = True
        self.feeder = threading.Thread(target=self._feed, daemon=True)
        self.feeder.start()

    def _on_config(self, m):
        with self.lock:
            self.config = json.loads(m.data)

    def _on_stats(self, m):
        self.stats = {v.key: v.value for v in m.values}
        self.stats['_level'] = m.level
        self.stats['_message'] = m.message

    def _on_cloud(self, t, m):
        with self.lock:
            self.clouds[t] = m
            if t == '/obstacles_cloud':
                self.stamps.append(time.time())

    def _feed(self):
        while self.feeding:
            stamp = self.rospy.Time.now()
            self.im.header.stamp = self.ci.header.stamp = stamp
            self.im.header.frame_id = self.ci.header.frame_id = 'cam_optical'
            self.pd.publish(self.im)
            self.pi.publish(self.ci)
            time.sleep(1 / 15.0)

    def start(self, **params):
        """a node with a clean parameter set; params: private parameters over the YAML"""
        self.stop()
        subprocess.check_call(['rosparam', 'delete', '/depth_obstacles'], env=self.env, stderr=subprocess.DEVNULL) \
            if subprocess.call(['rosparam', 'get', '/depth_obstacles'], env=self.env, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL) == 0 else None
        subprocess.check_call(['rosparam', 'load', CFG1, '/depth_obstacles'], env=self.env)
        params.setdefault('store_path', self.store)
        for k, v in params.items():
            self.rospy.set_param('/depth_obstacles/' + k, v)
        with self.lock:
            self.config = None
        self.proc = subprocess.Popen(
            ['rosrun', 'vitulus_depth_obstacles', 'depth_obstacles', '__name:=depth_obstacles',
             'depth/image:=/test/depth', 'depth/camera_info:=/test/info'],
            env=self.env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        t_end = time.time() + 20
        while time.time() < t_end and (self.config is None or self.sets.get_num_connections() < 1):
            time.sleep(0.05)
        assert self.config is not None, 'no ~config'
        self.clear()
        self.cloud('/obstacles_cloud')   # frames flow

    def stop(self):
        if self.proc:
            os.killpg(self.proc.pid, 2)
            self.proc.wait(10)
            self.proc = None

    def close(self):
        self.feeding = False
        self.feeder.join(2)
        self.stop()
        self.rospy.signal_shutdown('done')
        os.killpg(self.core.pid, 2)
        self.core.wait(10)

    def clear(self):
        with self.lock:
            self.clouds.clear()

    def cloud(self, topic, timeout=5.0):
        """the next message of a topic, processed after the call"""
        with self.lock:
            self.clouds.pop(topic, None)
        t_end = time.time() + timeout
        while time.time() < t_end:
            with self.lock:
                if topic in self.clouds:
                    return self.clouds[topic]
            time.sleep(0.01)
        raise AssertionError('no ' + topic)

    def base(self, msg):
        """cloud in the base frame"""
        import sensor_msgs.point_cloud2 as pc2
        p = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)), float).reshape(-1, 3)
        return p @ self.r.T + self.o

    def send(self, wait=True, **cmd):
        """publish on ~set; returns the answer (config.last with this id)"""
        from std_msgs.msg import String
        self._id = getattr(self, '_id', 0) + 1
        cmd['id'] = self._id
        self.sets.publish(String(data=json.dumps(cmd)))
        if not wait:
            return None
        t_end = time.time() + 5
        while time.time() < t_end:
            with self.lock:
                last = self.config.get('last') if self.config else None
                if last and last.get('id') == cmd['id']:
                    return last
            time.sleep(0.005)
        raise AssertionError('no answer to %r' % cmd)

    def profile(self, name):
        with self.lock:
            return next(p for p in self.config['profiles'] if p['name'] == name)

    def stored(self):
        if not os.path.exists(self.store):
            return None
        return json.load(open(self.store))['profiles']


@pytest.fixture(scope='module')
def ros1(tmp_path_factory):
    # the private master, whatever the shell says: this test must never reach the robot
    os.environ['ROS_MASTER_URI'] = MASTER
    os.environ['ROS_HOSTNAME'] = '127.0.0.1'
    os.environ.pop('ROS_IP', None)
    n = Ros1Node(str(tmp_path_factory.mktemp('store')))
    yield n
    n.close()


@pytest.fixture
def fresh(ros1):
    """a node without stored settings"""
    ros1.stop()
    for f in (ros1.store, ros1.store + '.bad'):
        if os.path.exists(f):
            os.remove(f)
    ros1.start()
    return ros1


@only_ros1
def test_ros1_config(fresh):
    n = fresh
    assert len(n.config['schema']) == 22
    keys = {e['key'] for e in n.config['schema']}
    y = yaml.safe_load(open(CFG1))
    for name in ('nav', 'wide'):
        p = n.profile(name)
        assert p['safety'] == y[name]['safety']
        assert p['changed'] == [] and p['values'] == p['defaults'] and set(p['values']) == keys
        assert p['load_ms'] > 0
        for k in keys & set(y[name]):
            assert p['values'][k] == y[name][k], k
    mo = next(e for e in n.config['schema'] if e['key'] == 'max_depth')
    assert mo['safe'] == [1.5, 6.0] and 'off' not in mo
    assert n.config['store_error'] == ''


@only_ros1
def test_ros1_change_live_without_gap(fresh):
    n = fresh
    before = n.base(n.cloud('/obstacles_cloud'))
    assert before[:, 2].max() > 0.38
    wide_before = len(n.base(n.cloud('/mapping/obstacles_cloud_wide')))
    with n.lock:
        n.stamps.clear()
    pid = n.proc.pid
    last = n.send(profile='nav', values={'max_obstacle_height': 0.3})
    assert last['ok'] and last['changed'] == ['max_obstacle_height'] and last['errors'] == []
    assert n.profile('nav')['values']['max_obstacle_height'] == 0.3
    assert n.profile('nav')['changed'] == ['max_obstacle_height']
    assert n.profile('wide')['changed'] == []
    after = n.base(n.cloud('/obstacles_cloud'))
    assert len(after) > 3 and after[:, 2].max() <= 0.3 + 1e-3     # the top of the box is gone from this frame on
    assert len(n.base(n.cloud('/mapping/obstacles_cloud_wide'))) == wide_before   # the other profile not touched
    # mapping may go blind: the box 1.2 m ahead out of its depth window
    assert n.send(profile='wide', values={'max_depth': 0.8})['ok']
    assert len(n.base(n.cloud('/mapping/obstacles_cloud_wide'))) == 0
    assert len(n.base(n.cloud('/obstacles_cloud'))) > 3
    time.sleep(1.0)
    with n.lock:
        gaps = np.diff(n.stamps)
    assert n.proc.poll() is None and n.proc.pid == pid
    assert len(gaps) > 5 and gaps.max() < 0.5, gaps.max()


@only_ros1
def test_ros1_refused_changes_nothing(fresh):
    n = fresh
    before = n.profile('nav')['values']
    cases = [({'max_depth': 0.3}, 'max_depth', 'below_min'),
             ({'max_depth': 6.5}, 'max_depth', 'above_max'),
             ({'no_such': 1}, 'no_such', 'unknown_key'),
             ({'max_depth': 'far'}, 'max_depth', 'not_number'),
             ({'max_depth': True}, 'max_depth', 'not_number'),
             ({'normal_k': 7.5}, 'normal_k', 'not_integer'),
             ({'decimation': 3}, 'decimation', 'not_choice'),
             ({'normals_segmentation': 1}, 'normals_segmentation', 'not_bool'),
             ({'min_depth': 0.6, 'range_max': 1.5, 'max_depth': 1.5}, '', 'window_short'),
             ({'max_obstacle_height': 0.3, 'min_obstacle_height': 0.28}, 'max_obstacle_height', 'obstacle_window'),
             # one good, one bad: all or nothing
             ({'max_obstacle_height': 0.4, 'cluster_radius': 0.01}, 'cluster_radius', 'below_min')]
    for values, key, code in cases:
        last = n.send(profile='nav', values=values)
        assert not last['ok'], values
        assert (last['errors'][0]['key'], last['errors'][0]['code']) == (key, code), values
        assert last['changed'] == []
        assert n.profile('nav')['values'] == before and n.profile('nav')['changed'] == []
    e = n.send(profile='nav', values={'max_depth': 0.3})['errors'][0]
    assert (e['value'], e['a'], e['b']) == (0.3, 1.5, 6)
    assert n.send(profile='nope', values={})['errors'][0]['code'] == 'unknown_profile'
    assert n.send(cmd='fly', profile='nav')['errors'][0]['code'] == 'unknown_cmd'
    from std_msgs.msg import String
    n.sets.publish(String(data='{not json'))
    t_end = time.time() + 3
    while time.time() < t_end and (n.config['last'] or {}).get('errors', [{}])[0].get('code') != 'bad_json':
        time.sleep(0.01)
    assert n.config['last']['errors'][0]['code'] == 'bad_json'
    assert not os.path.exists(n.store), 'a refused change is not stored'
    assert len(n.base(n.cloud('/obstacles_cloud'))) > 3


@only_ros1
def test_ros1_nav_keeps_seeing(fresh):
    n = fresh
    last = n.send(profile='nav', values={'noise_filter_radius': 0.08, 'noise_filter_min_neighbors': 15})
    assert not last['ok'] and last['errors'][0]['code'] == 'blind'
    assert n.send(profile='nav', values={'min_cluster_size': 25})['errors'][0]['code'] == 'blind'
    # the same change on the mapping profile is the owner's call
    assert n.send(profile='wide', values={'min_cluster_size': 25})['ok']


@only_ros1
def test_ros1_set_is_atomic_between_frames(fresh):
    n = fresh
    A = {'max_obstacle_height': 0.30, 'max_ground_height': 0.10}
    B = {'max_obstacle_height': 0.40, 'max_ground_height': 0.20}

    def kind(z):
        return ('A' if z.min() < 0.15 else 'B') + ('A' if z.max() < 0.33 else 'B')

    assert n.send(profile='nav', values=A)['ok']
    n.cloud('/obstacles_cloud')
    assert kind(n.base(n.cloud('/obstacles_cloud'))[:, 2]) == 'AA'
    assert n.send(profile='nav', values=B)['ok']
    n.cloud('/obstacles_cloud')
    assert kind(n.base(n.cloud('/obstacles_cloud'))[:, 2]) == 'BB'
    seen = []
    stop = threading.Event()

    def watch():
        while not stop.is_set():
            try:
                seen.append(kind(n.base(n.cloud('/obstacles_cloud', 1.0))[:, 2]))
            except AssertionError:
                pass
    w = threading.Thread(target=watch)
    w.start()
    for i in range(60):
        n.send(profile='nav', values=A if i % 2 else B)
        time.sleep(0.03)
    stop.set()
    w.join(3)
    assert len(seen) > 20 and {'AA', 'BB'} <= set(seen), seen
    assert set(seen) == {'AA', 'BB'}, 'a frame with half of a set: %s' % sorted(set(seen))


@only_ros1
def test_ros1_settings_survive_a_restart(fresh):
    n = fresh
    assert n.send(profile='nav', values={'max_obstacle_height': 0.35, 'flat_obstacles_detected': False})['ok']
    assert n.send(profile='wide', values={'decimation': 4})['ok']
    assert n.stored() == {'nav': {'flat_obstacles_detected': False, 'max_obstacle_height': 0.35},
                          'wide': {'decimation': 4}}
    assert not os.path.exists(n.store + '.tmp')
    n.start()
    nav, wide = n.profile('nav'), n.profile('wide')
    assert sorted(nav['changed']) == ['flat_obstacles_detected', 'max_obstacle_height']
    assert nav['values']['max_obstacle_height'] == 0.35 and nav['values']['flat_obstacles_detected'] is False
    assert nav['defaults']['max_obstacle_height'] == 0.5
    assert wide['changed'] == ['decimation'] and wide['values']['decimation'] == 4
    assert n.base(n.cloud('/obstacles_cloud'))[:, 2].max() <= 0.35 + 1e-3
    # a new default in the YAML reaches what was not changed; the change stays
    n.rospy.set_param('/depth_obstacles/nav/max_ground_height', 0.12)
    n.rospy.set_param('/depth_obstacles/nav/max_obstacle_height', 0.45)
    subprocess.check_call(['rosservice', 'call', '/depth_obstacles/reload'], env=n.env, stdout=subprocess.DEVNULL)
    time.sleep(0.3)
    nav = n.profile('nav')
    assert nav['values']['max_ground_height'] == 0.12 and nav['values']['max_obstacle_height'] == 0.35
    assert nav['defaults']['max_obstacle_height'] == 0.45


@only_ros1
def test_ros1_reset(fresh):
    n = fresh
    assert n.send(profile='nav', values={'max_obstacle_height': 0.35, 'max_depth': 2.0})['ok']
    last = n.send(cmd='reset', profile='nav', keys=['max_depth'])
    assert last['ok'] and last['changed'] == ['max_depth']
    assert n.profile('nav')['values']['max_depth'] == 2.8 and n.profile('nav')['changed'] == ['max_obstacle_height']
    assert n.stored() == {'nav': {'max_obstacle_height': 0.35}}
    # reset + a value in one change
    last = n.send(profile='nav', values={'min_cluster_size': 3}, reset=['max_obstacle_height'])
    assert last['ok'] and sorted(last['changed']) == ['max_obstacle_height', 'min_cluster_size']
    assert n.profile('nav')['changed'] == ['min_cluster_size']
    assert n.send(profile='wide', values={'max_depth': 3.0})['ok']
    assert n.send(cmd='reset', profile='nav')['ok']
    nav = n.profile('nav')
    assert nav['changed'] == [] and nav['values'] == nav['defaults']
    assert n.profile('wide')['changed'] == ['max_depth'], 'the other profile keeps its changes'
    assert n.stored() == {'wide': {'max_depth': 3.0}}
    # setting the default value is no change
    assert n.send(profile='wide', values={'max_depth': 4.0})['ok']
    assert n.profile('wide')['changed'] == [] and n.stored() == {}


@only_ros1
def test_ros1_bad_store_falls_back(fresh):
    n = fresh
    n.stop()
    open(n.store, 'w').write('{half a file')
    n.start()
    assert n.config['store_error'] == 'corrupt store'
    assert n.profile('nav')['changed'] == [] and os.path.exists(n.store + '.bad')
    assert len(n.base(n.cloud('/obstacles_cloud'))) > 3
    # a file from other limits: a blind or out-of-range value is not loaded
    n.stop()
    json.dump({'version': 1, 'profiles': {'nav': {'max_depth': 0.3, 'max_obstacle_height': 0.4},
                                          'wide': {'min_cluster_size': 25}}}, open(n.store, 'w'))
    n.start()
    nav = n.profile('nav')
    assert nav['values']['max_depth'] == 2.8 and nav['changed'] == ['max_obstacle_height']
    assert 'dropped' in n.config['store_error']
    assert n.profile('wide')['changed'] == ['min_cluster_size']
    n.stop()
    json.dump({'version': 1, 'profiles': {'nav': {'min_cluster_size': 25}}}, open(n.store, 'w'))
    n.start()
    assert n.profile('nav')['changed'] == [] and n.profile('nav')['values']['min_cluster_size'] == 2


@only_ros1
def test_ros1_load_budget(fresh):
    n = fresh
    n.start(frame_budget_ms=0.5)
    assert n.config['budget_ms'] == 0.5
    last = n.send(profile='nav', values={'decimation': 1})
    assert not last['ok'] and last['errors'][0]['code'] == 'load'
    assert last['errors'][0]['value'] > 0.5 and last['errors'][0]['a'] == 0.5
    assert n.profile('nav')['values']['decimation'] == 8
    # about the same cost or lighter passes, even over the budget; so does the way back
    assert n.send(profile='nav', values={'max_obstacle_height': 0.45})['ok']
    assert n.send(profile='nav', values={'decimation': 16})['ok']
    assert n.send(cmd='reset', profile='nav')['ok']
    n.start()
    t_end = time.time() + 8
    while time.time() < t_end and not (n.stats and float(n.stats.get('nav.frames', 0)) > 0):
        time.sleep(0.1)
    s = n.stats
    assert float(s['frame_budget_ms']) == 50.0
    assert s['nav.rest_active'] == '1' and float(s['nav.obstacle_pts_avg']) > 0
    assert float(s['nav.ground_pts_avg']) > 0 and float(s['nav.rest_pts_avg']) >= 0
