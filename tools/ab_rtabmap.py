#!/usr/bin/env python3
"""A/B: vitulus_depth_obstacles vs the RTAB-Map chains (rtabmap_util point_cloud_xyz + obstacles_detection) on the
same depth frames.

    ab_rtabmap.py parity BAG [--frames N] [--every K] [--rate 5] [--out report.json]
        frames at a fixed rate (default 5/s, below what both sides process), the 6 outputs of both collected by
        header stamp, compared point by point.
    ab_rtabmap.py cpu BAG [--seconds S] [--rate R] [--out report.json]
        real rate (R x the recorded rate): CPU time and RSS of the RTAB-Map processes vs the new node, run one after
        the other on the same frames.

BAG = rosbag2 with the D435 aligned depth, colour camera_info and /tf_static (e.g. a vsim2 recording).
Both pipelines run as subprocesses in their own ROS_DOMAIN_ID (default 53, LOCALHOST discovery), so nothing else on
the PC sees them. The RTAB-Map side is `ros2 launch vitulus_depth_obstacles rtabmap_reference.launch.py` (the
rtabmap_docker chains, same values); the new side is `depth_obstacles` with config/d435.yaml.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import numpy as np

DEPTH = '/d435/aligned_depth_to_color/image_raw'
INFO = '/d435/color/camera_info'
TOPICS = {  # rtabmap topic -> new topic
    '/cloudXYZ': '/new/cloudXYZ', '/obstacles_cloud': '/new/obstacles_cloud', '/ground_cloud': '/new/ground_cloud',
    '/mapping/cloudXYZ_wide': '/new/cloudXYZ_wide', '/mapping/obstacles_cloud_wide': '/new/obstacles_cloud_wide',
    '/mapping/ground_cloud_wide': '/new/ground_cloud_wide'}
NEW_OVERRIDE = """depth_obstacles:
  ros__parameters:
    depth_topic: %s
    camera_info_topic: %s
    nav: {cloud_topic: /new/cloudXYZ, obstacles_topic: /new/obstacles_cloud, ground_topic: /new/ground_cloud}
    wide: {cloud_topic: /new/cloudXYZ_wide, obstacles_topic: /new/obstacles_cloud_wide,
           ground_topic: /new/ground_cloud_wide}
""" % (DEPTH, INFO)


def read_bag(path, limit=None, every=1):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    statics, frames, info = [], [], None
    n = 0
    while r.has_next():
        topic, data, _t = r.read_next()
        if topic == '/tf_static':
            statics.extend(deserialize_message(data, TFMessage).transforms)
        elif topic == INFO:
            info = deserialize_message(data, CameraInfo)
        elif topic == DEPTH and info is not None:
            n += 1
            if (n - 1) % every:
                continue
            img = deserialize_message(data, Image)
            ci = CameraInfo()
            ci.header, ci.height, ci.width = img.header, info.height, info.width
            ci.distortion_model, ci.d, ci.k, ci.r, ci.p = info.distortion_model, info.d, info.k, info.r, info.p
            frames.append((img, ci))   # camera_info with the depth stamp: deterministic approx-sync pairing
            if limit and len(frames) >= limit:
                break
    return statics, frames


def env_for(domain):
    e = dict(os.environ)
    e['ROS_DOMAIN_ID'] = str(domain)
    e['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    e['RCUTILS_LOGGING_BUFFERED_STREAM'] = '1'
    return e


def start(cmd, domain, log):
    return subprocess.Popen(cmd, env=env_for(domain), stdout=open(log, 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)


def stop(p):
    if p and p.poll() is None:
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()


def rtabmap_cmd():
    return ['ros2', 'launch', 'vitulus_depth_obstacles', 'rtabmap_reference.launch.py']


def new_cmd(tmp):
    from ament_index_python.packages import get_package_share_directory
    cfg = os.path.join(get_package_share_directory('vitulus_depth_obstacles'), 'config', 'd435.yaml')
    ov = os.path.join(tmp, 'override.yaml')
    with open(ov, 'w') as f:
        f.write(NEW_OVERRIDE)
    return ['ros2', 'run', 'vitulus_depth_obstacles', 'depth_obstacles', '--ros-args', '-r', '__node:=depth_obstacles',
            '--params-file', cfg, '--params-file', ov]


def cloud_xyz(msg):
    from sensor_msgs_py import point_cloud2
    if msg.width * msg.height == 0:
        return np.zeros((0, 3), np.float32)
    a = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.asarray(a, dtype=np.float32).reshape(-1, 3)


def compare(a, b, tol=1e-3):
    from scipy.spatial import cKDTree
    out = {'n_ref': int(len(a)), 'n_new': int(len(b))}
    if len(a) == 0 or len(b) == 0:
        out.update(exact=int(len(a) == len(b) == 0), miss_ref=int(len(a)), miss_new=int(len(b)), max_nn=0.0)
        if len(a) == len(b):
            out.update(miss_ref=0, miss_new=0)
        return out
    sa = {tuple(p) for p in a.view(np.uint32).reshape(-1, 3).tolist()}
    sb = {tuple(p) for p in b.view(np.uint32).reshape(-1, 3).tolist()}
    out['bitwise_common'] = len(sa & sb)
    da, _ = cKDTree(b).query(a)
    db, _ = cKDTree(a).query(b)
    out['miss_ref'] = int((da > tol).sum())     # rtabmap points with no new point within tol
    out['miss_new'] = int((db > tol).sum())     # new points with no rtabmap point within tol
    out['max_nn'] = float(max(da.max(), db.max()))
    out['exact'] = int(out['miss_ref'] == 0 and out['miss_new'] == 0 and len(a) == len(b))
    return out


class Harness:
    def __init__(self, statics, domain):
        os.environ['ROS_DOMAIN_ID'] = str(domain)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        from tf2_msgs.msg import TFMessage
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('ab_rtabmap_player')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.tf_pub = self.node.create_publisher(TFMessage, '/tf_static', latched)
        self.tf_pub.publish(TFMessage(transforms=statics))
        self.depth_pub = self.node.create_publisher(Image, DEPTH, 10)
        self.info_pub = self.node.create_publisher(CameraInfo, INFO, 10)
        self.lock = threading.Lock()
        self.got = {}
        for t in list(TOPICS) + list(TOPICS.values()):
            self.node.create_subscription(PointCloud2, t, lambda m, t=t: self._cb(t, m), 10)
        self.exe = rclpy.executors.SingleThreadedExecutor()
        self.exe.add_node(self.node)
        self.th = threading.Thread(target=self.exe.spin, daemon=True)
        self.th.start()

    def _cb(self, topic, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self.lock:
            self.got.setdefault(key, {})[topic] = msg

    def subscribers_ready(self, n_each=1, timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if (self.depth_pub.get_subscription_count() >= n_each and
                    self.info_pub.get_subscription_count() >= n_each):
                return True
            time.sleep(0.2)
        return False

    def send(self, img, ci):
        self.depth_pub.publish(img)
        self.info_pub.publish(ci)

    def wait_for(self, key, topics, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self.lock:
                have = self.got.get(key, {})
                if all(t in have for t in topics):
                    return self.got.pop(key)
            time.sleep(0.002)
        with self.lock:
            return self.got.pop(key, {})

    def close(self):
        self.exe.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown()


def parity(a):
    statics, frames = read_bag(a.bag, a.frames, a.every)
    print('%d frames, %d static transforms' % (len(frames), len(statics)), flush=True)
    tmp = tempfile.mkdtemp(prefix='ab_rtabmap_')
    h = Harness(statics, a.domain)
    pr = start(rtabmap_cmd(), a.domain, os.path.join(tmp, 'rtabmap.log'))
    pn = start(new_cmd(tmp), a.domain, os.path.join(tmp, 'new.log'))
    try:
        # point_cloud_xyz x2 + depth_obstacles = 3 depth subscribers
        if not h.subscribers_ready(3, 90):
            sys.exit('pipelines did not subscribe (logs in %s)' % tmp)
        time.sleep(3.0)    # TF static + discovery of the outputs
        totals = {t: {'frames': 0, 'exact': 0, 'n_ref': 0, 'n_new': 0, 'miss_ref': 0, 'miss_new': 0,
                      'max_nn': 0.0, 'missing_ref': 0, 'missing_new': 0} for t in TOPICS}
        per_frame = []
        # fixed rate, outputs collected by stamp (a lock-step wait stalls: approximate sync may hold a set back
        # until the next frame arrives)
        t0 = time.time()
        for i, (img, ci) in enumerate(frames):
            h.send(img, ci)
            time.sleep(max(0.0, t0 + (i + 1) / a.rate - time.time()))
            if (i + 1) % 200 == 0:
                print('sent %d/%d' % (i + 1, len(frames)), flush=True)
        time.sleep(a.timeout)
        with h.lock:
            got = dict(h.got)
        for i, (img, ci) in enumerate(frames):
            key = (img.header.stamp.sec, img.header.stamp.nanosec)
            have = got.get(key, {})
            row = {'i': i, 'stamp': key[0] + key[1] * 1e-9}
            for t, tn in TOPICS.items():
                tt = totals[t]
                if t not in have:
                    tt['missing_ref'] += 1
                if tn not in have:
                    tt['missing_new'] += 1
                if t not in have or tn not in have:
                    continue
                c = compare(cloud_xyz(have[t]), cloud_xyz(have[tn]))
                row[t] = c
                tt['frames'] += 1
                tt['exact'] += c['exact']
                for k in ('n_ref', 'n_new', 'miss_ref', 'miss_new'):
                    tt[k] += c[k]
                tt['max_nn'] = max(tt['max_nn'], c['max_nn'])
            per_frame.append(row)
    finally:
        stop(pn)
        stop(pr)
        h.close()
    rep = {'bag': a.bag, 'frames': len(frames), 'every': a.every, 'tolerance_m': 1e-3, 'totals': totals,
           'per_frame': per_frame, 'logs': tmp}
    print(json.dumps(totals, indent=1))
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(rep, f)
    bad = [t for t, v in totals.items() if v['frames'] == 0 or v['missing_new'] > v['missing_ref']]
    return 1 if bad else 0


def proc_tree(pid):
    out, todo = [], [pid]
    while todo:
        p = todo.pop()
        out.append(p)
        try:
            with open('/proc/%d/task/%d/children' % (p, p)) as f:
                todo += [int(x) for x in f.read().split()]
        except OSError:
            pass
    return out


def cpu_rss(pids):
    tck = os.sysconf('SC_CLK_TCK')
    cpu, rss = 0.0, 0
    for p in pids:
        try:
            with open('/proc/%d/stat' % p) as f:
                s = f.read().rsplit(')', 1)[1].split()
            cpu += (int(s[11]) + int(s[12])) / tck
            with open('/proc/%d/status' % p) as f:
                for line in f:
                    if line.startswith('VmRSS:'):
                        rss += int(line.split()[1])
        except (OSError, IndexError, ValueError):
            pass
    return cpu, rss


def cpu(a):
    statics, frames = read_bag(a.bag, None, 1)
    stamps = [f[0].header.stamp.sec + f[0].header.stamp.nanosec * 1e-9 for f in frames]
    period = (stamps[-1] - stamps[0]) / max(1, len(stamps) - 1) / a.rate
    n = min(len(frames), int(a.seconds / period))
    print('%d frames at %.1f Hz' % (n, 1.0 / period), flush=True)
    tmp = tempfile.mkdtemp(prefix='ab_cpu_')
    h = Harness(statics, a.domain)
    res = {}
    try:
        for name, cmd, nsub in (('rtabmap', rtabmap_cmd(), 2), ('new', new_cmd(tmp), 1)):
            p = start(cmd, a.domain, os.path.join(tmp, name + '.log'))
            if not h.subscribers_ready(nsub, 90):
                sys.exit('%s did not subscribe' % name)
            time.sleep(3.0)
            pids = [x for x in proc_tree(p.pid) if x != p.pid]
            c0, _ = cpu_rss(pids)
            t0 = time.time()
            got0 = 0
            rss_max = 0
            for i in range(n):
                img, ci = frames[i]
                h.send(img, ci)
                if i % 10 == 0:
                    rss_max = max(rss_max, cpu_rss(pids)[1])
                time.sleep(max(0.0, t0 + (i + 1) * period - time.time()))
            time.sleep(1.0)
            c1, rss = cpu_rss(pids)
            wall = time.time() - t0
            with h.lock:
                outs = {}
                for v in h.got.values():
                    for t in v:
                        outs[t] = outs.get(t, 0) + 1
                h.got.clear()
            res[name] = {'cpu_s': round(c1 - c0, 2), 'wall_s': round(wall, 1),
                         'cpu_percent_of_one_core': round(100 * (c1 - c0) / wall, 1),
                         'rss_mb': round(max(rss, rss_max) / 1024, 1), 'processes': len(pids),
                         'outputs': outs}
            stop(p)
            time.sleep(2.0)
            print(name, json.dumps(res[name]), flush=True)
    finally:
        h.close()
    rep = {'bag': a.bag, 'frames': n, 'rate_hz': round(1.0 / period, 2), 'load_before': os.getloadavg(), **res}
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(rep, f, indent=1)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest='mode', required=True)
    p = sp.add_parser('parity')
    p.add_argument('bag')
    p.add_argument('--frames', type=int)
    p.add_argument('--every', type=int, default=1)
    p.add_argument('--timeout', type=float, default=5.0)
    p.add_argument('--rate', type=float, default=5.0, help='frames/s sent')
    p.add_argument('--domain', type=int, default=53)
    p.add_argument('--out')
    c = sp.add_parser('cpu')
    c.add_argument('bag')
    c.add_argument('--seconds', type=float, default=60.0)
    c.add_argument('--rate', type=float, default=1.0)
    c.add_argument('--domain', type=int, default=53)
    c.add_argument('--out')
    a = ap.parse_args()
    sys.exit(parity(a) if a.mode == 'parity' else cpu(a))


if __name__ == '__main__':
    main()
