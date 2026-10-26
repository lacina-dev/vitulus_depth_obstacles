#!/usr/bin/env python3
"""ROS1 A/B in the vitulus_sim garden: vitulus_depth_obstacles (/obstacles_cloud ...) vs the RTAB-Map chain
(ref_obstacles_detection.launch = rtabmap_docker obstacles_detection.launch with outputs under /ref/) on the same
RTABMAP_ODOM_SYNC frames. The robot is teleported in front of the garden objects (4 directions x 2 distances).

    python3 ab_ros1.py OUT.json [DWELL_S]
"""
import json
import math
import sys
import threading
import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetWorldProperties, SetModelState
from sensor_msgs.msg import PointCloud2

OUT = sys.argv[1]
DWELL = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
PAIRS = {'/obstacles_cloud': '/ref/obstacles_cloud', '/ground_cloud': '/ref/ground_cloud',
         '/cloudXYZ': '/ref/cloudXYZ', '/mapping/obstacles_cloud_wide': '/ref/obstacles_cloud_wide',
         '/mapping/ground_cloud_wide': '/ref/ground_cloud_wide', '/mapping/cloudXYZ_wide': '/ref/cloudXYZ_wide'}
OBJ_HINTS = ('molehill', 'stump', 'branch', 'gnome', 'hose', 'sprinkler', 'hedgehog', 'pot', 'trowel', 'cone',
             'hydrant', 'trash', 'mailbox', 'lamp', 'chair', 'bench', 'toy_', 'ball', 'watering', 'rake', 'person',
             'compost', 'bush_1', 'bush_4', 'stones', 'sandbox', 'parasol', 'patio_table')

lock = threading.Lock()
got = {}


def xyz(m):
    if m.width * m.height == 0:
        return np.zeros((0, 3), np.float32)
    off = {f.name: f.offset for f in m.fields}
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4'] * 3,
                   'offsets': [off['x'], off['y'], off['z']], 'itemsize': m.point_step})
    a = np.frombuffer(m.data, dtype=dt, count=m.width * m.height)
    p = np.stack([a['x'], a['y'], a['z']], 1)
    return p[np.all(np.isfinite(p), 1)]


def cb(topic, m):
    with lock:
        got.setdefault(m.header.stamp.to_nsec(), {})[topic] = xyz(m)


def nn_max(a, b):
    """max over a of the distance to the nearest point of b (chunked brute force)"""
    if len(a) == 0:
        return 0.0
    best = 0.0
    for i in range(0, len(a), 512):
        d = np.sqrt(((a[i:i + 512, None, :] - b[None, :, :]) ** 2).sum(-1)).min(1)
        best = max(best, float(d.max()))
    return best


def compare(a, b):
    r = {'n_new': len(a), 'n_ref': len(b)}
    if len(a) == 0 or len(b) == 0:
        r['max_nn'] = 0.0 if len(a) == len(b) else float('inf')
    else:
        r['max_nn'] = max(nn_max(a, b), nn_max(b, a))
    r['same'] = int(len(a) == len(b) and r['max_nn'] <= 1e-3)
    return r


def main():
    rospy.init_node('ab_ros1', anonymous=True)
    for t in list(PAIRS) + list(PAIRS.values()):
        rospy.Subscriber(t, PointCloud2, lambda m, t=t: cb(t, m), queue_size=20)
    rospy.wait_for_service('/gazebo/set_model_state', 60)
    gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    sms = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    names = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)().model_names
    robot = 'vitulus'                      # (not vitulus_dock)
    assert robot in names, names
    objs = [n for n in names if any(h in n.lower() for h in OBJ_HINTS)]
    poses = []
    for o in objs:
        p = gms(o, '').pose.position
        for k in range(4):
            yaw = -math.pi + k * math.pi / 2
            for dist in (1.2, 2.2):
                poses.append((o, p.x - dist * math.cos(yaw), p.y - dist * math.sin(yaw), yaw))
    rospy.loginfo('ab_ros1: robot %s, %d objects, %d poses', robot, len(objs), len(poses))
    time.sleep(3.0)
    for o, x, y, yaw in poses:
        s = ModelState()
        s.model_name = robot
        s.pose.position.x, s.pose.position.y, s.pose.position.z = x, y, 0.15
        s.pose.orientation.z, s.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        sms(s)
        time.sleep(DWELL)
    time.sleep(2.0)
    with lock:
        frames = dict(got)
    totals = {t: {'compared': 0, 'same': 0, 'pts_new': 0, 'pts_ref': 0, 'max_nn': 0.0, 'only_new': 0,
                  'only_ref': 0} for t in PAIRS}
    for stamp, have in frames.items():
        for t, r in PAIRS.items():
            T = totals[t]
            if t in have and r not in have:
                T['only_new'] += 1
            elif r in have and t not in have:
                T['only_ref'] += 1
            elif t in have and r in have:
                c = compare(have[t], have[r])
                T['compared'] += 1
                T['same'] += c['same']
                T['pts_new'] += c['n_new']
                T['pts_ref'] += c['n_ref']
                T['max_nn'] = max(T['max_nn'], c['max_nn'])
    rep = {'robot': robot, 'objects': objs, 'poses': len(poses), 'frames': len(frames), 'totals': totals}
    json.dump(rep, open(OUT, 'w'), indent=1)
    print(json.dumps(totals, indent=1))


if __name__ == '__main__':
    main()
