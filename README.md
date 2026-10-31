# vitulus_depth_obstacles

Ground, obstacle and "rest" point clouds from a depth camera (Intel RealSense D435) in **one lightweight C++
node**, for ROS 1 Noetic and ROS 2 Jazzy. Built for the [Vitulus](https://github.com/lacina-dev/vitulus) robotic
lawn mower, where it replaces the RTAB-Map obstacle chains (`rtabmap_util/point_cloud_xyz` +
`rtabmap_util/obstacles_detection`, run twice).

- **Same output as RTAB-Map**: the segmentation reproduces RTAB-Map's algorithm step by step on the same PCL
  classes (see [NOTICE](NOTICE)); with the same parameters the clouds are identical point for point
  (measured, see [Results](#results)).
- **One process instead of four**: the depth image is decoded and projected once and shared by any number of
  *profiles* (e.g. `nav` for navigation, `wide` for mapping), each with its own depth window, segmentation
  parameters and topics.
- **Lighter**: ~10x less memory, ~25 % less CPU per frame, and it keeps up with the camera rate.
- **More to see**: a third cloud, the **rest** - points that are neither ground nor obstacle, each tagged with
  the reason (too high, tall grass / slope below the obstacle floor, isolated points).
- **Configurable while it runs**: readable parameter names; ROS 1: every segmentation parameter of every profile
  live over `~set` / `~config`, validated, atomic between frames, remembered across restarts ([Live
  settings](#live-settings-ros-1)); ROS 2: `ros2 param set` with validation. `~stats` diagnostics (time per stage,
  point counts, TF failures).
- No RTAB-Map library and no Docker needed.

## Layout

One package for both ROS versions: `package.xml` (format 3) and `CMakeLists.txt` pick catkin or ament from
`$ROS_VERSION` (set by the distribution's `setup.bash`).

| file | ROS 1 | ROS 2 |
|---|---|---|
| `src/segmentation.cpp`, `include/.../segmentation.hpp` | the algorithm, no ROS | same |
| `src/live_config.cpp`, `include/.../live_config.hpp` | parameter table, limits, validation, reference scene; no ROS | built, not used by the node |
| node | `src/ros1_node.cpp` | `src/ros2_node.cpp` (also the component `vitulus_depth_obstacles::DepthObstaclesNode`) |
| launch | `launch/depth_obstacles.launch` | `launch/depth_obstacles.launch.py` |
| parameters | `config/d435_ros1.yaml` | `config/d435.yaml` |
| RTAB-Map reference for A/B | `launch/rtabmap_reference.launch` (outputs under `/ref/`), `tools/ab_ros1.py` | `launch/rtabmap_reference.launch.py`, `tools/ab_rtabmap.py` |
| tests | 14 gtest + 10 pytest (live settings on a private roscore, config = the RTAB-Map launch values) | 14 gtest + 3 pytest (node, runtime parameters, config = the RTAB-Map launch values) |

## Build

ROS 2 Jazzy:

```bash
cd ~/ros2_ws/src && git clone https://github.com/lacina-dev/vitulus_depth_obstacles.git
cd .. && rosdep install --from-paths src --ignore-src -y
colcon build --packages-select vitulus_depth_obstacles
colcon test --packages-select vitulus_depth_obstacles    # 14 gtest + 3 pytest (ctest: 2 + pytest)
```

ROS 1 Noetic:

```bash
cd ~/catkin_ws/src && git clone https://github.com/lacina-dev/vitulus_depth_obstacles.git
cd .. && catkin build vitulus_depth_obstacles
catkin build vitulus_depth_obstacles --catkin-make-args run_tests   # 14 gtest
source devel/setup.bash && python3 -m pytest src/vitulus_depth_obstacles/test/test_node.py   # own roscore, port 11820
```

## Run

ROS 2:

```bash
ros2 launch vitulus_depth_obstacles depth_obstacles.launch.py camera2:=d435
# inputs:  /d435/aligned_depth_to_color/image_raw + /d435/color/camera_info, TF base_link <- camera
rviz2 -d $(ros2 pkg prefix vitulus_depth_obstacles)/share/vitulus_depth_obstacles/config/depth_obstacles.rviz
ros2 param set /depth_obstacles nav.max_depth 3.0       # any numeric parameter, validated
```

ROS 1:

```bash
roslaunch vitulus_depth_obstacles depth_obstacles.launch depth:=/d435/aligned_depth_to_color/image_raw \
    camera_info:=/d435/color/camera_info      # defaults: the Vitulus RTABMAP_ODOM_SYNC outputs
rostopic pub -1 /depth_obstacles/set std_msgs/String \
    'data: "{\"profile\": \"nav\", \"values\": {\"max_depth\": 3.0}}"'      # live, see below
```

Outputs per profile (defaults in `config/d435.yaml`):

| profile | cloud (after voxel + noise filter) | obstacles | ground | rest |
|---|---|---|---|---|
| `nav` | `/cloudXYZ` | `/obstacles_cloud` | `/ground_cloud` | `/rest_cloud` |
| `wide` | `/mapping/cloudXYZ_wide` | `/mapping/obstacles_cloud_wide` | `/mapping/ground_cloud_wide` | `/mapping/rest_cloud_wide` |

Clouds are in the camera frame with the header of the depth image (as RTAB-Map). A cloud is computed only while
somebody subscribes to it. `rest` is `x y z intensity`, intensity = reason:

| intensity | reason |
|---|---|
| 1 | above `max_obstacle_height` (branches, things above the robot) |
| 2 | below `min_ground_height` (off by default) |
| 3 | not the ground surface, but below the obstacle floor (tall grass, slopes, small steps) |
| 4 | above the floor, but in a cluster smaller than `min_cluster_size` (isolated points) |

## Live settings (ROS 1)

The ROS parameters (`config/d435_ros1.yaml`) are the **defaults**. On top of them, 22 parameters of every profile
change while the node runs, with no restart and no gap in the clouds. The robot UI has this as Settings > Camera >
Obstacle detection.

| topic | type | |
|---|---|---|
| `~set` | `std_msgs/String` JSON | `{"cmd": "set", "id": 1, "profile": "nav", "values": {"max_depth": 3.0}, "reset": ["voxel_size"]}`; `{"cmd": "reset", "profile": "nav", "keys": ["max_depth"]}` (no `keys` = the whole profile); `{"cmd": "refresh"}` |
| `~config` | `std_msgs/String` JSON, latched | `profiles[]`: `name`, `safety`, `values`, `defaults`, `changed`, `load_ms`; `schema[]`: `key`, `group`, `unit`, `type`, `step`, `safe` and `free` `[min, max]`, `off`, `off_safe`, `off_free`, `choices`; `last`: the answer to the last change (`id`, `ok`, `changed`, `errors[]` with `key`, `code`, `value`, `a`, `b`, `other`); `budget_ms`, `store_path`, `store_error`, `image` |
| `~stats` | `diagnostic_msgs/DiagnosticStatus` | as before + `frame_budget_ms`, `<profile>.rest_active`; WARN over the budget |

A message is **all or nothing**. It runs in the callback queue of the one `ros::spin` thread, so it lands between two
frames, never inside one. A refused message changes nothing and `last.errors` says why.

**Why JSON and not dynamic_reconfigure**: limits differ per profile and the profiles are a parameter (a `.cfg` is
fixed at build time); a change has to be refused as a whole with a reason (dynamic_reconfigure only clamps, one
parameter at a time, and has no answer for rosbridge clients); the store and the reset per parameter would still
have to be written. It is the same shape as the other UI nodes of the robot.

**Stored** in `~store_path` (default `~/.vitulus/depth_obstacles/settings.json`, written to a temporary file and
renamed): only what differs from the defaults, so a new default in the YAML reaches everything not changed. A file
that does not parse is moved to `.bad`; a value outside the limits or a set that fails the checks below is dropped.
The node never fails to start because of the store. `~reload` re-reads the defaults and keeps the changes.

**Safety.** A profile with `safety: true` (the default; `nav` feeds the costmaps) gets narrower limits and one more
check, `safety: false` (`wide`) only the wide limits and the relations:

1. **Limits** per parameter (`src/live_config.cpp`, also in `~config`): `max_depth` >= 1.5 m, the height limits cannot
   be switched off, `max_ground_angle_deg` <= 50 (above that walls join the ground), a grid is mandatory.
2. **Relations**: min < max of every pair, the overlap of `min_depth..max_depth` and `range_min..range_max`, the
   obstacle window, and for `safety`: at least 1 m of distance and 0.1 m of obstacle height left, `cluster_radius` >=
   1.2 grid, `noise_filter_radius` >= 1.5 grid; `decimation` must divide the real image.
3. **Reference scene** (`safety` only): flat ground and a box 0.3 m wide and high, 1.3 m ahead of a level camera 0.29 m
   high, through the whole chain. The change is refused when the box gives fewer than 5 obstacle points (`blind`), or
   the ground fewer than 10 ground points or obstacles of its own (`phantom`). The limits alone cannot do this: e.g.
   `min_cluster_size: 19` or `noise_filter_radius: 0.08` + `noise_filter_min_neighbors: 15` are inside them and blind.
4. **Load**: the time of one reference frame of the changed profile + the others against `frame_budget_ms` (default 50;
   the camera gives a frame every 66 ms). Over the budget and 20 % more than the running settings = refused (`load`),
   so a change can pass on a robot that is busy with something else. A reset to the defaults is never refused.

The reference scene proves the chain still works, not that a real thin obstacle is seen. Stored settings are not
held against the load at start (the boot is the worst moment to measure); the real time per frame is in `~stats`.

## Algorithm

Per frame (profile parameters in brackets):

1. Depth image -> points every `decimation` pixels, pinhole model from `camera_info` (P, else K); keep
   `min_depth <= z <= max_depth`.
2. Voxel grid (`voxel_size`), radius outlier filter (`noise_filter_radius`, `noise_filter_min_neighbors`) -> *cloud*.
3. Drop points beyond `range_max` from the sensor, transform to `frame_id` (TF at the image stamp,
   `wait_for_transform`), voxel grid again (`cell_size`), keep `min_ground_height <= z <= max_obstacle_height`.
4. Normals (`normal_k` neighbours, oriented to the camera); points whose normal is within `max_ground_angle_deg`
   of vertical are *flat*. Flat points are clustered (`cluster_radius`, `min_cluster_size`); the biggest cluster
   starting below `max_ground_height` is the ground, other flat clusters whose centroid is not higher join it,
   the rest are flat obstacles.
5. Everything that is not ground and is at least `max_ground_height` (or `min_obstacle_height`) high is clustered
   again; clusters of `min_cluster_size` or more are *obstacles*. What is left is *rest*.
6. Ground, obstacles and rest are transformed back to the camera frame.

Parameter names map 1:1 to RTAB-Map's (`Grid/MaxGroundHeight` -> `max_ground_height`, `Grid/MaxGroundAngle` ->
`max_ground_angle_deg`, ...; table in the comments of `config/d435.yaml`). Note: `Grid/MinObstacleHeight`, often
seen in launch files, is **not** an RTAB-Map parameter and is ignored there; here `min_obstacle_height: -1`
(default) keeps RTAB-Map's behaviour, a value >= 0 raises the obstacle floor.

## Results

A/B against RTAB-Map on the same frames: every frame both produced, all six clouds compared point by point.

| | data | frames compared | identical | max point distance |
|---|---|---|---|---|
| ROS 2, rtabmap 0.23.13 | 3065 D435 frames, garden simulation, 91 views of 11 objects | 1065 / 1084 per topic | **all** | 0.36 um (float rounding of the TF) |
| ROS 1, rtabmap 0.21.13 | live Gazebo garden (vitulus_sim) with the full robot stack, robot moved to 240 views of 30 objects, 8116 frames | 8114 - 8116 per topic (0.98 M obstacle, 2.4 M ground, 5.8 M cloud points in the nav profile) | **all** | 0.48 um |

Frames compared = frames both sides answered (RTAB-Map dropped many frames in the ROS 2 run, see below).

Resources, ROS 2, the same 864 frames at 14.4 Hz (desktop PC, other load present):

| | processes | memory | CPU per processed frame | frames processed |
|---|---|---|---|---|
| RTAB-Map chains | 4 | 616 MB | 10.7 ms | 289-316 / 864 |
| depth_obstacles | 1 | 63 MB | 7.8 ms | 841-843 / 864 |

On the Vitulus robot (Intel NUC 8 i7, ROS 1, D435 at 15 Hz, both profiles): 6.1 ms per frame on average,
~9 % of one core, 52 MB.

The live settings do not touch `segmentation.cpp`: with the defaults the clouds stay byte for byte what they were.

Reproduce: `tools/ab_rtabmap.py parity|cpu BAG` (starts both pipelines in their own
ROS_DOMAIN_ID; the RTAB-Map side is `launch/rtabmap_reference.launch.py`); ROS 1: `tools/ab_ros1.py` next to
`launch/rtabmap_reference.launch` in a running Gazebo world.

## License

MIT, see [LICENSE](LICENSE). The segmentation follows RTAB-Map (BSD-3-Clause), see [NOTICE](NOTICE).
