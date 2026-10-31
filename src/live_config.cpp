// See live_config.hpp.
#include "vitulus_depth_obstacles/live_config.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

namespace vitulus_depth_obstacles
{

namespace
{
const char * const G_DIST = "distance";
const char * const G_HEIGHT = "height";
const char * const G_GROUND = "ground";
const char * const G_NOISE = "noise";
const char * const G_CLUSTER = "clusters";
const char * const G_PERF = "performance";

// The safe window keeps the detection alive, not merely the node: far enough to stop in front of something
// (max_depth, range_max, up to 6 m set by the owner 2026-10-31), obstacles between the ground and the robot's height (no 0 = off on the heights),
// a voxel grid, ground that does not swallow walls (max_ground_angle_deg < 90).
#define PARAM_D(KEY, GROUP, UNIT, STEP, S_MIN, S_MAX, S_OFF, F_MIN, F_MAX, F_OFF, HAS_OFF, OFF) \
  ParamDef{#KEY, GROUP, UNIT, ParamType::Double, STEP, {S_MIN, S_MAX, S_OFF}, {F_MIN, F_MAX, F_OFF}, HAS_OFF, OFF, {}, \
           [](const ProfileSettings & s) -> double { return s.p.KEY; }, \
           [](ProfileSettings & s, double v) { s.p.KEY = v; }}
#define PARAM_I(KEY, GROUP, S_MIN, S_MAX, F_MIN, F_MAX) \
  ParamDef{#KEY, GROUP, "", ParamType::Int, 1, {S_MIN, S_MAX, false}, {F_MIN, F_MAX, false}, false, 0, {}, \
           [](const ProfileSettings & s) -> double { return s.p.KEY; }, \
           [](ProfileSettings & s, double v) { s.p.KEY = static_cast<int>(v); }}
#define PARAM_B(KEY, GROUP) \
  ParamDef{#KEY, GROUP, "", ParamType::Bool, 1, {0, 1, false}, {0, 1, false}, false, 0, {}, \
           [](const ProfileSettings & s) -> double { return s.p.KEY ? 1.0 : 0.0; }, \
           [](ProfileSettings & s, double v) { s.p.KEY = v != 0.0; }}

std::vector<ParamDef> make_table()
{
  std::vector<ParamDef> t;
  //                                       step  safe min, max, off  free min, max, off  has off, off
  t.push_back(PARAM_D(min_depth, G_DIST, "m", 0.01, 0.0, 0.6, false, 0.0, 2.0, false, false, 0.0));
  t.push_back(PARAM_D(max_depth, G_DIST, "m", 0.05, 1.5, 6.0, false, 0.5, 6.0, false, false, 0.0));
  t.push_back(PARAM_D(range_min, G_DIST, "m", 0.01, 0.0, 0.6, false, 0.0, 2.0, false, false, 0.0));
  t.push_back(PARAM_D(range_max, G_DIST, "m", 0.05, 1.5, 6.0, true, 0.5, 6.0, true, true, 0.0));

  t.push_back(PARAM_D(min_ground_height, G_HEIGHT, "m", 0.01, -1.0, -0.05, true, -1.0, 0.3, true, true, 0.0));
  t.push_back(PARAM_D(max_ground_height, G_HEIGHT, "m", 0.01, 0.05, 0.3, false, 0.02, 0.6, true, true, 0.0));
  t.push_back(ParamDef{"min_obstacle_height", G_HEIGHT, "m", ParamType::Double, 0.01, {0.0, 0.3, true}, {0.0, 1.0, true},
                       true, -1.0, {},
                       [](const ProfileSettings & s) -> double {
                         return std::isnan(s.p.min_obstacle_height) ? -1.0 : s.p.min_obstacle_height;
                       },
                       [](ProfileSettings & s, double v) {
                         s.p.min_obstacle_height = v < 0.0 ? std::numeric_limits<double>::quiet_NaN() : v;
                       }});
  t.push_back(PARAM_D(max_obstacle_height, G_HEIGHT, "m", 0.05, 0.3, 1.5, false, 0.2, 4.0, true, true, 0.0));

  t.push_back(PARAM_D(max_ground_angle_deg, G_GROUND, "deg", 1.0, 10.0, 50.0, false, 5.0, 85.0, false, false, 0.0));
  t.push_back(PARAM_B(normals_segmentation, G_GROUND));
  t.push_back(PARAM_I(normal_k, G_GROUND, 5, 50, 3, 100));
  t.push_back(PARAM_D(ground_normals_up, G_GROUND, "", 0.05, 0.0, 0.95, false, 0.0, 0.95, false, false, 0.0));
  t.push_back(PARAM_B(flat_obstacles_detected, G_GROUND));

  t.push_back(PARAM_D(voxel_size, G_NOISE, "m", 0.01, 0.02, 0.15, false, 0.01, 0.3, true, true, 0.0));
  t.push_back(PARAM_D(cell_size, G_NOISE, "m", 0.01, 0.02, 0.15, false, 0.01, 0.3, false, false, 0.0));
  t.push_back(PARAM_B(pre_voxel_filtering, G_NOISE));
  t.push_back(PARAM_D(noise_filter_radius, G_NOISE, "m", 0.01, 0.05, 0.5, true, 0.02, 1.0, true, true, 0.0));
  t.push_back(PARAM_I(noise_filter_min_neighbors, G_NOISE, 1, 15, 1, 50));

  t.push_back(PARAM_D(cluster_radius, G_CLUSTER, "m", 0.01, 0.05, 0.5, false, 0.02, 1.0, false, false, 0.0));
  t.push_back(PARAM_I(min_cluster_size, G_CLUSTER, 1, 30, 1, 500));

  // a decimation that does not divide the image publishes nothing: only the powers of 2 that divide 640x480,
  // 848x480 and 1280x720
  t.push_back(ParamDef{"decimation", G_PERF, "px", ParamType::Int, 1, {1, 16, false}, {1, 16, false}, false, 0,
                       {1, 2, 4, 8, 16},
                       [](const ProfileSettings & s) -> double { return s.decimation; },
                       [](ProfileSettings & s, double v) { s.decimation = static_cast<int>(v); }});
  t.push_back(PARAM_I(normal_threads, G_PERF, 1, 4, 1, 4));
  return t;
}

// optical frame (z forward, x right, y down) -> base frame: the mount of the Vitulus D435 (measured TF 2026-10-31)
Eigen::Isometry3d reference_camera()
{
  Eigen::Matrix3d opt_to_link;
  opt_to_link << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(0.21, 0.06, 0.29);
  t.linear() = opt_to_link;
  return t;
}

const int REF_W = 640, REF_H = 480;
const double REF_F = 615.0, REF_CX = 319.5, REF_CY = 239.5;
const double BOX_X = 1.5, BOX_HALF_W = 0.15, BOX_H = 0.3;
const std::size_t MIN_BOX_POINTS = 5, MIN_GROUND_POINTS = 10;

Intrinsics reference_intrinsics()
{
  Intrinsics k;
  k.fx = k.fy = REF_F;
  k.cx = REF_CX;
  k.cy = REF_CY;
  k.width = REF_W;
  k.height = REF_H;
  return k;
}

// ray-cast depth image [mm]
std::vector<std::uint16_t> render(bool box)
{
  const Eigen::Isometry3d cam = reference_camera();
  const Eigen::Vector3d o = cam.translation();
  std::vector<std::uint16_t> img(REF_W * REF_H, 0);
  for (int v = 0; v < REF_H; ++v) {
    for (int u = 0; u < REF_W; ++u) {
      const Eigen::Vector3d d = cam.linear() * Eigen::Vector3d((u - REF_CX) / REF_F, (v - REF_CY) / REF_F, 1.0);
      double best = 1e9;
      if (d.z() < -1e-6) best = -o.z() / d.z();
      if (box && d.x() > 1e-6) {
        const double s = (BOX_X - o.x()) / d.x();
        const Eigen::Vector3d p = o + s * d;
        if (std::abs(p.y() - o.y()) < BOX_HALF_W && p.z() >= 0.0 && p.z() <= BOX_H && s < best) best = s;
      }
      if (best < 6.0) img[v * REF_W + u] = static_cast<std::uint16_t>(std::lround(best * 1000.0));
    }
  }
  return img;
}

void run_chain(const std::vector<std::uint16_t> & img, const ProfileSettings & s, Cloud::Ptr & ground,
               Cloud::Ptr & obstacles)
{
  DepthPoints dp;
  ground.reset(new Cloud);
  obstacles.reset(new Cloud);
  if (project_depth(img.data(), true, REF_H, REF_W, REF_W * 2, reference_intrinsics(), s.decimation, dp)) {
    segment(make_cloud(dp, s.p), reference_camera(), s.p, ground, obstacles);
  }
}

Issue issue(const std::string & key, const std::string & code, double value = 0, double a = 0, double b = 0,
            const std::string & other = "")
{
  Issue i;
  i.key = key;
  i.code = code;
  i.value = value;
  i.a = a;
  i.b = b;
  i.other = other;
  return i;
}
}  // namespace

const std::vector<ParamDef> & param_table()
{
  static const std::vector<ParamDef> table = make_table();
  return table;
}

const ParamDef * find_param(const std::string & key)
{
  for (const auto & d : param_table()) {
    if (key == d.key) return &d;
  }
  return nullptr;
}

ParamValues to_values(const ProfileSettings & s)
{
  ParamValues out;
  for (const auto & d : param_table()) out[d.key] = d.get(s);
  return out;
}

ProfileSettings with_values(ProfileSettings base, const ParamValues & values)
{
  for (const auto & kv : values) {
    const ParamDef * d = find_param(kv.first);
    if (d) d->set(base, kv.second);
  }
  return base;
}

bool check_value(const ParamDef & def, double value, bool safety, std::vector<Issue> & issues)
{
  const ParamLimits & l = safety ? def.safe : def.free;
  if (!std::isfinite(value)) {
    issues.push_back(issue(def.key, "not_number"));
    return false;
  }
  if (def.type == ParamType::Bool && value != 0.0 && value != 1.0) {
    issues.push_back(issue(def.key, "not_bool", value));
    return false;
  }
  if (def.type == ParamType::Int && value != std::floor(value)) {
    issues.push_back(issue(def.key, "not_integer", value));
    return false;
  }
  if (def.has_off && value == def.off) {
    if (l.off_allowed) return true;
    issues.push_back(issue(def.key, "off_refused", value, l.min, l.max));
    return false;
  }
  if (!def.choices.empty() &&
      std::find(def.choices.begin(), def.choices.end(), static_cast<int>(value)) == def.choices.end()) {
    issues.push_back(issue(def.key, "not_choice", value, l.min, l.max));
    return false;
  }
  if (value < l.min) {
    issues.push_back(issue(def.key, "below_min", value, l.min, l.max));
    return false;
  }
  if (value > l.max) {
    issues.push_back(issue(def.key, "above_max", value, l.min, l.max));
    return false;
  }
  return true;
}

std::vector<Issue> check_relations(const ProfileSettings & s, bool safety, int rows, int cols)
{
  std::vector<Issue> out;
  const ChainParams & p = s.p;
  const auto order = [&](const char * low, double a, const char * high, double b) {
    out.push_back(issue(low, "order", a, a, b, high));
  };
  if (p.max_depth > 0.0 && p.min_depth >= p.max_depth) order("min_depth", p.min_depth, "max_depth", p.max_depth);
  if (p.range_max > 0.0 && p.range_min >= p.range_max) order("range_min", p.range_min, "range_max", p.range_max);
  if (p.min_ground_height != 0.0 && p.max_ground_height != 0.0 && p.min_ground_height >= p.max_ground_height) {
    order("min_ground_height", p.min_ground_height, "max_ground_height", p.max_ground_height);
  }
  double floor = p.max_ground_height;
  const char * floor_key = "max_ground_height";
  if (!std::isnan(p.min_obstacle_height) && p.min_obstacle_height > floor) {
    floor = p.min_obstacle_height;
    floor_key = "min_obstacle_height";
  }
  if (p.max_obstacle_height > 0.0 && floor >= p.max_obstacle_height) {
    order(floor_key, floor, "max_obstacle_height", p.max_obstacle_height);
  }
  // the two distance cuts act one after the other: what is left is their overlap
  const double near = std::max(p.min_depth, p.range_min);
  const double inf = std::numeric_limits<double>::infinity();
  const double far = std::min(p.max_depth > 0.0 ? p.max_depth : inf, p.range_max > 0.0 ? p.range_max : inf);
  if (out.empty() && near >= far) out.push_back(issue("", "window_short", far - near, 0.0));
  if (rows > 0 && cols > 0 && (rows % s.decimation != 0 || cols % s.decimation != 0)) {
    out.push_back(issue("decimation", "no_divide", s.decimation, cols, rows));
  }
  if (!safety) return out;
  const double min_window = 1.0, min_obstacle_window = 0.1;
  if (near < far && far - near < min_window) out.push_back(issue("", "window_short", far - near, min_window));
  if (p.max_obstacle_height - floor < min_obstacle_window && floor < p.max_obstacle_height) {
    out.push_back(issue("max_obstacle_height", "obstacle_window", p.max_obstacle_height - floor, min_obstacle_window));
  }
  // voxels are one grid step apart: closer than that nothing has a neighbour, every cluster / filter drops all
  const double grid = std::max(p.voxel_size, p.pre_voxel_filtering ? p.cell_size : 0.0);
  if (p.cluster_radius < 1.2 * grid) out.push_back(issue("cluster_radius", "cluster_small", p.cluster_radius, 1.2 * grid));
  if (p.noise_filter_radius > 0.0 && p.noise_filter_radius < 1.5 * p.voxel_size) {
    out.push_back(issue("noise_filter_radius", "noise_small", p.noise_filter_radius, 1.5 * p.voxel_size));
  }
  return out;
}

ReferenceResult run_reference(const ProfileSettings & s)
{
  static const std::vector<std::uint16_t> with_box = render(true), empty = render(false);
  ReferenceResult r;
  Cloud::Ptr ground, obstacles;
  const auto t0 = std::chrono::steady_clock::now();
  run_chain(with_box, s, ground, obstacles);
  r.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  r.ground_points = ground->size();
  const Eigen::Isometry3d cam = reference_camera();
  for (const auto & q : *obstacles) {
    const Eigen::Vector3d b = cam * Eigen::Vector3d(q.x, q.y, q.z);
    if (std::abs(b.x() - BOX_X) < 0.25 && std::abs(b.y() - cam.translation().y()) < BOX_HALF_W + 0.1) ++r.box_points;
  }
  run_chain(empty, s, ground, obstacles);
  r.phantom_points = obstacles->size();
  return r;
}

std::vector<Issue> check_reference(const ReferenceResult & r)
{
  std::vector<Issue> out;
  if (r.box_points < MIN_BOX_POINTS) out.push_back(issue("", "blind", static_cast<double>(r.box_points), MIN_BOX_POINTS));
  if (r.ground_points < MIN_GROUND_POINTS || r.phantom_points > std::max<std::size_t>(3, r.ground_points / 20)) {
    out.push_back(issue("", "phantom", static_cast<double>(r.phantom_points), static_cast<double>(r.ground_points)));
  }
  return out;
}

}  // namespace vitulus_depth_obstacles
