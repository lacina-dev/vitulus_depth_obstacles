// Algorithm tests on synthetic scenes (no ROS). The RTAB-Map equality itself is measured by tools/ab_rtabmap.py.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "vitulus_depth_obstacles/live_config.hpp"
#include "vitulus_depth_obstacles/segmentation.hpp"

using namespace vitulus_depth_obstacles;

namespace
{
// D435 colour intrinsics of the sim (640x480)
Intrinsics k640()
{
  Intrinsics k;
  k.fx = k.fy = 615.0;
  k.cx = 319.5;
  k.cy = 239.5;
  k.width = 640;
  k.height = 480;
  return k;
}

ChainParams nav()
{
  ChainParams p;   // config/d435.yaml "nav"
  p.max_depth = 2.8;
  p.voxel_size = 0.05;
  p.noise_filter_radius = 0.12;
  p.noise_filter_min_neighbors = 5;
  p.max_obstacle_height = 0.5;
  p.max_ground_height = 0.15;
  p.max_ground_angle_deg = 35.0;
  p.cluster_radius = 0.15;
  p.min_cluster_size = 2;
  return p;
}

// camera 0.4 m above the ground, looking forward and 20 deg down: optical frame (z forward, x right, y down)
Eigen::Isometry3d camera_pose()
{
  Eigen::Matrix3d opt_to_link;
  opt_to_link << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(0.3, 0.0, 0.4);
  t.linear() = Eigen::AngleAxisd(20.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()).toRotationMatrix() * opt_to_link;
  return t;
}

// ray-cast depth image [mm] of flat ground + an optional box (x0..x1, y0..y1, height h) in the base frame
std::vector<std::uint16_t> render(const Eigen::Isometry3d & cam, bool box, double h = 0.3)
{
  const Intrinsics k = k640();
  std::vector<std::uint16_t> img(640 * 480, 0);
  const Eigen::Vector3d o = cam.translation();
  for (int v = 0; v < 480; ++v) {
    for (int u = 0; u < 640; ++u) {
      const Eigen::Vector3d ray_opt((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0);   // z = 1 (depth units)
      const Eigen::Vector3d d = cam.linear() * ray_opt;
      double best = 1e9;
      if (d.z() < -1e-6) best = -o.z() / d.z();                                    // ground z = 0
      if (box) {   // front face x = 1.5, 0.3 m wide, h high
        if (d.x() > 1e-6) {
          const double s = (1.5 - o.x()) / d.x();
          const Eigen::Vector3d p = o + s * d;
          if (s > 0 && std::abs(p.y()) < 0.15 && p.z() >= 0 && p.z() <= h && s < best) best = s;
        }
      }
      if (best < 6.0) img[v * 640 + u] = static_cast<std::uint16_t>(std::lround(best * 1000.0));
    }
  }
  return img;
}

// returned by value: a range-for over a temporary Cloud keeps it alive (over *Ptr it would dangle)
Cloud to_base(const Cloud & c, const Eigen::Isometry3d & cam)
{
  Cloud out;
  for (const auto & p : c) {
    const Eigen::Vector3d b = cam * Eigen::Vector3d(p.x, p.y, p.z);
    out.push_back(pcl::PointXYZ(b.x(), b.y(), b.z()));
  }
  return out;
}
}  // namespace

TEST(ProjectDepth, FormulaAndInvalidPixels)
{
  std::vector<std::uint16_t> img(16 * 8, 0);
  img[0] = 1000;                 // (0,0) 1 m
  img[2 * 16 + 4] = 2000;        // (u=4, v=2) 2 m
  img[4 * 16 + 8] = 65535;       // invalid (saturated)
  Intrinsics k;
  k.fx = 10; k.fy = 20; k.cx = 8; k.cy = 4; k.width = 16; k.height = 8;
  DepthPoints dp;
  ASSERT_TRUE(project_depth(img.data(), true, 8, 16, 16 * 2, k, 2, dp));
  ASSERT_EQ(dp.points.size(), 2u);
  EXPECT_FLOAT_EQ(dp.points[0].x, (0 - 8) * 1.0f / 10);
  EXPECT_FLOAT_EQ(dp.points[0].y, (0 - 4) * 1.0f / 20);
  EXPECT_FLOAT_EQ(dp.points[1].x, (4 - 8) * 2.0f / 10);
  EXPECT_FLOAT_EQ(dp.points[1].y, (2 - 4) * 2.0f / 20);
  EXPECT_FLOAT_EQ(dp.points[1].z, 2.0f);
  EXPECT_FALSE(project_depth(img.data(), true, 8, 16, 32, k, 3, dp));   // 3 does not divide 16
}

TEST(ProjectDepth, SmallerDepthImageScalesIntrinsics)
{
  std::vector<float> img(8 * 4, 0.0f);
  img[1 * 8 + 2] = 1.5f;
  Intrinsics k;
  k.fx = k.fy = 100; k.cx = 8; k.cy = 4; k.width = 16; k.height = 8;   // camera_info 2x the depth image
  DepthPoints dp;
  ASSERT_TRUE(project_depth(img.data(), false, 4, 8, 8 * 4, k, 1, dp));
  ASSERT_EQ(dp.points.size(), 1u);
  EXPECT_FLOAT_EQ(dp.points[0].x, (2 - 4.0f) * 1.5f / 50.0f);
}

TEST(Segment, FlatGroundIsGroundOnly)
{
  const auto cam = camera_pose();
  const auto img = render(cam, false);
  DepthPoints dp;
  ASSERT_TRUE(project_depth(img.data(), true, 480, 640, 640 * 2, k640(), 8, dp));
  const ChainParams p = nav();
  Cloud::Ptr cloud = make_cloud(dp, p), ground, obstacles;
  ASSERT_GT(cloud->size(), 100u);
  segment(cloud, cam, p, ground, obstacles);
  EXPECT_GT(ground->size(), 0.9 * cloud->size());
  EXPECT_EQ(obstacles->size(), 0u);
  for (const auto & q : to_base(*ground, cam)) EXPECT_NEAR(q.z, 0.0, 0.03);
}

TEST(Segment, BoxIsObstacleAboveGroundHeight)
{
  const auto cam = camera_pose();
  const auto img = render(cam, true, 0.3);
  DepthPoints dp;
  ASSERT_TRUE(project_depth(img.data(), true, 480, 640, 640 * 2, k640(), 8, dp));
  ChainParams p = nav();
  Cloud::Ptr cloud = make_cloud(dp, p), ground, obstacles;
  segment(cloud, cam, p, ground, obstacles);
  ASSERT_GT(obstacles->size(), 3u);
  for (const auto & q : to_base(*obstacles, cam)) {
    EXPECT_GE(q.z, 0.15 - 1e-4);       // obstacles start at max_ground_height (RTAB-Map behaviour)
    EXPECT_NEAR(q.x, 1.5, 0.06);
    EXPECT_LT(std::abs(q.y), 0.2);
  }
  // an explicit min_obstacle_height raises the floor
  p.min_obstacle_height = 0.25;
  segment(cloud, cam, p, ground, obstacles);
  ASSERT_GT(obstacles->size(), 0u);
  for (const auto & q : to_base(*obstacles, cam)) EXPECT_GE(q.z, 0.25 - 1e-4);
  // the box is lower than a raised floor -> nothing
  p.min_obstacle_height = 0.4;
  segment(cloud, cam, p, ground, obstacles);
  EXPECT_EQ(obstacles->size(), 0u);
}

TEST(Segment, MaxObstacleHeightAndDepthWindow)
{
  const auto cam = camera_pose();
  const auto img = render(cam, true, 0.45);
  DepthPoints dp;
  ASSERT_TRUE(project_depth(img.data(), true, 480, 640, 640 * 2, k640(), 8, dp));
  ChainParams p = nav();
  p.max_obstacle_height = 0.3;
  Cloud::Ptr cloud = make_cloud(dp, p), ground, obstacles;
  segment(cloud, cam, p, ground, obstacles);
  for (const auto & q : to_base(*obstacles, cam)) EXPECT_LE(q.z, 0.3 + 1e-4);
  p.max_depth = 1.0;    // the box at 1.5 m is out of the depth window
  cloud = make_cloud(dp, p);
  for (const auto & q : *cloud) EXPECT_LE(q.z, 1.0f + 0.05f);   // voxel centroids
  segment(cloud, cam, p, ground, obstacles);
  EXPECT_EQ(obstacles->size(), 0u);
}

TEST(Segment, RestIsEverythingElseWithReason)
{
  const auto cam = camera_pose();
  const auto img = render(cam, true, 0.45);   // box 0.45 m high
  DepthPoints dp;
  ASSERT_TRUE(project_depth(img.data(), true, 480, 640, 640 * 2, k640(), 8, dp));
  ChainParams p = nav();
  p.max_obstacle_height = 0.3;                // the box top (0.3..0.45) -> rest, reason ABOVE
  Cloud::Ptr cloud = make_cloud(dp, p), ground, obstacles;
  RestCloud::Ptr rest;
  segment(cloud, cam, p, ground, obstacles, &rest);
  ASSERT_TRUE(rest);
  // the outputs stay exactly the same with or without the rest
  Cloud::Ptr g2, o2;
  segment(cloud, cam, p, g2, o2);
  EXPECT_EQ(g2->size(), ground->size());
  EXPECT_EQ(o2->size(), obstacles->size());
  int above = 0, low = 0;
  for (const auto & q : rest->points) {
    const Eigen::Vector3d b = cam * Eigen::Vector3d(q.x, q.y, q.z);
    if (q.intensity == REST_ABOVE) {
      ++above;
      EXPECT_GT(b.z(), 0.3 - 1e-4);
    } else if (q.intensity == REST_LOW) {
      ++low;
      EXPECT_LT(b.z(), 0.15 + 1e-4);
    } else if (q.intensity == REST_SMALL) {
      EXPECT_GE(b.z(), 0.15 - 1e-4);
    }
  }
  EXPECT_GT(above, 0);
  // the low part of the box front (0..0.15 m, vertical = not ground) is rest LOW
  EXPECT_GT(low, 0);
}

TEST(Segment, EmptyInput)
{
  Cloud::Ptr cloud(new Cloud), ground, obstacles;
  segment(cloud, camera_pose(), nav(), ground, obstacles);
  EXPECT_TRUE(ground->empty());
  EXPECT_TRUE(obstacles->empty());
  DepthPoints dp;
  EXPECT_TRUE(make_cloud(dp, nav())->empty());
}

// --- live changes (live_config.cpp) ---------------------------------------------------------------------------

namespace
{
ProfileSettings nav_settings()
{
  ProfileSettings s;   // config/d435_ros1.yaml "nav"
  s.decimation = 8;
  s.p = nav();
  return s;
}

ProfileSettings wide_settings()
{
  ProfileSettings s = nav_settings();
  s.p.max_depth = 4.0;
  s.p.max_obstacle_height = 2.0;
  return s;
}

bool has_code(const std::vector<Issue> & v, const std::string & code)
{
  for (const auto & i : v) {
    if (i.code == code) return true;
  }
  return false;
}

// the value refused by the table
std::string refused(const std::string & key, double value, bool safety)
{
  std::vector<Issue> issues;
  EXPECT_NE(find_param(key), nullptr) << key;
  return check_value(*find_param(key), value, safety, issues) ? "" : issues[0].code;
}

// one value through everything a set of the safety profile goes through; "" = accepted
std::string safe_change(const std::string & key, double value)
{
  const ProfileSettings s = with_values(nav_settings(), {{key, value}});
  std::vector<Issue> v;
  if (!check_value(*find_param(key), value, true, v)) return v[0].code;
  v = check_relations(s, true, 480, 640);
  if (v.empty()) v = check_reference(run_reference(s));
  return v.empty() ? "" : v[0].code;
}
}  // namespace

TEST(LiveConfig, TableCoversEveryParameterAndRoundTrips)
{
  EXPECT_EQ(param_table().size(), 22u);
  ProfileSettings s = nav_settings();
  const ParamValues v = to_values(s);
  EXPECT_EQ(v.size(), param_table().size());
  EXPECT_EQ(v.at("decimation"), 8.0);
  EXPECT_EQ(v.at("flat_obstacles_detected"), 1.0);
  EXPECT_EQ(v.at("min_obstacle_height"), -1.0);    // NaN in ChainParams
  const ProfileSettings t = with_values(ProfileSettings(), v);
  EXPECT_EQ(t.decimation, 8);
  EXPECT_DOUBLE_EQ(t.p.noise_filter_radius, 0.12);
  EXPECT_TRUE(std::isnan(t.p.min_obstacle_height));
  EXPECT_DOUBLE_EQ(with_values(s, {{"min_obstacle_height", 0.2}}).p.min_obstacle_height, 0.2);
}

TEST(LiveConfig, DefaultsPassEverything)
{
  EXPECT_TRUE(check_relations(nav_settings(), true, 480, 640).empty());
  EXPECT_TRUE(check_relations(wide_settings(), false, 480, 640).empty());
  const ReferenceResult r = run_reference(nav_settings());
  EXPECT_GE(r.box_points, 10u) << "the reference box must be seen with margin";
  EXPECT_GE(r.ground_points, 50u);
  EXPECT_EQ(r.phantom_points, 0u);
  EXPECT_TRUE(check_reference(r).empty());
  // every default inside the limits of its profile
  for (const auto & d : param_table()) {
    if (std::string(d.key) == "normal_threads") continue;
    std::vector<Issue> issues;
    EXPECT_TRUE(check_value(d, d.get(nav_settings()), true, issues)) << d.key << " " << (issues.empty() ? "" : issues[0].code);
    EXPECT_TRUE(check_value(d, d.get(wide_settings()), false, issues)) << d.key;
  }
}

TEST(LiveConfig, ValueChecks)
{
  EXPECT_EQ(refused("max_depth", 1.0, true), "below_min");     // nav: must see far enough to stop
  EXPECT_EQ(refused("max_depth", 1.0, false), "");             // mapping may
  EXPECT_EQ(refused("max_depth", 0.0, true), "below_min");
  EXPECT_EQ(refused("max_depth", 0.0, false), "below_min");
  EXPECT_EQ(refused("max_depth", 99.0, false), "above_max");
  EXPECT_EQ(refused("max_depth", std::nan(""), false), "not_number");
  EXPECT_EQ(refused("max_obstacle_height", 0.0, true), "off_refused");
  EXPECT_EQ(refused("voxel_size", 0.0, true), "off_refused");
  EXPECT_EQ(refused("max_ground_angle_deg", 85.0, true), "above_max");   // walls would become ground
  EXPECT_EQ(refused("normal_k", 5.5, false), "not_integer");
  EXPECT_EQ(refused("normal_k", 2.0, false), "below_min");
  EXPECT_EQ(refused("decimation", 3.0, false), "not_choice");  // does not divide 640: nothing would be published
  EXPECT_EQ(refused("normals_segmentation", 2.0, false), "not_bool");
  EXPECT_EQ(refused("min_obstacle_height", -1.0, true), "");   // off = the default
  EXPECT_EQ(refused("noise_filter_radius", 0.0, true), "");
}

TEST(LiveConfig, Relations)
{
  auto wide = [](const ParamValues & v) { return check_relations(with_values(wide_settings(), v), false, 480, 640); };
  EXPECT_TRUE(has_code(wide({{"min_depth", 1.5}, {"max_depth", 1.0}}), "order"));
  EXPECT_TRUE(has_code(wide({{"range_min", 1.5}, {"range_max", 1.0}}), "order"));
  EXPECT_TRUE(has_code(wide({{"min_ground_height", 0.2}, {"max_ground_height", 0.1}}), "order"));
  EXPECT_TRUE(has_code(wide({{"max_ground_height", 0.5}, {"max_obstacle_height", 0.4}}), "order"));
  EXPECT_TRUE(has_code(wide({{"min_obstacle_height", 0.9}, {"max_obstacle_height", 0.5}}), "order"));
  EXPECT_TRUE(has_code(wide({{"min_depth", 1.9}, {"range_max", 1.5}}), "window_short"));   // no overlap of the two cuts
  EXPECT_TRUE(wide({{"max_depth", 6.0}, {"range_max", 0.0}}).empty());                    // both off is fine there
  // the decimation against the real image
  ProfileSettings s = wide_settings();
  s.decimation = 16;
  EXPECT_TRUE(has_code(check_relations(s, false, 720, 1270), "no_divide"));
  EXPECT_TRUE(check_relations(s, false, 720, 1280).empty());
  EXPECT_TRUE(check_relations(s, false, 0, 0).empty());
}

// Every way found to blind or to stop the robot through one or two parameters of nav must be refused.
TEST(LiveConfig, NavCannotBeBlinded)
{
  EXPECT_EQ(safe_change("max_depth", 0.3), "below_min");
  EXPECT_EQ(safe_change("max_obstacle_height", 0.1), "below_min");
  EXPECT_EQ(safe_change("max_ground_height", 0.9), "above_max");
  EXPECT_EQ(safe_change("min_cluster_size", 500), "above_max");
  EXPECT_EQ(safe_change("cluster_radius", 0.04), "below_min");
  // a window of 0.1 m of height at least
  EXPECT_EQ(safe_change("min_obstacle_height", 0.2), "");
  EXPECT_EQ(safe_change("min_obstacle_height", 0.3), "blind");   // the obstacle floor above the 0.3 m box
  EXPECT_EQ(safe_change("max_ground_height", 0.3), "blind");
  EXPECT_EQ(safe_change("min_cluster_size", 25), "blind");
  ProfileSettings s = with_values(nav_settings(), {{"min_obstacle_height", 0.3}, {"max_obstacle_height", 0.35}});
  EXPECT_TRUE(has_code(check_relations(s, true, 480, 640), "obstacle_window"));
  // the distance overlap: both limits fine, the overlap too short
  s = with_values(nav_settings(), {{"min_depth", 0.6}, {"max_depth", 1.5}});
  EXPECT_TRUE(has_code(check_relations(s, true, 480, 640), "window_short"));
  // voxels bigger than the reach of the clusters / of the noise filter
  s = with_values(nav_settings(), {{"voxel_size", 0.15}, {"cell_size", 0.15}, {"cluster_radius", 0.1}});
  EXPECT_TRUE(has_code(check_relations(s, true, 480, 640), "cluster_small"));
  s = with_values(nav_settings(), {{"voxel_size", 0.1}});
  EXPECT_TRUE(has_code(check_relations(s, true, 480, 640), "noise_small"));
  // inside every limit, but the box is gone: only the reference scene knows
  s = with_values(nav_settings(), {{"noise_filter_radius", 0.08}, {"noise_filter_min_neighbors", 15}});
  ASSERT_TRUE(check_relations(s, true, 480, 640).empty());
  EXPECT_TRUE(has_code(check_reference(run_reference(s)), "blind"));
  EXPECT_EQ(safe_change("decimation", 16), "");
}

TEST(LiveConfig, ReferenceSeesPhantomObstacles)
{
  // the whole ground taken for an obstacle: no normals, a ground band below the real ground
  ProfileSettings s = wide_settings();
  s.p.normals_segmentation = false;
  s.p.max_ground_height = -0.2;
  s.p.min_ground_height = -1.0;
  const ReferenceResult r = run_reference(s);
  EXPECT_GT(r.phantom_points, 10u);
  EXPECT_TRUE(has_code(check_reference(r), "phantom"));
}

// The settings of a frame are all the core knows: a change between frames gives what a fresh start with it gives.
TEST(LiveConfig, ChangeBetweenFramesEqualsFreshStart)
{
  const auto cam = camera_pose();
  const auto img = render(cam, true, 0.45);
  DepthPoints dp;
  ASSERT_TRUE(project_depth(img.data(), true, 480, 640, 640 * 2, k640(), 8, dp));
  ProfileSettings s = nav_settings();
  Cloud::Ptr g1, o1, g2, o2;
  segment(make_cloud(dp, s.p), cam, s.p, g1, o1);
  const ProfileSettings changed = with_values(s, {{"max_obstacle_height", 0.35}, {"max_ground_height", 0.1}});
  segment(make_cloud(dp, changed.p), cam, changed.p, g1, o1);
  ChainParams fresh = nav();
  fresh.max_obstacle_height = 0.35;
  fresh.max_ground_height = 0.1;
  segment(make_cloud(dp, fresh), cam, fresh, g2, o2);
  ASSERT_EQ(o1->size(), o2->size());
  ASSERT_EQ(g1->size(), g2->size());
  ASSERT_GT(o1->size(), 3u);
  for (std::size_t i = 0; i < o1->size(); ++i) EXPECT_EQ((*o1)[i].z, (*o2)[i].z);
}
