// Algorithm tests on synthetic scenes (no ROS). The RTAB-Map equality itself is measured by tools/ab_rtabmap.py.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

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
