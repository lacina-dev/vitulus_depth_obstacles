// Depth image -> ground / obstacle point clouds, without ROS.
//
// Replaces the RTAB-Map chain rtabmap_util/point_cloud_xyz + rtabmap_util/obstacles_detection (0.23.13) with the
// same algorithm on the same PCL classes, so the default output matches it point for point. Every step names the
// RTAB-Map function it reproduces (rtabmap corelib util3d*.cpp, LocalGridMaker.hpp, util3d_mapping.hpp).
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vitulus_depth_obstacles
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Indices = std::vector<int>;

struct Intrinsics
{
  double fx{0}, fy{0}, cx{0}, cy{0};
  int width{0}, height{0};  // camera_info size (0 = unknown)
};

// One raw depth sample per decimated pixel: camera-frame point, only finite and > 0 depth.
struct DepthPoints
{
  std::vector<pcl::PointXYZ> points;
  std::vector<float> depth;  // = z, kept separately for the per-profile depth window
};

// util3d::cloudFromDepth (no confidence, no ROI): pixel (w, h) every `decimation` pixels, depth in metres
// (16UC1 = mm, 0 and 65535 invalid; 32FC1 as is). Returns false when the decimation does not divide the image
// (RTAB-Map then publishes nothing useful either; the caller logs it).
bool project_depth(const void * data, bool is_mm, int rows, int cols, std::size_t step_bytes,
                   const Intrinsics & k, int decimation, DepthPoints & out);

struct ChainParams
{
  // point_cloud_xyz
  double min_depth{0.0};
  double max_depth{0.0};           // 0 = no limit
  double voxel_size{0.05};         // 0 = off
  double noise_filter_radius{0.0}; // 0 = off
  int noise_filter_min_neighbors{5};
  // obstacles_detection (rtabmap Grid/*)
  double range_min{0.0};           // Grid/RangeMin, from the sensor, before segmentation
  double range_max{5.0};           // Grid/RangeMax
  double cell_size{0.05};          // Grid/CellSize (pre-voxel filter in the base frame)
  bool pre_voxel_filtering{true};  // Grid/PreVoxelFiltering
  double max_obstacle_height{0.0}; // Grid/MaxObstacleHeight (0 = off)
  double min_ground_height{0.0};   // Grid/MinGroundHeight (0 = off)
  double max_ground_height{0.0};   // Grid/MaxGroundHeight (0 = off)
  double max_ground_angle_deg{45.0};
  int normal_k{20};
  double cluster_radius{0.1};
  int min_cluster_size{10};
  bool flat_obstacles_detected{true};
  bool normals_segmentation{true};
  double ground_normals_up{0.0};   // Icp/PointToPlaneGroundNormalsUp
  // Lowest obstacle point. RTAB-Map has no such parameter (Grid/MinObstacleHeight does not exist and was
  // silently ignored in the old launch files): with max_ground_height set, obstacles start at max_ground_height.
  // NaN = that behaviour; a number = explicit floor (>= max_ground_height has no extra effect).
  double min_obstacle_height{std::numeric_limits<double>::quiet_NaN()};
  int normal_threads{1};           // NormalEstimationOMP threads (0 = all cores, RTAB-Map default); same result
};

struct ChainResult
{
  Cloud::Ptr cloud;      // the /cloudXYZ equivalent (camera frame, after voxel + noise filter)
  Cloud::Ptr ground;     // camera frame
  Cloud::Ptr obstacles;  // camera frame
  bool input_empty{false};
  // timings [ms]
  double t_cloud{0}, t_segment{0};
};

// point_cloud_xyz part: depth window, voxel grid, radius noise filter.
Cloud::Ptr make_cloud(const DepthPoints & pts, const ChainParams & p);

// The rest: segmented points that are neither ground nor obstacle (not an RTAB-Map output).
// intensity = reason:
enum RestReason : int
{
  REST_ABOVE = 1,   // above max_obstacle_height (branches, things over the robot)
  REST_BELOW = 2,   // below min_ground_height
  REST_LOW = 3,     // not the ground surface, but below the obstacle floor (tall grass, slopes, small steps)
  REST_SMALL = 4,   // above the floor, but in a cluster smaller than min_cluster_size (isolated points)
};
using RestCloud = pcl::PointCloud<pcl::PointXYZI>;

// obstacles_detection part. camera_to_base = pose of the camera frame in the base frame (TF base <- camera).
// rest_cam (optional): the rest, camera frame, after the same pre-voxel filter as ground / obstacles.
void segment(const Cloud::Ptr & cloud_cam, const Eigen::Isometry3d & camera_to_base, const ChainParams & p,
             Cloud::Ptr & ground_cam, Cloud::Ptr & obstacles_cam, RestCloud::Ptr * rest_cam = nullptr);

}  // namespace vitulus_depth_obstacles
