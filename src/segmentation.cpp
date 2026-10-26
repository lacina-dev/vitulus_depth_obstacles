// See segmentation.hpp. Line references: rtabmap 0.23.13-jazzy corelib/src.
#include "vitulus_depth_obstacles/segmentation.hpp"

#include <cmath>
#include <limits>

#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace vitulus_depth_obstacles
{

namespace
{
using IndicesPtr = pcl::IndicesPtr;

IndicesPtr make_indices() { return IndicesPtr(new Indices); }

IndicesPtr concat(const IndicesPtr & a, const IndicesPtr & b)
{
  IndicesPtr out(new Indices(*a));
  out->insert(out->end(), b->begin(), b->end());
  return out;
}

// util3d::passThrough -> pcl::PassThrough (limits inclusive)
IndicesPtr pass_through_z(const Cloud::Ptr & cloud, const IndicesPtr & indices, float min, float max)
{
  IndicesPtr out = make_indices();
  pcl::PassThrough<pcl::PointXYZ> f;
  f.setNegative(false);
  f.setFilterFieldName("z");
  f.setFilterLimits(min, max);
  f.setInputCloud(cloud);
  f.setIndices(indices);
  f.filter(*out);
  return out;
}

// util3d::extractIndices(cloud, indices, negative=true): everything NOT in indices
IndicesPtr complement(const Cloud::Ptr & cloud, const IndicesPtr & indices)
{
  IndicesPtr out = make_indices();
  pcl::ExtractIndices<pcl::PointXYZ> e;
  e.setInputCloud(cloud);
  e.setIndices(indices);
  e.setNegative(true);
  e.filter(*out);
  return out;
}

// util3d::voxelizeImpl; our clouds are a few metres wide, so the int32 overflow split never triggers (asserted).
Cloud::Ptr voxelize(const Cloud::Ptr & cloud, const IndicesPtr & indices, float leaf)
{
  Cloud::Ptr out(new Cloud);
  if ((cloud->is_dense && !cloud->empty()) || (!cloud->is_dense && !indices->empty())) {
    pcl::VoxelGrid<pcl::PointXYZ> f;
    f.setLeafSize(leaf, leaf, leaf);
    f.setInputCloud(cloud);
    if (!indices->empty()) {
      f.setIndices(indices);
    }
    f.filter(*out);
  }
  return out;
}

// util3d::radiusFilteringImpl: keep points with more than min_neighbors points (itself included) within radius
IndicesPtr radius_filter(const Cloud::Ptr & cloud, const IndicesPtr & indices, float radius, int min_neighbors)
{
  pcl::search::KdTree<pcl::PointXYZ> tree(false);
  const bool all = indices->empty();
  const std::size_t n = all ? cloud->size() : indices->size();
  IndicesPtr out(new Indices(n));
  std::size_t oi = 0;
  if (all) {
    tree.setInputCloud(cloud);
  } else {
    tree.setInputCloud(cloud, indices);
  }
  Indices k_idx;
  std::vector<float> k_dist;
  for (std::size_t i = 0; i < n; ++i) {
    const int id = all ? static_cast<int>(i) : indices->at(i);
    const int k = tree.radiusSearch(cloud->at(id), radius, k_idx, k_dist, min_neighbors + 1);
    if (k > min_neighbors) {
      (*out)[oi++] = id;
    }
  }
  out->resize(oi);
  return out;
}

// util3d::extractClustersImpl (sorted KdTree, EuclideanClusterExtraction; clusters come largest first)
std::vector<IndicesPtr> extract_clusters(const Cloud::Ptr & cloud, const IndicesPtr & indices, float tolerance,
                                         int min_size, int * biggest)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>(true));
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(tolerance);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(std::numeric_limits<int>::max());
  ec.setInputCloud(cloud);
  if (!indices->empty()) {
    ec.setIndices(indices);
    tree->setInputCloud(cloud, indices);
  } else {
    tree->setInputCloud(cloud);
  }
  ec.setSearchMethod(tree);
  std::vector<pcl::PointIndices> clusters;
  ec.extract(clusters);
  std::vector<IndicesPtr> out(clusters.size());
  int max_index = -1;
  std::size_t max_size = 0;
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    out[i] = IndicesPtr(new Indices(clusters[i].indices));
    if (max_size < clusters[i].indices.size()) {
      max_size = clusters[i].indices.size();
      max_index = static_cast<int>(i);
    }
  }
  if (biggest) {
    *biggest = max_index;
  }
  return out;
}

// util3d::normalFilteringImpl: normals (K neighbours, oriented to the viewpoint), keep angle to +z < angle_max
IndicesPtr normal_filter(const Cloud::Ptr & cloud, const IndicesPtr & indices, float angle_max, int k,
                         const Eigen::Vector4f & viewpoint, float ground_normals_up, int threads)
{
  IndicesPtr out = make_indices();
  if (cloud->empty()) {
    return out;
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>(false));
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(threads);
  ne.setInputCloud(cloud);
  if (!indices->empty()) {
    ne.setIndices(indices);
    tree->setInputCloud(cloud, indices);
  } else {
    tree->setInputCloud(cloud);
  }
  ne.setSearchMethod(tree);
  ne.setKSearch(k);
  if (viewpoint[0] != 0 || viewpoint[1] != 0 || viewpoint[2] != 0) {
    ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
  }
  pcl::PointCloud<pcl::Normal> normals;
  ne.compute(normals);
  const Eigen::Vector4f up(0, 0, 1, 0);
  out->resize(normals.size());
  std::size_t oi = 0;
  for (std::size_t i = 0; i < normals.size(); ++i) {
    const int id = indices->empty() ? static_cast<int>(i) : indices->at(i);
    Eigen::Vector4f v(normals[i].normal_x, normals[i].normal_y, normals[i].normal_z, 0.0f);
    if (ground_normals_up > 0.0f && v[2] < -ground_normals_up && cloud->at(id).z < viewpoint[2]) {
      v *= -1.0f;
    }
    if (pcl::getAngle3D(up, v) < angle_max) {
      (*out)[oi++] = id;
    }
  }
  out->resize(oi);
  return out;
}

// util3d::segmentObstaclesFromGround (util3d_mapping.hpp), obstacle floor optionally raised (min_obstacle_height)
void segment_from_ground(const Cloud::Ptr & cloud, const IndicesPtr & indices, IndicesPtr & ground,
                         IndicesPtr & obstacles, const ChainParams & p, const Eigen::Vector4f & viewpoint)
{
  ground = make_indices();
  obstacles = make_indices();
  if (cloud->empty()) {
    return;
  }
  const float max_ground_height = static_cast<float>(p.max_ground_height);
  const float angle = static_cast<float>(p.max_ground_angle_deg * M_PI / 180.0f);
  IndicesPtr flat = normal_filter(cloud, indices, angle, p.normal_k, viewpoint,
                                  static_cast<float>(p.ground_normals_up), p.normal_threads);

  if (!flat->empty() && (p.flat_obstacles_detected || max_ground_height != 0.0f || p.min_cluster_size > 1)) {
    int biggest = -1;
    std::vector<IndicesPtr> surfaces = extract_clusters(cloud, flat, static_cast<float>(p.cluster_radius),
                                                        p.min_cluster_size, &biggest);
    if (!surfaces.empty()) {
      Eigen::Vector4f bmin, bmax;
      if (max_ground_height != 0.0f) {
        std::size_t points = 0;
        biggest = -1;
        for (std::size_t i = 0; i < surfaces.size(); ++i) {
          Eigen::Vector4f mn, mx;
          pcl::getMinMax3D(*cloud, *surfaces[i], mn, mx);
          if (mn[2] < max_ground_height && surfaces[i]->size() > points) {
            points = surfaces[i]->size();
            biggest = static_cast<int>(i);
            bmin = mn;
            bmax = mx;
          }
        }
      } else {
        pcl::getMinMax3D(*cloud, *surfaces[biggest], bmin, bmax);
      }
      if (biggest >= 0) {
        ground = surfaces[biggest];
      }
      if (!ground->empty() && (max_ground_height == 0.0f || bmin[2] < max_ground_height)) {
        for (std::size_t i = 0; i < surfaces.size(); ++i) {
          if (static_cast<int>(i) == biggest) {
            continue;
          }
          Eigen::Vector4f c(0, 0, 0, 1);
          pcl::compute3DCentroid(*cloud, *surfaces[i], c);
          if ((max_ground_height != 0.0f && c[2] <= max_ground_height) || c[2] <= bmax[2] ||
              (max_ground_height == 0.0f && !p.flat_obstacles_detected)) {
            ground = concat(ground, surfaces[i]);
          }
          // else: flat obstacle; it stays in the "other stuff" below like in RTAB-Map
        }
      } else {
        ground = make_indices();  // reject ground
      }
    }
  } else {
    ground = flat;
  }

  if (ground->size() != cloud->size()) {
    IndicesPtr not_obstacles = ground;
    if (!indices->empty()) {
      not_obstacles = concat(complement(cloud, indices), ground);
    }
    IndicesPtr other = complement(cloud, not_obstacles);
    float floor = max_ground_height;
    const bool has_floor = !std::isnan(p.min_obstacle_height);
    if (has_floor) {
      floor = std::max(floor, static_cast<float>(p.min_obstacle_height));
    }
    if (floor != 0.0f || has_floor) {
      other = pass_through_z(cloud, other, floor, std::numeric_limits<float>::max());
    }
    if (!other->empty()) {
      for (const auto & c : extract_clusters(cloud, other, static_cast<float>(p.cluster_radius),
                                             p.min_cluster_size, nullptr)) {
        obstacles->insert(obstacles->end(), c->begin(), c->end());
      }
    }
  }
}

}  // namespace

bool project_depth(const void * data, bool is_mm, int rows, int cols, std::size_t step_bytes, const Intrinsics & k,
                   int decimation, DepthPoints & out)
{
  out.points.clear();
  out.depth.clear();
  if (decimation == 0) {
    decimation = 1;
  }
  decimation = std::abs(decimation);
  if (rows % decimation != 0 || cols % decimation != 0) {
    return false;
  }
  // util3d::cloudFromDepth: intrinsics scaled when the depth image is smaller than camera_info
  float fx = static_cast<float>(k.fx), fy = static_cast<float>(k.fy);
  float cx = static_cast<float>(k.cx), cy = static_cast<float>(k.cy);
  if (k.width > 0 && k.height > 0) {
    if (k.height % rows != 0 || k.width % cols != 0) {
      return false;
    }
    const float sx = 1.0f / static_cast<float>(k.width / cols);
    const float sy = 1.0f / static_cast<float>(k.height / rows);
    fx *= sx; fy *= sy; cx *= sx; cy *= sy;
  }
  // util3d::projectDepthTo3D fallback principal point
  if (!(cx > 0.0f)) cx = static_cast<float>(cols / 2) - 0.5f;
  if (!(cy > 0.0f)) cy = static_cast<float>(rows / 2) - 0.5f;
  const auto * base = static_cast<const std::uint8_t *>(data);
  out.points.reserve((rows / decimation) * (cols / decimation));
  out.depth.reserve(out.points.capacity());
  for (int h = 0; h < rows; h += decimation) {
    const std::uint8_t * row = base + static_cast<std::size_t>(h) * step_bytes;
    for (int w = 0; w < cols; w += decimation) {
      float d;
      if (is_mm) {
        const std::uint16_t v = reinterpret_cast<const std::uint16_t *>(row)[w];
        if (v == 0 || v == std::numeric_limits<std::uint16_t>::max()) continue;
        d = static_cast<float>(v) * 0.001f;
      } else {
        d = reinterpret_cast<const float *>(row)[w];
        if (!(d > 0.0f) || !std::isfinite(d)) continue;
      }
      pcl::PointXYZ pt;
      pt.x = (static_cast<float>(w) - cx) * d / fx;
      pt.y = (static_cast<float>(h) - cy) * d / fy;
      pt.z = d;
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
      out.points.push_back(pt);
      out.depth.push_back(d);
    }
  }
  return true;
}

Cloud::Ptr make_cloud(const DepthPoints & pts, const ChainParams & p)
{
  // depth window (cloudFromDepth: z >= min_depth && (max_depth <= 0 || z <= max_depth))
  Cloud::Ptr raw(new Cloud);
  raw->reserve(pts.points.size());
  const float min_d = static_cast<float>(p.min_depth), max_d = static_cast<float>(p.max_depth);
  for (std::size_t i = 0; i < pts.points.size(); ++i) {
    const float z = pts.depth[i];
    if (z >= min_d && (max_d <= 0.0f || z <= max_d)) {
      raw->push_back(pts.points[i]);
    }
  }
  raw->is_dense = true;
  Cloud::Ptr cloud = raw;
  if (!raw->empty() && p.voxel_size > 0.0) {
    cloud = voxelize(raw, make_indices(), static_cast<float>(p.voxel_size));
    cloud->is_dense = true;
  }
  if (!cloud->empty() && p.noise_filter_radius > 0.0 && p.noise_filter_min_neighbors > 0) {
    IndicesPtr keep = radius_filter(cloud, make_indices(), static_cast<float>(p.noise_filter_radius),
                                    p.noise_filter_min_neighbors);
    Cloud::Ptr tmp(new Cloud);
    pcl::copyPointCloud(*cloud, *keep, *tmp);
    cloud = tmp;
  }
  return cloud;
}

void segment(const Cloud::Ptr & cloud_cam, const Eigen::Isometry3d & camera_to_base, const ChainParams & p,
             Cloud::Ptr & ground_cam, Cloud::Ptr & obstacles_cam, RestCloud::Ptr * rest_cam)
{
  ground_cam.reset(new Cloud);
  obstacles_cam.reset(new Cloud);
  if (rest_cam) {
    rest_cam->reset(new RestCloud);
  }
  // obstacles_detection rangeFiltering (sensor frame)
  Cloud::Ptr input(new Cloud);
  if (p.range_min > 0.0 || p.range_max > 0.0) {
    const float rmin2 = static_cast<float>(p.range_min * p.range_min);
    const float rmax2 = static_cast<float>(p.range_max * p.range_max);
    input->reserve(cloud_cam->size());
    for (const auto & pt : *cloud_cam) {
      const float r = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (p.range_min > 0.0 && r < rmin2) continue;
      if (p.range_max > 0.0 && r > rmax2) continue;
      input->push_back(pt);
    }
  } else {
    *input = *cloud_cam;
  }
  input->is_dense = true;
  if (input->empty()) {
    return;
  }
  // rtabmap::Transform keeps a float 3x4 matrix built from the double TF
  const Eigen::Matrix4f local = camera_to_base.matrix().cast<float>();
  Cloud::Ptr base(new Cloud);
  pcl::transformPointCloud(*input, *base, local);

  // LocalGridMaker::segmentCloud (pose = identity, no map-frame projection, no footprint)
  Cloud::Ptr cloud = base;
  IndicesPtr indices = make_indices();
  if (p.pre_voxel_filtering) {
    cloud = voxelize(base, make_indices(), static_cast<float>(p.cell_size));
  }
  if (cloud->empty()) {
    return;
  }
  indices->resize(cloud->size());
  for (std::size_t i = 0; i < indices->size(); ++i) {
    (*indices)[i] = static_cast<int>(i);
  }
  if (p.min_ground_height != 0.0 || p.max_obstacle_height != 0.0) {
    indices = pass_through_z(
      cloud, indices,
      p.min_ground_height != 0.0 ? static_cast<float>(p.min_ground_height)
                                 : static_cast<float>(std::numeric_limits<int>::min()),
      p.max_obstacle_height > 0.0 ? static_cast<float>(p.max_obstacle_height)
                                  : static_cast<float>(std::numeric_limits<int>::max()));
  }
  IndicesPtr ground = make_indices(), obstacles = make_indices();
  if (!indices->empty()) {
    if (p.normals_segmentation) {
      const Eigen::Vector4f vp(local(0, 3), local(1, 3), local(2, 3), 1.0f);
      segment_from_ground(cloud, indices, ground, obstacles, p, vp);
    } else {
      ground = pass_through_z(
        cloud, indices,
        p.min_ground_height != 0.0 ? static_cast<float>(p.min_ground_height)
                                   : static_cast<float>(std::numeric_limits<int>::min()),
        p.max_ground_height != 0.0 ? static_cast<float>(p.max_ground_height)
                                   : static_cast<float>(std::numeric_limits<int>::max()));
      IndicesPtr not_obstacles = concat(complement(cloud, indices), ground);
      obstacles = complement(cloud, not_obstacles);
      if (!std::isnan(p.min_obstacle_height)) {
        obstacles = pass_through_z(cloud, obstacles, static_cast<float>(p.min_obstacle_height),
                                   std::numeric_limits<float>::max());
      }
    }
  }
  // back to the camera frame: t = localTransform.inverse()
  const Eigen::Matrix4f inv = Eigen::Affine3f(local).inverse().matrix();
  if (rest_cam) {
    // every pre-voxelled point that ended in neither output, with the reason
    std::vector<char> used(cloud->size(), 0), in_window(cloud->size(), 0);
    for (int i : *ground) used[i] = 1;
    for (int i : *obstacles) used[i] = 1;
    for (int i : *indices) in_window[i] = 1;
    float floor = static_cast<float>(p.max_ground_height);
    if (!std::isnan(p.min_obstacle_height)) {
      floor = std::max(floor, static_cast<float>(p.min_obstacle_height));
    }
    if (!p.normals_segmentation && std::isnan(p.min_obstacle_height)) {
      floor = -std::numeric_limits<float>::max();
    }
    RestCloud rest;
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      if (used[i]) continue;
      const auto & q = (*cloud)[i];
      pcl::PointXYZI r;
      r.x = q.x;
      r.y = q.y;
      r.z = q.z;
      if (!in_window[i]) {
        r.intensity = (p.max_obstacle_height > 0.0 && q.z > p.max_obstacle_height) ? REST_ABOVE : REST_BELOW;
      } else {
        r.intensity = q.z < floor ? REST_LOW : REST_SMALL;
      }
      rest.push_back(r);
    }
    pcl::transformPointCloud(rest, **rest_cam, inv);
  }
  if (ground->empty() && obstacles->empty()) {
    return;
  }
  if (!ground->empty()) {
    Cloud tmp;
    pcl::copyPointCloud(*cloud, *ground, tmp);
    pcl::transformPointCloud(tmp, *ground_cam, inv);
  }
  if (!obstacles->empty()) {
    Cloud tmp;
    pcl::copyPointCloud(*cloud, *obstacles, tmp);
    pcl::transformPointCloud(tmp, *obstacles_cam, inv);
  }
}

}  // namespace vitulus_depth_obstacles
