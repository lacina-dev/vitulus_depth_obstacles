// depth_obstacles (ROS1 Noetic): the D435 ground / obstacle clouds in one node instead of the RTAB-Map container
// RTABMAP_OBSTACLES (2x rtabmap_ros point_cloud_xyz + 2x obstacles_detection). The algorithm is the shared core
// (segmentation.cpp, the same file as the ROS2 package vitulus_depth_obstacles); the depth image is projected
// once and shared by the profiles (nav, wide, ...). Parameters are re-read on ~reload (std_srvs/Empty).
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vitulus_depth_obstacles/segmentation.hpp"

using namespace vitulus_depth_obstacles;
using Clock = std::chrono::steady_clock;

namespace
{
double ms_since(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
}  // namespace

class DepthObstacles
{
public:
  DepthObstacles() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("frame_id", frame_id_, "base_link");
    pnh_.param("wait_for_transform", wait_for_transform_, 0.1);
    bool approx = true;
    int sync_queue = 10, topic_queue = 1;
    pnh_.param("approx_sync", approx, true);
    pnh_.param("queue_size", sync_queue, 10);
    pnh_.param("topic_queue_size", topic_queue, 1);
    double stats_period = 5.0;
    pnh_.param("stats_period", stats_period, 5.0);
    std::vector<std::string> names;
    if (!pnh_.getParam("profiles", names) || names.empty()) {
      names = {"nav"};
    }
    for (const auto & n : names) {
      Profile pr;
      pr.name = n;
      read_profile(pr);
      const std::string cloud_t = pnh_.param<std::string>(n + "/cloud_topic", "");
      const std::string obst_t = pnh_.param<std::string>(n + "/obstacles_topic", n + "/obstacles");
      const std::string ground_t = pnh_.param<std::string>(n + "/ground_topic", n + "/ground");
      const std::string rest_t = pnh_.param<std::string>(n + "/rest_topic", "");
      if (!cloud_t.empty()) pr.cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(cloud_t, 1);
      if (!obst_t.empty()) pr.obstacles_pub = nh_.advertise<sensor_msgs::PointCloud2>(obst_t, 1);
      if (!ground_t.empty()) pr.ground_pub = nh_.advertise<sensor_msgs::PointCloud2>(ground_t, 1);
      if (!rest_t.empty()) pr.rest_pub = nh_.advertise<sensor_msgs::PointCloud2>(rest_t, 1);
      profiles_.push_back(pr);
    }
    depth_sub_.subscribe(nh_, "depth/image", topic_queue);
    info_sub_.subscribe(nh_, "depth/camera_info", topic_queue);
    if (approx) {
      approx_sync_.reset(new ApproxSync(ApproxPolicy(sync_queue), depth_sub_, info_sub_));
      approx_sync_->registerCallback(boost::bind(&DepthObstacles::callback, this, _1, _2));
    } else {
      exact_sync_.reset(new ExactSync(ExactPolicy(sync_queue), depth_sub_, info_sub_));
      exact_sync_->registerCallback(boost::bind(&DepthObstacles::callback, this, _1, _2));
    }
    reload_srv_ = pnh_.advertiseService("reload", &DepthObstacles::reload, this);
    stats_pub_ = pnh_.advertise<diagnostic_msgs::DiagnosticStatus>("stats", 1);
    if (stats_period > 0.0) {
      stats_timer_ = nh_.createWallTimer(ros::WallDuration(stats_period), &DepthObstacles::publish_stats, this);
    }
    std::string list;
    for (const auto & p : profiles_) list += " " + p.name;
    ROS_INFO("depth_obstacles: %s + %s -> profiles:%s (frame %s)", depth_sub_.getTopic().c_str(),
             info_sub_.getTopic().c_str(), list.c_str(), frame_id_.c_str());
  }

private:
  struct Profile
  {
    std::string name;
    int decimation{1};
    ChainParams p;
    ros::Publisher cloud_pub, obstacles_pub, ground_pub, rest_pub;
    std::size_t frames{0}, n_cloud{0}, n_ground{0}, n_obst{0}, n_rest{0};
    double t_cloud{0}, t_seg{0}, t_max{0};
  };

  void read_profile(Profile & pr)
  {
    const std::string n = pr.name + "/";
    ChainParams & p = pr.p;
    pnh_.param(n + "decimation", pr.decimation, 1);
    pnh_.param(n + "min_depth", p.min_depth, p.min_depth);
    pnh_.param(n + "max_depth", p.max_depth, p.max_depth);
    pnh_.param(n + "voxel_size", p.voxel_size, p.voxel_size);
    pnh_.param(n + "noise_filter_radius", p.noise_filter_radius, p.noise_filter_radius);
    pnh_.param(n + "noise_filter_min_neighbors", p.noise_filter_min_neighbors, p.noise_filter_min_neighbors);
    pnh_.param(n + "range_min", p.range_min, p.range_min);
    pnh_.param(n + "range_max", p.range_max, p.range_max);
    pnh_.param(n + "cell_size", p.cell_size, p.cell_size);
    pnh_.param(n + "pre_voxel_filtering", p.pre_voxel_filtering, p.pre_voxel_filtering);
    pnh_.param(n + "max_obstacle_height", p.max_obstacle_height, p.max_obstacle_height);
    pnh_.param(n + "min_ground_height", p.min_ground_height, p.min_ground_height);
    pnh_.param(n + "max_ground_height", p.max_ground_height, p.max_ground_height);
    pnh_.param(n + "max_ground_angle_deg", p.max_ground_angle_deg, p.max_ground_angle_deg);
    pnh_.param(n + "normal_k", p.normal_k, p.normal_k);
    pnh_.param(n + "cluster_radius", p.cluster_radius, p.cluster_radius);
    pnh_.param(n + "min_cluster_size", p.min_cluster_size, p.min_cluster_size);
    pnh_.param(n + "flat_obstacles_detected", p.flat_obstacles_detected, p.flat_obstacles_detected);
    pnh_.param(n + "normals_segmentation", p.normals_segmentation, p.normals_segmentation);
    pnh_.param(n + "ground_normals_up", p.ground_normals_up, p.ground_normals_up);
    pnh_.param(n + "normal_threads", p.normal_threads, p.normal_threads);
    double mo = -1.0;
    pnh_.param(n + "min_obstacle_height", mo, -1.0);
    p.min_obstacle_height = mo < 0.0 ? std::numeric_limits<double>::quiet_NaN() : mo;
    if (pr.decimation < 1) pr.decimation = 1;
    if (p.normal_k < 3) p.normal_k = 3;
    if (p.min_cluster_size < 1) p.min_cluster_size = 1;
  }

  bool reload(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    pnh_.param("wait_for_transform", wait_for_transform_, wait_for_transform_);
    for (auto & pr : profiles_) read_profile(pr);
    ROS_INFO("depth_obstacles: parameters reloaded");
    return true;
  }

  static bool wanted(const ros::Publisher & pub) { return pub && pub.getNumSubscribers() > 0; }

  template<typename CloudT>
  void publish(const ros::Publisher & pub, const CloudT & cloud, const std_msgs::Header & header)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header = header;
    pub.publish(msg);
  }

  void callback(const sensor_msgs::ImageConstPtr & depth, const sensor_msgs::CameraInfoConstPtr & info)
  {
    namespace enc = sensor_msgs::image_encodings;
    const Clock::time_point t0 = Clock::now();
    std::lock_guard<std::mutex> lk(mutex_);
    const bool is_mm = depth->encoding == enc::TYPE_16UC1 || depth->encoding == enc::MONO16;
    if (!is_mm && depth->encoding != enc::TYPE_32FC1) {
      ROS_ERROR_THROTTLE(5, "Input type depth=32FC1,16UC1,MONO16 (got %s)", depth->encoding.c_str());
      return;
    }
    bool any = false;
    for (const auto & pr : profiles_) {
      any = any || wanted(pr.cloud_pub) || wanted(pr.obstacles_pub) || wanted(pr.ground_pub) || wanted(pr.rest_pub);
    }
    if (!any) return;
    Intrinsics k;
    const bool p_set = info->P[0] != 0.0;
    k.fx = p_set ? info->P[0] : info->K[0];
    k.fy = p_set ? info->P[5] : info->K[4];
    k.cx = p_set ? info->P[2] : info->K[2];
    k.cy = p_set ? info->P[6] : info->K[5];
    k.width = static_cast<int>(info->width);
    k.height = static_cast<int>(info->height);
    if (!(k.fx > 0.0) || !(k.fy > 0.0)) {
      ROS_ERROR_THROTTLE(5, "camera_info has no focal length");
      return;
    }
    std::map<int, DepthPoints> projected;
    bool tf_done = false, tf_ok = false;
    Eigen::Isometry3d cam_to_base = Eigen::Isometry3d::Identity();
    for (auto & pr : profiles_) {
      const bool w_cloud = wanted(pr.cloud_pub), w_obst = wanted(pr.obstacles_pub), w_gnd = wanted(pr.ground_pub);
      const bool w_rest = wanted(pr.rest_pub);
      if (!w_cloud && !w_obst && !w_gnd && !w_rest) continue;
      const Clock::time_point tp = Clock::now();
      auto it = projected.find(pr.decimation);
      if (it == projected.end()) {
        DepthPoints dp;
        if (!project_depth(depth->data.data(), is_mm, static_cast<int>(depth->height),
                           static_cast<int>(depth->width), depth->step, k, pr.decimation, dp)) {
          ROS_ERROR_THROTTLE(5, "%s: decimation %d does not divide the depth image %ux%u", pr.name.c_str(),
                             pr.decimation, depth->width, depth->height);
          continue;
        }
        it = projected.emplace(pr.decimation, std::move(dp)).first;
      }
      Cloud::Ptr cloud = make_cloud(it->second, pr.p);
      if (w_cloud) publish(pr.cloud_pub, *cloud, depth->header);
      const double t_cloud = ms_since(tp);
      if (!w_obst && !w_gnd && !w_rest) {
        pr.t_cloud += t_cloud;
        continue;
      }
      if (!tf_done) {
        tf_done = true;
        try {
          const auto tf = tf_buffer_.lookupTransform(frame_id_, depth->header.frame_id, depth->header.stamp,
                                                     ros::Duration(wait_for_transform_));
          cam_to_base = tf2::transformToEigen(tf.transform);
          tf_ok = true;
        } catch (const tf2::TransformException & e) {
          ++tf_failures_;
          ROS_ERROR_THROTTLE(5, "Failed to get transform between %s and %s: %s", frame_id_.c_str(),
                             depth->header.frame_id.c_str(), e.what());
        }
      }
      if (!tf_ok) continue;
      const Clock::time_point ts = Clock::now();
      Cloud::Ptr ground, obstacles;
      RestCloud::Ptr rest;
      segment(cloud, cam_to_base, pr.p, ground, obstacles, w_rest ? &rest : nullptr);
      if (w_gnd) publish(pr.ground_pub, *ground, depth->header);
      if (w_obst) publish(pr.obstacles_pub, *obstacles, depth->header);
      if (w_rest) {
        publish(pr.rest_pub, *rest, depth->header);
        pr.n_rest += rest->size();
      }
      const double t_seg = ms_since(ts);
      ++pr.frames;
      pr.t_cloud += t_cloud;
      pr.t_seg += t_seg;
      pr.t_max = std::max(pr.t_max, t_cloud + t_seg);
      pr.n_cloud += cloud->size();
      pr.n_ground += ground->size();
      pr.n_obst += obstacles->size();
    }
    ++frames_;
    t_total_ += ms_since(t0);
  }

  void publish_stats(const ros::WallTimerEvent &)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    diagnostic_msgs::DiagnosticStatus st;
    st.name = ros::this_node::getName() + ": depth obstacles";
    st.hardware_id = "d435";
    auto kv = [&](const std::string & k, const std::string & v) {
      diagnostic_msgs::KeyValue x;
      x.key = k;
      x.value = v;
      st.values.push_back(x);
    };
    char buf[64];
    auto f = [&](double v) { std::snprintf(buf, sizeof(buf), "%.2f", v); return std::string(buf); };
    kv("frames", std::to_string(frames_));
    kv("frame_ms_avg", f(frames_ ? t_total_ / frames_ : 0.0));
    kv("tf_failures", std::to_string(tf_failures_));
    for (auto & pr : profiles_) {
      const double n = pr.frames ? static_cast<double>(pr.frames) : 1.0;
      kv(pr.name + ".frames", std::to_string(pr.frames));
      kv(pr.name + ".cloud_ms_avg", f(pr.t_cloud / n));
      kv(pr.name + ".segment_ms_avg", f(pr.t_seg / n));
      kv(pr.name + ".frame_ms_max", f(pr.t_max));
      kv(pr.name + ".cloud_pts_avg", f(pr.n_cloud / n));
      kv(pr.name + ".ground_pts_avg", f(pr.n_ground / n));
      kv(pr.name + ".obstacle_pts_avg", f(pr.n_obst / n));
      kv(pr.name + ".rest_pts_avg", f(pr.n_rest / n));
      pr.frames = pr.n_cloud = pr.n_ground = pr.n_obst = pr.n_rest = 0;
      pr.t_cloud = pr.t_seg = pr.t_max = 0;
    }
    st.level = (frames_ == 0 || tf_failures_ > 0) ? diagnostic_msgs::DiagnosticStatus::WARN
                                                   : diagnostic_msgs::DiagnosticStatus::OK;
    st.message = frames_ == 0 ? "no depth frames processed" : (tf_failures_ ? "TF failures" : "ok");
    frames_ = tf_failures_ = 0;
    t_total_ = 0;
    stats_pub_.publish(st);
  }

  using ApproxPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo>;
  using ExactPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo>;
  using ApproxSync = message_filters::Synchronizer<ApproxPolicy>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex mutex_;
  std::string frame_id_;
  double wait_for_transform_{0.1};
  std::vector<Profile> profiles_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  std::unique_ptr<ApproxSync> approx_sync_;
  std::unique_ptr<ExactSync> exact_sync_;
  ros::ServiceServer reload_srv_;
  ros::Publisher stats_pub_;
  ros::WallTimer stats_timer_;
  std::size_t frames_{0}, tf_failures_{0};
  double t_total_{0};
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "depth_obstacles");
  DepthObstacles node;
  ros::spin();
  return 0;
}
