// depth_obstacles: one node for the D435 ground / obstacle clouds (replaces 2x rtabmap point_cloud_xyz +
// 2x rtabmap obstacles_detection). The depth image is decoded and projected once; each "profile" (nav, wide, ...)
// gets its own depth window, segmentation parameters and output topics. All numeric parameters can be changed at
// runtime (ros2 param set); topics are fixed at start.
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vitulus_depth_obstacles/segmentation.hpp"

namespace vitulus_depth_obstacles
{

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Clock = std::chrono::steady_clock;

namespace
{
double ms_since(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}
}  // namespace

class DepthObstaclesNode : public rclcpp::Node
{
public:
  explicit DepthObstaclesNode(const rclcpp::NodeOptions & options) : Node("depth_obstacles", options)
  {
    frame_id_ = declare_parameter("frame_id", std::string("base_link"));
    wait_for_transform_ = declare_parameter("wait_for_transform", 0.2);
    max_rate_hz_ = declare_parameter("max_rate_hz", 0.0);
    const bool approx = declare_parameter("approx_sync", true);
    const double approx_max_interval = declare_parameter("approx_sync_max_interval", 0.0);
    const int sync_queue = declare_parameter("sync_queue_size", 10);
    const int topic_queue = declare_parameter("topic_queue_size", 1);
    const bool reliable = declare_parameter("qos_reliable", true);
    const std::string depth_topic = declare_parameter("depth_topic", std::string("depth/image"));
    const std::string info_topic = declare_parameter("camera_info_topic", std::string("depth/camera_info"));
    const double stats_period = declare_parameter("stats_period", 5.0);
    const auto names = declare_parameter("profiles", std::vector<std::string>{"nav"});

    rclcpp::QoS out_qos(1);
    if (!reliable) out_qos.best_effort();
    for (const auto & n : names) {
      declare_profile(n, out_qos);
    }
    if (profiles_.empty()) {
      throw std::runtime_error("depth_obstacles: parameter 'profiles' is empty");
    }
    param_cb_ = add_on_set_parameters_callback(
      std::bind(&DepthObstaclesNode::on_params, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS in_qos(topic_queue);
    if (!reliable) in_qos.best_effort();
    depth_sub_.subscribe(this, depth_topic, in_qos.get_rmw_qos_profile());
    info_sub_.subscribe(this, info_topic, in_qos.get_rmw_qos_profile());
    if (approx) {
      approx_sync_ = std::make_shared<ApproxSync>(ApproxPolicy(sync_queue), depth_sub_, info_sub_);
      if (approx_max_interval > 0.0) {
        approx_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approx_max_interval));
      }
      approx_sync_->registerCallback(
        std::bind(&DepthObstaclesNode::callback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      exact_sync_ = std::make_shared<ExactSync>(ExactPolicy(sync_queue), depth_sub_, info_sub_);
      exact_sync_->registerCallback(
        std::bind(&DepthObstaclesNode::callback, this, std::placeholders::_1, std::placeholders::_2));
    }
    stats_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/stats", rclcpp::QoS(1));
    if (stats_period > 0.0) {
      stats_timer_ = create_wall_timer(std::chrono::duration<double>(stats_period),
                                       std::bind(&DepthObstaclesNode::publish_stats, this));
    }
    std::string list;
    for (const auto & p : profiles_) list += " " + p.name;
    RCLCPP_INFO(get_logger(), "depth_obstacles: %s + %s -> profiles:%s (frame %s)",
                depth_sub_.getSubscriber()->get_topic_name(), info_sub_.getSubscriber()->get_topic_name(),
                list.c_str(), frame_id_.c_str());
  }

private:
  struct Profile
  {
    std::string name;
    int decimation{1};
    ChainParams p;
    rclcpp::Publisher<PointCloud2>::SharedPtr cloud_pub, obstacles_pub, ground_pub, rest_pub;
    // stats
    std::size_t frames{0};
    double t_cloud{0}, t_seg{0}, t_max{0};
    std::size_t n_cloud{0}, n_ground{0}, n_obst{0}, n_rest{0};
  };

  void declare_profile(const std::string & n, const rclcpp::QoS & qos)
  {
    Profile pr;
    pr.name = n;
    auto d = [&](const std::string & k, auto def) { return declare_parameter(n + "." + k, def); };
    pr.decimation = d("decimation", 1);
    ChainParams & p = pr.p;
    p.min_depth = d("min_depth", p.min_depth);
    p.max_depth = d("max_depth", p.max_depth);
    p.voxel_size = d("voxel_size", p.voxel_size);
    p.noise_filter_radius = d("noise_filter_radius", p.noise_filter_radius);
    p.noise_filter_min_neighbors = d("noise_filter_min_neighbors", p.noise_filter_min_neighbors);
    p.range_min = d("range_min", p.range_min);
    p.range_max = d("range_max", p.range_max);
    p.cell_size = d("cell_size", p.cell_size);
    p.pre_voxel_filtering = d("pre_voxel_filtering", p.pre_voxel_filtering);
    p.max_obstacle_height = d("max_obstacle_height", p.max_obstacle_height);
    p.min_ground_height = d("min_ground_height", p.min_ground_height);
    p.max_ground_height = d("max_ground_height", p.max_ground_height);
    p.max_ground_angle_deg = d("max_ground_angle_deg", p.max_ground_angle_deg);
    p.normal_k = d("normal_k", p.normal_k);
    p.cluster_radius = d("cluster_radius", p.cluster_radius);
    p.min_cluster_size = d("min_cluster_size", p.min_cluster_size);
    p.flat_obstacles_detected = d("flat_obstacles_detected", p.flat_obstacles_detected);
    p.normals_segmentation = d("normals_segmentation", p.normals_segmentation);
    p.ground_normals_up = d("ground_normals_up", p.ground_normals_up);
    // < 0 (default -1) = RTAB-Map behaviour: obstacles start at max_ground_height
    const double mo = d("min_obstacle_height", -1.0);
    p.min_obstacle_height = mo < 0.0 ? std::numeric_limits<double>::quiet_NaN() : mo;
    p.normal_threads = d("normal_threads", p.normal_threads);
    const std::string cloud_t = d("cloud_topic", std::string(""));
    const std::string obst_t = d("obstacles_topic", n + "/obstacles");
    const std::string ground_t = d("ground_topic", n + "/ground");
    if (!cloud_t.empty()) pr.cloud_pub = create_publisher<PointCloud2>(cloud_t, qos);
    if (!obst_t.empty()) pr.obstacles_pub = create_publisher<PointCloud2>(obst_t, qos);
    if (!ground_t.empty()) pr.ground_pub = create_publisher<PointCloud2>(ground_t, qos);
    // neither ground nor obstacle; intensity = reason (segmentation.hpp RestReason); '' = off
    const std::string rest_t = d("rest_topic", std::string(""));
    if (!rest_t.empty()) pr.rest_pub = create_publisher<PointCloud2>(rest_t, qos);
    profiles_.push_back(std::move(pr));
  }

  // runtime changes of "<profile>.<numeric>" and the node-wide numeric parameters
  rcl_interfaces::msg::SetParametersResult on_params(const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<Profile> next = profiles_;  // validate everything before applying
    double wait = wait_for_transform_, rate = max_rate_hz_;
    for (const auto & prm : params) {
      const std::string & full = prm.get_name();
      if (full == "wait_for_transform") { wait = prm.as_double(); continue; }
      if (full == "max_rate_hz") { rate = prm.as_double(); continue; }
      const auto dot = full.find('.');
      if (dot == std::string::npos) continue;  // start-only parameter (declared read-write by rclcpp)
      const std::string n = full.substr(0, dot), k = full.substr(dot + 1);
      Profile * pr = nullptr;
      for (auto & x : next) if (x.name == n) pr = &x;
      if (!pr) continue;
      ChainParams & p = pr->p;
      try {
        if (k == "decimation") pr->decimation = static_cast<int>(prm.as_int());
        else if (k == "min_depth") p.min_depth = prm.as_double();
        else if (k == "max_depth") p.max_depth = prm.as_double();
        else if (k == "voxel_size") p.voxel_size = prm.as_double();
        else if (k == "noise_filter_radius") p.noise_filter_radius = prm.as_double();
        else if (k == "noise_filter_min_neighbors") p.noise_filter_min_neighbors = static_cast<int>(prm.as_int());
        else if (k == "range_min") p.range_min = prm.as_double();
        else if (k == "range_max") p.range_max = prm.as_double();
        else if (k == "cell_size") p.cell_size = prm.as_double();
        else if (k == "pre_voxel_filtering") p.pre_voxel_filtering = prm.as_bool();
        else if (k == "max_obstacle_height") p.max_obstacle_height = prm.as_double();
        else if (k == "min_ground_height") p.min_ground_height = prm.as_double();
        else if (k == "max_ground_height") p.max_ground_height = prm.as_double();
        else if (k == "max_ground_angle_deg") p.max_ground_angle_deg = prm.as_double();
        else if (k == "normal_k") p.normal_k = static_cast<int>(prm.as_int());
        else if (k == "cluster_radius") p.cluster_radius = prm.as_double();
        else if (k == "min_cluster_size") p.min_cluster_size = static_cast<int>(prm.as_int());
        else if (k == "flat_obstacles_detected") p.flat_obstacles_detected = prm.as_bool();
        else if (k == "normals_segmentation") p.normals_segmentation = prm.as_bool();
        else if (k == "ground_normals_up") p.ground_normals_up = prm.as_double();
        else if (k == "normal_threads") p.normal_threads = static_cast<int>(prm.as_int());
        else if (k == "min_obstacle_height") {
          const double v = prm.as_double();
          p.min_obstacle_height = v < 0.0 ? std::numeric_limits<double>::quiet_NaN() : v;
        } else {
          res.successful = false;
          res.reason = full + " cannot be changed at runtime (restart the node)";
          return res;
        }
      } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        res.successful = false;
        res.reason = full + ": " + e.what();
        return res;
      }
      std::string err;
      if (pr->decimation < 1) err = "decimation must be >= 1";
      else if (p.voxel_size < 0 || p.cell_size <= 0) err = "voxel_size >= 0 and cell_size > 0";
      else if (p.normal_k < 3) err = "normal_k >= 3";
      else if (p.cluster_radius <= 0 || p.min_cluster_size < 1) err = "cluster_radius > 0, min_cluster_size >= 1";
      else if (p.max_obstacle_height > 0 && p.max_obstacle_height <= p.min_ground_height)
        err = "max_obstacle_height must be above min_ground_height";
      if (!err.empty()) {
        res.successful = false;
        res.reason = n + ": " + err;
        return res;
      }
    }
    profiles_.swap(next);
    wait_for_transform_ = wait;
    max_rate_hz_ = rate;
    return res;
  }

  static bool wanted(const rclcpp::Publisher<PointCloud2>::SharedPtr & pub)
  {
    return pub && pub->get_subscription_count() + pub->get_intra_process_subscription_count() > 0;
  }

  template<typename CloudT>
  void publish(const rclcpp::Publisher<PointCloud2>::SharedPtr & pub, const CloudT & cloud,
               const std_msgs::msg::Header & header)
  {
    auto msg = std::make_unique<PointCloud2>();
    pcl::toROSMsg(cloud, *msg);
    msg->header = header;
    pub->publish(std::move(msg));
  }

  void callback(const Image::ConstSharedPtr & depth, const CameraInfo::ConstSharedPtr & info)
  {
    namespace enc = sensor_msgs::image_encodings;
    const Clock::time_point t0 = Clock::now();
    std::lock_guard<std::mutex> lk(mutex_);
    if (max_rate_hz_ > 0.0) {
      const rclcpp::Time stamp(depth->header.stamp);
      if (last_stamp_.nanoseconds() > 0 && stamp > last_stamp_ &&
          (stamp - last_stamp_).seconds() < 1.0 / max_rate_hz_ * 0.95) {
        ++skipped_rate_;
        return;
      }
      last_stamp_ = stamp;
    }
    const bool is_mm = depth->encoding == enc::TYPE_16UC1 || depth->encoding == enc::MONO16;
    if (!is_mm && depth->encoding != enc::TYPE_32FC1) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Input type depth=32FC1,16UC1,MONO16 (got %s)",
                            depth->encoding.c_str());
      return;
    }
    if (depth->is_bigendian) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "big-endian depth images are not supported");
      return;
    }
    bool any = false;
    for (const auto & pr : profiles_) {
      any = any || wanted(pr.cloud_pub) || wanted(pr.obstacles_pub) || wanted(pr.ground_pub) || wanted(pr.rest_pub);
    }
    if (!any) return;  // nobody listens (RTAB-Map did the same)

    Intrinsics k;
    const bool p_set = info->p[0] != 0.0;
    k.fx = p_set ? info->p[0] : info->k[0];
    k.fy = p_set ? info->p[5] : info->k[4];
    k.cx = p_set ? info->p[2] : info->k[2];
    k.cy = p_set ? info->p[6] : info->k[5];
    k.width = static_cast<int>(info->width);
    k.height = static_cast<int>(info->height);
    if (!(k.fx > 0.0) || !(k.fy > 0.0)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "camera_info has no focal length");
      return;
    }

    // one projection per distinct decimation (shared by the profiles)
    std::map<int, DepthPoints> projected;
    // TF once per frame
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
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                                "%s: decimation %d does not divide the depth image %ux%u (camera_info %ux%u)",
                                pr.name.c_str(), pr.decimation, depth->width, depth->height, info->width,
                                info->height);
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
          const auto tf = tf_buffer_->lookupTransform(frame_id_, depth->header.frame_id, depth->header.stamp,
                                                      tf2::durationFromSec(wait_for_transform_));
          cam_to_base = tf2::transformToEigen(tf.transform);
          tf_ok = true;
        } catch (const tf2::TransformException & e) {
          ++tf_failures_;
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to get transform between %s and %s: %s",
                                frame_id_.c_str(), depth->header.frame_id.c_str(), e.what());
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

  void publish_stats()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = std::string(get_name()) + ": depth obstacles";
    st.hardware_id = "d435";
    auto kv = [&](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue x;
      x.key = k;
      x.value = v;
      st.values.push_back(x);
    };
    char buf[64];
    auto f = [&](double v) { std::snprintf(buf, sizeof(buf), "%.2f", v); return std::string(buf); };
    kv("frames", std::to_string(frames_));
    kv("frame_ms_avg", f(frames_ ? t_total_ / frames_ : 0.0));
    kv("skipped_rate_limit", std::to_string(skipped_rate_));
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
      pr.frames = 0;
      pr.t_cloud = pr.t_seg = pr.t_max = 0;
      pr.n_cloud = pr.n_ground = pr.n_obst = pr.n_rest = 0;
    }
    st.level = (frames_ == 0 || tf_failures_ > 0) ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                                   : diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = frames_ == 0 ? "no depth frames processed" : (tf_failures_ ? "TF failures" : "ok");
    frames_ = 0;
    t_total_ = 0;
    skipped_rate_ = tf_failures_ = 0;
    stats_pub_->publish(st);
  }

  using ApproxPolicy = message_filters::sync_policies::ApproximateTime<Image, CameraInfo>;
  using ExactPolicy = message_filters::sync_policies::ExactTime<Image, CameraInfo>;
  using ApproxSync = message_filters::Synchronizer<ApproxPolicy>;
  using ExactSync = message_filters::Synchronizer<ExactPolicy>;

  std::mutex mutex_;
  std::string frame_id_;
  double wait_for_transform_{0.2};
  double max_rate_hz_{0.0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<Profile> profiles_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<CameraInfo> info_sub_;
  std::shared_ptr<ApproxSync> approx_sync_;
  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  std::size_t frames_{0}, skipped_rate_{0}, tf_failures_{0};
  double t_total_{0};
};

}  // namespace vitulus_depth_obstacles

RCLCPP_COMPONENTS_REGISTER_NODE(vitulus_depth_obstacles::DepthObstaclesNode)
