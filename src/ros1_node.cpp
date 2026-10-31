// depth_obstacles (ROS1 Noetic): the D435 ground / obstacle clouds in one node instead of the RTAB-Map container
// RTABMAP_OBSTACLES (2x rtabmap_ros point_cloud_xyz + 2x obstacles_detection). The algorithm is the shared core
// (segmentation.cpp, the same file as the ROS2 package vitulus_depth_obstacles); the depth image is projected
// once and shared by the profiles (nav, wide, ...).
//
// Parameters: the ROS parameters are the defaults, re-read on ~reload (std_srvs/Empty). On top of them the changes
// made while the node runs (validated in live_config.cpp), kept in ~store_path:
//   ~set     std_msgs/String JSON {cmd: set, id, profile, values: {key: value}, reset: [key]} (all of it or nothing)
//            | {cmd: reset, id, profile, keys: [key]} (no keys = the whole profile) | {cmd: refresh}
//   ~config  std_msgs/String JSON (latched): values, defaults, changed keys and limits of every profile, the answer to
//            the last ~set (last), the reference load against ~frame_budget_ms, the state of the store
// A callback of the one ros::spin thread: a change lands between two frames, never inside one.
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <json/json.h>
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
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "vitulus_depth_obstacles/live_config.hpp"
#include "vitulus_depth_obstacles/segmentation.hpp"

using namespace vitulus_depth_obstacles;
using Clock = std::chrono::steady_clock;

namespace
{
double ms_since(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// one parameter value of the table as JSON (bool / int / double), the other way in parse_value
Json::Value json_value(const ParamDef & d, double v)
{
  if (d.type == ParamType::Bool) return Json::Value(v != 0.0);
  if (d.type == ParamType::Int) return Json::Value(static_cast<int>(v));
  return Json::Value(v);
}

bool parse_value(const ParamDef & d, const Json::Value & j, double & out, std::vector<Issue> & issues)
{
  Issue i;
  i.key = d.key;
  if (d.type == ParamType::Bool) {
    if (!j.isBool()) {
      i.code = "not_bool";
      issues.push_back(i);
      return false;
    }
    out = j.asBool() ? 1.0 : 0.0;
    return true;
  }
  if (!j.isNumeric() || j.isBool()) {
    i.code = "not_number";
    issues.push_back(i);
    return false;
  }
  out = j.asDouble();
  return true;
}

bool parse_json(const std::string & text, Json::Value & out)
{
  Json::Reader reader;
  return reader.parse(text, out, false) && out.isObject();
}

std::string to_json(const Json::Value & v)
{
  Json::FastWriter w;
  w.omitEndingLineFeed();
  return w.write(v);
}

Json::Value issue_json(const Issue & i)
{
  Json::Value e;
  e["key"] = i.key;
  e["code"] = i.code;
  e["value"] = i.value;
  e["a"] = i.a;
  e["b"] = i.b;
  e["other"] = i.other;
  return e;
}

bool same(double a, double b) { return std::abs(a - b) < 1e-9; }
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
    pnh_.param("frame_budget_ms", budget_ms_, 50.0);
    const char * home = std::getenv("HOME");
    pnh_.param<std::string>("store_path", store_path_,
                            std::string(home ? home : "/tmp") + "/.vitulus/depth_obstacles/settings.json");
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
    const std::map<std::string, ParamValues> stored = load_store();
    for (auto & pr : profiles_) {
      const auto it = stored.find(pr.name);
      adopt(pr, it == stored.end() ? ParamValues() : it->second);
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
    config_pub_ = pnh_.advertise<std_msgs::String>("config", 1, true);
    set_sub_ = pnh_.subscribe("set", 10, &DepthObstacles::on_set, this);
    publish_config();
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
    bool safety{true};        // feeds the navigation: the safe limits and the reference scene apply
    ProfileSettings s;        // what the frames use = defaults + overrides
    ProfileSettings defaults; // the ROS parameters
    ParamValues overrides;    // the live changes, only keys that differ from the defaults
    double load_ms{0};        // one reference frame with s
    ros::Publisher cloud_pub, obstacles_pub, ground_pub, rest_pub;
    std::size_t frames{0}, n_cloud{0}, n_ground{0}, n_obst{0}, n_rest{0};
    double t_cloud{0}, t_seg{0}, t_max{0};
  };

  void read_profile(Profile & pr)
  {
    const std::string n = pr.name + "/";
    ProfileSettings d;
    ChainParams & p = d.p;
    pnh_.param(n + "safety", pr.safety, true);
    pnh_.param(n + "decimation", d.decimation, 1);
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
    if (d.decimation < 1) d.decimation = 1;
    if (p.normal_k < 3) p.normal_k = 3;
    if (p.min_cluster_size < 1) p.min_cluster_size = 1;
    pr.defaults = d;
  }

  // Overrides from the store or kept over ~reload: limits may have moved since they were written, so they are checked
  // again and a set that fails falls back to the defaults (the file stays until the next change).
  void adopt(Profile & pr, const ParamValues & wanted_values)
  {
    ParamValues keep;
    const ParamValues base = to_values(pr.defaults);
    for (const auto & kv : wanted_values) {
      const ParamDef * d = find_param(kv.first);
      std::vector<Issue> issues;
      if (!d || !check_value(*d, kv.second, pr.safety, issues)) {
        store_error_ = "dropped " + pr.name + "/" + kv.first;
        continue;
      }
      if (!same(base.at(kv.first), kv.second)) keep[kv.first] = kv.second;
    }
    ProfileSettings s = with_values(pr.defaults, keep);
    if (!keep.empty()) {
      ReferenceResult r = run_reference(s);
      std::vector<Issue> issues = check_relations(s, pr.safety, image_rows_, image_cols_);
      if (issues.empty() && pr.safety) issues = check_reference(r);
      if (!issues.empty()) {
        store_error_ = "dropped " + pr.name + ": " + issues[0].code;
        ROS_WARN("depth_obstacles: stored settings of %s refused (%s), using the defaults", pr.name.c_str(),
                 issues[0].code.c_str());
        keep.clear();
        s = pr.defaults;
      }
    }
    pr.overrides = keep;
    pr.s = s;
    pr.load_ms = reference_ms(s);
  }

  // the store: {version, profiles: {name: {key: value}}}, only what differs from the defaults
  std::map<std::string, ParamValues> load_store()
  {
    std::map<std::string, ParamValues> out;
    if (store_path_.empty()) return out;
    std::ifstream f(store_path_);
    if (!f) return out;
    std::stringstream text;
    text << f.rdbuf();
    Json::Value root;
    if (!parse_json(text.str(), root)) {
      // never the reason a robot has no obstacle detection: keep the file for a look, run on the defaults
      store_error_ = "corrupt store";
      ROS_WARN("depth_obstacles: %s is not readable, using the defaults", store_path_.c_str());
      std::rename(store_path_.c_str(), (store_path_ + ".bad").c_str());
      return out;
    }
    const Json::Value & profiles = root["profiles"];
    if (!profiles.isObject()) return out;
    for (const auto & name : profiles.getMemberNames()) {
      const Json::Value & vals = profiles[name];
      if (!vals.isObject()) continue;
      for (const auto & key : vals.getMemberNames()) {
        const Json::Value & v = vals[key];
        if (v.isBool()) out[name][key] = v.asBool() ? 1.0 : 0.0;
        else if (v.isNumeric()) out[name][key] = v.asDouble();
      }
    }
    return out;
  }

  // write-to-temp + rename: a power cut leaves the old file or the new one, not half of it
  bool save_store()
  {
    if (store_path_.empty()) return true;
    Json::Value root;
    root["version"] = 1;
    root["profiles"] = Json::Value(Json::objectValue);
    for (const auto & pr : profiles_) {
      if (pr.overrides.empty()) continue;
      Json::Value & vals = root["profiles"][pr.name];
      for (const auto & kv : pr.overrides) vals[kv.first] = json_value(*find_param(kv.first), kv.second);
    }
    const std::string tmp = store_path_ + ".tmp";
    try {
      std::filesystem::create_directories(std::filesystem::path(store_path_).parent_path());
    } catch (const std::exception & e) {
      store_error_ = e.what();
      return false;
    }
    FILE * f = std::fopen(tmp.c_str(), "w");
    const std::string text = root.toStyledString();
    bool ok = f && std::fwrite(text.data(), 1, text.size(), f) == text.size() && std::fflush(f) == 0 &&
              fsync(fileno(f)) == 0;
    if (f) ok = std::fclose(f) == 0 && ok;
    ok = ok && std::rename(tmp.c_str(), store_path_.c_str()) == 0;
    store_error_ = ok ? "" : "cannot write " + store_path_;
    if (!ok) ROS_WARN("depth_obstacles: %s", store_error_.c_str());
    return ok;
  }

  // the fastest of two runs: one frame of a busy robot is not the cost of the settings
  static double reference_ms(const ProfileSettings & s)
  {
    return std::min(run_reference(s).ms, run_reference(s).ms);
  }

  void answer(const Json::Value & id, const std::string & cmd, const std::string & profile,
              const std::vector<Issue> & issues, const std::vector<std::string> & changed)
  {
    Json::Value l;
    l["id"] = id;
    l["cmd"] = cmd;
    l["profile"] = profile;
    l["ok"] = issues.empty();
    l["t"] = ros::Time::now().toSec();
    l["errors"] = Json::Value(Json::arrayValue);
    for (const auto & i : issues) l["errors"].append(issue_json(i));
    l["changed"] = Json::Value(Json::arrayValue);
    for (const auto & k : changed) l["changed"].append(k);
    last_ = l;
    publish_config();
    if (!issues.empty()) {
      ROS_WARN("depth_obstacles: %s %s refused: %s %s", cmd.c_str(), profile.c_str(), issues[0].key.c_str(),
               issues[0].code.c_str());
    } else if (!changed.empty()) {
      ROS_INFO("depth_obstacles: %s %s: %zu parameters changed", cmd.c_str(), profile.c_str(), changed.size());
    }
  }

  void on_set(const std_msgs::String::ConstPtr & msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    Json::Value j;
    Issue bad;
    if (!parse_json(msg->data, j)) {
      bad.code = "bad_json";
      answer(Json::Value(), "set", "", {bad}, {});
      return;
    }
    const std::string cmd = j.get("cmd", "set").asString();
    const std::string profile = j.get("profile", "").asString();
    const Json::Value id = j.get("id", Json::Value());
    if (cmd == "refresh") {
      publish_config();
      return;
    }
    auto it = std::find_if(profiles_.begin(), profiles_.end(), [&](const Profile & p) { return p.name == profile; });
    if ((cmd != "set" && cmd != "reset") || it == profiles_.end()) {
      bad.code = it == profiles_.end() ? "unknown_profile" : "unknown_cmd";
      bad.other = it == profiles_.end() ? profile : cmd;
      answer(id, cmd, profile, {bad}, {});
      return;
    }
    Profile & pr = *it;
    ParamValues next = pr.overrides;
    std::vector<Issue> issues;
    const Json::Value & keys = cmd == "reset" ? j["keys"] : j["reset"];
    if (cmd == "reset" && keys.isNull()) next.clear();
    if (keys.isArray()) {
      for (const auto & k : keys) {
        if (!find_param(k.asString())) {
          bad.key = k.asString();
          bad.code = "unknown_key";
          issues.push_back(bad);
        }
        next.erase(k.asString());
      }
    }
    const ParamValues base = to_values(pr.defaults);
    const Json::Value & vals = j["values"];
    if (vals.isObject()) {
      for (const auto & key : vals.getMemberNames()) {
        const ParamDef * d = find_param(key);
        double v = 0.0;
        if (!d) {
          bad.key = key;
          bad.code = "unknown_key";
          issues.push_back(bad);
        } else if (parse_value(*d, vals[key], v, issues) && check_value(*d, v, pr.safety, issues)) {
          if (same(base.at(key), v)) next.erase(key);
          else next[key] = v;
        }
      }
    }
    const ProfileSettings s = with_values(pr.defaults, next);
    // back to the defaults is always possible: they are what the owner runs on, whatever the limits say
    if (issues.empty() && !next.empty()) {
      issues = check_relations(s, pr.safety, image_rows_, image_cols_);
      if (issues.empty()) issues = preflight(pr, s);
    }
    if (!issues.empty()) {
      answer(id, cmd, profile, issues, {});
      return;
    }
    std::vector<std::string> changed;
    const ParamValues before = to_values(pr.s), after = to_values(s);
    for (const auto & kv : after) {
      if (!same(before.at(kv.first), kv.second)) changed.push_back(kv.first);
    }
    pr.overrides = next;
    pr.s = s;
    pr.load_ms = reference_ms(s);
    if (!changed.empty()) save_store();
    answer(id, cmd, profile, {}, changed);
  }

  // A change must leave a safety profile able to see, and must not add load the robot cannot carry: the reference
  // load of all profiles (they share the camera) against the budget. A change that costs about what the running
  // settings cost passes on a robot that is busy with something else.
  std::vector<Issue> preflight(const Profile & pr, const ProfileSettings & s)
  {
    const ReferenceResult r = run_reference(s);
    std::vector<Issue> out;
    if (pr.safety) out = check_reference(r);
    if (!out.empty()) return out;
    double others = 0.0;
    for (const auto & o : profiles_) {
      if (o.name != pr.name) others += o.load_ms;
    }
    const double cost = std::min(r.ms, run_reference(s).ms);
    if (cost + others > budget_ms_ && cost > 1.2 * reference_ms(pr.s)) {
      Issue i;
      i.code = "load";
      i.value = cost + others;
      i.a = budget_ms_;
      out.push_back(i);
    }
    return out;
  }

  void publish_config()
  {
    Json::Value root;
    root["version"] = 1;
    root["seq"] = static_cast<Json::UInt64>(++config_seq_);
    root["budget_ms"] = budget_ms_;
    root["store_path"] = store_path_;
    root["store_error"] = store_error_;
    root["last"] = last_;
    root["image"]["width"] = image_cols_;
    root["image"]["height"] = image_rows_;
    root["schema"] = Json::Value(Json::arrayValue);
    for (const auto & d : param_table()) {
      Json::Value e;
      e["key"] = d.key;
      e["group"] = d.group;
      e["unit"] = d.unit;
      e["type"] = d.type == ParamType::Bool ? "bool" : d.type == ParamType::Int ? "int" : "double";
      e["step"] = d.step;
      e["safe"].append(d.safe.min);
      e["safe"].append(d.safe.max);
      e["free"].append(d.free.min);
      e["free"].append(d.free.max);
      if (d.has_off) {
        e["off"] = d.off;
        e["off_safe"] = d.safe.off_allowed;
        e["off_free"] = d.free.off_allowed;
      }
      for (int c : d.choices) e["choices"].append(c);
      root["schema"].append(e);
    }
    root["profiles"] = Json::Value(Json::arrayValue);
    for (const auto & pr : profiles_) {
      Json::Value e;
      e["name"] = pr.name;
      e["safety"] = pr.safety;
      e["load_ms"] = pr.load_ms;
      const ParamValues vals = to_values(pr.s), defs = to_values(pr.defaults);
      for (const auto & d : param_table()) {
        e["values"][d.key] = json_value(d, vals.at(d.key));
        e["defaults"][d.key] = json_value(d, defs.at(d.key));
      }
      e["changed"] = Json::Value(Json::arrayValue);
      for (const auto & kv : pr.overrides) e["changed"].append(kv.first);
      root["profiles"].append(e);
    }
    std_msgs::String m;
    m.data = to_json(root);
    config_pub_.publish(m);
  }

  bool reload(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    pnh_.param("wait_for_transform", wait_for_transform_, wait_for_transform_);
    pnh_.param("frame_budget_ms", budget_ms_, budget_ms_);
    for (auto & pr : profiles_) {
      read_profile(pr);
      adopt(pr, ParamValues(pr.overrides));
    }
    publish_config();
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
    image_rows_ = static_cast<int>(depth->height);
    image_cols_ = static_cast<int>(depth->width);
    std::map<int, DepthPoints> projected;
    bool tf_done = false, tf_ok = false;
    Eigen::Isometry3d cam_to_base = Eigen::Isometry3d::Identity();
    for (auto & pr : profiles_) {
      const bool w_cloud = wanted(pr.cloud_pub), w_obst = wanted(pr.obstacles_pub), w_gnd = wanted(pr.ground_pub);
      const bool w_rest = wanted(pr.rest_pub);
      if (!w_cloud && !w_obst && !w_gnd && !w_rest) continue;
      const Clock::time_point tp = Clock::now();
      auto it = projected.find(pr.s.decimation);
      if (it == projected.end()) {
        DepthPoints dp;
        if (!project_depth(depth->data.data(), is_mm, static_cast<int>(depth->height),
                           static_cast<int>(depth->width), depth->step, k, pr.s.decimation, dp)) {
          ROS_ERROR_THROTTLE(5, "%s: decimation %d does not divide the depth image %ux%u", pr.name.c_str(),
                             pr.s.decimation, depth->width, depth->height);
          continue;
        }
        it = projected.emplace(pr.s.decimation, std::move(dp)).first;
      }
      Cloud::Ptr cloud = make_cloud(it->second, pr.s.p);
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
      segment(cloud, cam_to_base, pr.s.p, ground, obstacles, w_rest ? &rest : nullptr);
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
    kv("frame_budget_ms", f(budget_ms_));
    for (auto & pr : profiles_) {
      const double n = pr.frames ? static_cast<double>(pr.frames) : 1.0;
      kv(pr.name + ".frames", std::to_string(pr.frames));
      kv(pr.name + ".rest_active", wanted(pr.rest_pub) ? "1" : "0");
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
    const bool over = frames_ > 0 && t_total_ / frames_ > budget_ms_;
    st.level = (frames_ == 0 || tf_failures_ > 0 || over) ? diagnostic_msgs::DiagnosticStatus::WARN
                                                          : diagnostic_msgs::DiagnosticStatus::OK;
    st.message = frames_ == 0 ? "no depth frames processed"
                              : (tf_failures_ ? "TF failures" : (over ? "over the frame budget" : "ok"));
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
  ros::Publisher stats_pub_, config_pub_;
  ros::Subscriber set_sub_;
  ros::WallTimer stats_timer_;
  std::size_t frames_{0}, tf_failures_{0};
  double t_total_{0};
  double budget_ms_{50.0};
  int image_rows_{0}, image_cols_{0};   // 0 = no frame yet
  std::string store_path_, store_error_;
  Json::Value last_;
  std::size_t config_seq_{0};
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "depth_obstacles");
  DepthObstacles node;
  ros::spin();
  return 0;
}
