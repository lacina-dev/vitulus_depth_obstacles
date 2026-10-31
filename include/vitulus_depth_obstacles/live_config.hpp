// What may be changed while the node runs, and what must not happen when it is: the parameter table (group, unit,
// step, limits), the validation of a change and a reference scene every change of a safety profile has to pass.
// No ROS and no JSON here; ros1_node.cpp carries it over ~set / ~config.
#pragma once

#include <map>
#include <string>
#include <vector>

#include "vitulus_depth_obstacles/segmentation.hpp"

namespace vitulus_depth_obstacles
{

// Everything a profile computes with.
struct ProfileSettings
{
  int decimation{1};
  ChainParams p;
};

// key -> value; a bool is 0 / 1, min_obstacle_height "off" is -1 (the value of the YAML; NaN in ChainParams).
using ParamValues = std::map<std::string, double>;

enum class ParamType { Double, Int, Bool };

struct ParamLimits
{
  double min, max;
  bool off_allowed;   // the off value of the parameter is allowed besides min..max
};

struct ParamDef
{
  const char * key;
  const char * group;   // distance, height, ground, noise, clusters, performance
  const char * unit;
  ParamType type;
  double step;
  ParamLimits safe;     // profile feeding the navigation: the detection cannot be switched off
  ParamLimits free;     // any other profile (mapping)
  bool has_off;
  double off;           // value that switches the step off
  std::vector<int> choices;   // not empty: only these values (min / max are their ends)
  double (*get)(const ProfileSettings &);
  void (*set)(ProfileSettings &, double);
};

const std::vector<ParamDef> & param_table();
const ParamDef * find_param(const std::string & key);

// One reason to refuse a change. The code is what the UI translates, the numbers are its arguments.
struct Issue
{
  std::string key;     // the parameter, empty = the whole set
  std::string code;    // unknown_key, not_number, not_integer, not_bool, below_min, above_max, not_choice, off_refused,
                       // order, window_short, obstacle_window, cluster_small, noise_small, no_divide, blind, phantom, load
  double value{0};
  double a{0}, b{0};   // limits or the other value
  std::string other;   // the other parameter
};

ParamValues to_values(const ProfileSettings & s);
ProfileSettings with_values(ProfileSettings base, const ParamValues & values);

// One value against the table (no relation to the other parameters). false = refused, the reason appended.
bool check_value(const ParamDef & def, double value, bool safety, std::vector<Issue> & issues);

// The relations between the parameters. rows / cols: size of the depth image, 0 = not known yet.
std::vector<Issue> check_relations(const ProfileSettings & s, bool safety, int rows, int cols);

// The reference scene: the Vitulus camera (0.29 m high, level), flat ground and a box 0.3 m wide and high 1.3 m ahead,
// once with the box and once without, through the whole chain.
struct ReferenceResult
{
  std::size_t box_points{0};      // obstacle points on the box
  std::size_t ground_points{0};   // ground points with the box in the scene
  std::size_t phantom_points{0};  // obstacle points on the empty ground
  double ms{0};                   // one frame of the scene with the box: projection + cloud + segmentation
};
ReferenceResult run_reference(const ProfileSettings & s);

// Empty = the settings see the box and take the flat ground for ground.
std::vector<Issue> check_reference(const ReferenceResult & r);

}  // namespace vitulus_depth_obstacles
