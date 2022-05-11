#ifndef FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_OPTIONS_H
#define FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_OPTIONS_H

#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "trajectory_options.h"

struct NodeOptions {
  ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  std::string map_frame;
  double lookup_transform_timeout_sec;
  double submap_publish_period_sec;
  double pose_publish_period_sec;
  double trajectory_publish_period_sec;
  bool publish_to_tf = true;
  bool publish_tracked_pose = false;
  bool use_pose_extrapolator = true;
};

NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

#endif  // FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_OPTIONS_H
