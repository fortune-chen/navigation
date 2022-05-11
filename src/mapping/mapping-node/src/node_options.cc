
#include "node_options.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "glog/logging.h"


NodeOptions CreateNodeOptions(
    ::cartographer::common::LuaParameterDictionary* const
        lua_parameter_dictionary) {
  NodeOptions options;
  options.map_builder_options =
      ::cartographer::mapping::CreateMapBuilderOptions(
          lua_parameter_dictionary->GetDictionary("map_builder").get());
  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
      lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
      lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
  options.trajectory_publish_period_sec =
      lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
  if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
    options.publish_to_tf =
        lua_parameter_dictionary->GetBool("publish_to_tf");
  }
  if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
    options.publish_tracked_pose =
        lua_parameter_dictionary->GetBool("publish_tracked_pose");
  }
  if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
    options.use_pose_extrapolator =
        lua_parameter_dictionary->GetBool("use_pose_extrapolator");
  }
  return options;
}

std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
                         CreateTrajectoryOptions(&lua_parameter_dictionary));
}

