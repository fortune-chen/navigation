#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "node.h"
#include "node_options.h"
#include "ros_log_sink.h"
#include "gflags/gflags.h"

#include "node_mgr_rtk.h"
#include "node_mgr_carto.h"
#include "dds_bridge_client.h"

#include "tf2_ros/transform_listener.h"
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "robot_state_publisher/robot_state_publisher.h"

#include "glog/logging.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(config_dir, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(config_mapping, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(config_localization, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

DEFINE_string(robot_model, "",
              "Basename, i.e. filename of a robot model file to load "
              "robot model file.");
DEFINE_string(sensor_bridge_ip, "",
              "connect to sensor bridge to get sensor data from ros space "
              "ip address.");
DEFINE_string(local_ip, "",
              "publish event from this ip "
              "ip address.");


using namespace cartographer_ros;

void PublishRobotState() {
  urdf::Model model;

  if (!model.initFile(FLAGS_robot_model)) {
    LOG(WARNING) << "Failed to init modelfile";
    return;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    LOG(WARNING) << "Failed to extract kdl tree from xml robot description";
    return;
  }

  robot_state_publisher::RobotStatePublisher state_publisher(tree, model);
 
  // use other thread to send fix transform
  std::thread t2([&state_publisher](){
    state_publisher.publishFixedTransforms(true);
    sleep(2);
  });
  t2.join();
}

void NodeRun() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  NodeOptions node_mapping_options;
  NodeOptions node_localization_options;
  TrajectoryOptions trajectory_mapping_options;
  TrajectoryOptions trajectory_localization_options;
  LOG(WARNING) << "FLAGS_config_dir: " << FLAGS_config_dir;
  LOG(WARNING) << "FLAGS_config_mapping: " << FLAGS_config_mapping;
  LOG(WARNING) << "FLAGS_config_localization: " << FLAGS_config_localization;
  LOG(WARNING) << "FLAGS_robot_model: " << FLAGS_robot_model;
  LOG(WARNING) << "FLAGS_sensor_bridge_ip: " << FLAGS_sensor_bridge_ip;
  LOG(WARNING) << "FLAGS_local_ip: " << FLAGS_local_ip;
  std::tie(node_mapping_options, trajectory_mapping_options) = LoadOptions(FLAGS_config_dir, FLAGS_config_mapping);
  std::tie(node_localization_options, trajectory_localization_options) = LoadOptions(FLAGS_config_dir, FLAGS_config_localization);
  ros::Time::init(); // temp code

  if (!FLAGS_robot_model.empty()) {
    PublishRobotState();
  }
  tf2_ros::Buffer &tf_buffer = tf2_ros::Buffer::getInstance();
  std::shared_ptr<mapping_node::DDSBridgeClient> ddsBridge =  std::make_shared<mapping_node::DDSBridgeClient>(FLAGS_local_ip, FLAGS_sensor_bridge_ip);

  std::shared_ptr<mapping_node::NodeMgrInterface> nodeMgr;

#if USED_GNSS_MAPPING
    nodeMgr = std::make_shared<mapping_node::NodeMgrRtk>(ddsBridge, &tf_buffer, node_mapping_options, node_localization_options, trajectory_mapping_options, trajectory_localization_options);
#else
    nodeMgr = std::make_shared<mapping_node::NodeMgrCarto>(ddsBridge, &tf_buffer, node_mapping_options, node_localization_options, trajectory_mapping_options, trajectory_localization_options);
#endif

  pause(); 
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_config_dir.empty()) << "-configuration_directory is missing.";
  CHECK(!FLAGS_config_mapping.empty()) << "-configuration_basename is missing.";
  CHECK(!FLAGS_config_localization.empty()) << "-configuration_basename is missing.";

  NodeRun();
}
