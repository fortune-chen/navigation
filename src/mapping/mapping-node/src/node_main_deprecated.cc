/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "node.h"
#include "node_options.h"
#include "ros_log_sink.h"
#include "gflags/gflags.h"

#include "tf2_ros/transform_listener.h"
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "robot_state_publisher/robot_state_publisher.h"

#include "glog/logging.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
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

DEFINE_string(robot_model_filename, "",
              "Basename, i.e. filename of a robot model file to load "
              "robot model file.");
DEFINE_string(sensor_bridge_ip, "",
              "connect to sensor bridge to get sensor data from ros space "
              "ip address.");
DEFINE_string(local_ip, "",
              "publish event from this ip "
              "ip address.");

namespace cartographer_ros {
namespace {

void StartRobot() {
  urdf::Model model;

  if (!model.initFile(FLAGS_robot_model_filename)) {
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
    sleep(1);
  });
  t2.join();
}

void Run() {
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  // tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  // tf2_ros::TransformListener tf(tf_buffer);
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  LOG(WARNING) << "FLAGS_configuration_directory: " << FLAGS_configuration_directory;
  LOG(WARNING) << "FLAGS_configuration_basename: " << FLAGS_configuration_basename;
  LOG(WARNING) << "FLAGS_robot_model_filename: " << FLAGS_robot_model_filename;
  LOG(WARNING) << "FLAGS_sensor_bridge_ip: " << FLAGS_sensor_bridge_ip;
  LOG(WARNING) << "FLAGS_local_ip: " << FLAGS_local_ip;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  ros::Time::init(); // temp code

  if (!FLAGS_robot_model_filename.empty()) {
    StartRobot();
  }

  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  tf2_ros::Buffer &tf_buffer = tf2_ros::Buffer::getInstance();
  Node node(node_options, trajectory_options, std::move(map_builder), &tf_buffer,
          FLAGS_collect_metrics, FLAGS_local_ip, FLAGS_sensor_bridge_ip);
  if (!FLAGS_load_state_filename.empty()) {
    node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  // if (FLAGS_start_trajectory_with_default_topics) {
  //   node.StartTrajectoryWithDefaultTopics(trajectory_options);
  // }

  pause(); // temp code

  // node.FinishAllTrajectories();
  // node.RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node.SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros
//#include "ros/init.h"
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  //::ros::init(argc, argv, "cartographer_node");
  //::ros::start();
  //cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();

}
