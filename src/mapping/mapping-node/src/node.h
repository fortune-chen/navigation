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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <atomic>
#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "map_builder_bridge.h"
#include "metrics/family_factory.h"
#include "node_constants.h"
#include "node_options.h"
#include "trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/WriteState.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"
#include "cartographer/io/submap_painter.h"

#include "dds_bridge_client.h"

#include "slam_timer.h"

namespace cartographer_ros {

using ::cartographer::mapping::SubmapId;
using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;

class Node {
 public:
  Node(const NodeOptions& node_options, const TrajectoryOptions& trajectory_options,
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
       std::shared_ptr<mapping_node::DDSBridgeClient> dds_bridge,
       tf2_ros::Buffer* tf_buffer, bool collect_metrics);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  void FinishAllTrajectories();

  bool FinishTrajectory(int trajectory_id);

  void RunFinalOptimization();

  int StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  std::vector<
      std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
  ComputeDefaultSensorIdsForMultipleBags(
      const std::vector<TrajectoryOptions>& bags_options) const;

  int AddOfflineTrajectory(
      const std::set<
          cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& options);

  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const nav_msgs::Odometry::ConstPtr& msg);
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::NavSatFix::ConstPtr& msg);
  void HandleLandmarkMessage(
      int trajectory_id, const std::string& sensor_id,
      const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::Imu::ConstPtr& msg);
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const std::string& sensor_id,
      const sensor_msgs::MultiEchoLaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::PointCloud2::ConstPtr& msg);

  void SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  void LoadState(const std::string& state_filename, bool load_frozen_state);

  void PublishSubmapList(int trajectory_id);

  void PublishLocalTrajectoryData();

  bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
                        cartographer_ros_msgs::WriteState::Response& response);

 private:
  struct Subscriber {
    std::string topic;
  };

  bool HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleTrajectoryQuery(
      ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
      ::cartographer_ros_msgs::TrajectoryQuery::Response& response);
  bool HandleStartTrajectory(
      cartographer_ros_msgs::StartTrajectory::Request& request,
      cartographer_ros_msgs::StartTrajectory::Response& response);
  bool HandleFinishTrajectory(
      cartographer_ros_msgs::FinishTrajectory::Request& request,
      cartographer_ros_msgs::FinishTrajectory::Response& response);

  bool HandleGetTrajectoryStates(
      ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
      ::cartographer_ros_msgs::GetTrajectoryStates::Response& response);
  bool HandleReadMetrics(
      cartographer_ros_msgs::ReadMetrics::Request& request,
      cartographer_ros_msgs::ReadMetrics::Response& response);

  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(const TrajectoryOptions& options) const;
  int AddTrajectory(const TrajectoryOptions& options);
  void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);

  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);

  void HandlerSubmapList(cartographer_ros_msgs::SubmapList& msg, int trajectory_id);
  void PublishGridmap(std::map<SubmapId, SubmapSlice>&submapSlices);
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const TrajectoryOptions& options);
  cartographer_ros_msgs::StatusResponse FinishTrajectoryUnderLock(
      int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  std::unique_ptr<::cartographer::io::SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id);

  cartographer_ros_msgs::StatusResponse TrajectoryStateToStatus(
      int trajectory_id,
      const std::set<
          cartographer::mapping::PoseGraphInterface::TrajectoryState>&
          valid_states);
  const NodeOptions node_options_;
  const TrajectoryOptions trajectory_options_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  absl::Mutex mutex_;

  std::shared_ptr<mapping_node::DDSBridgeClient> dds_bridge_;

  std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
  MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);

  std::string last_frame_id_;
  ros::Time last_timestamp_;
  double resolution_;

  int trajectory_id_;

  Timer publishMapTimer_;
  Timer updatePoseByTrajectoryTimer_;

  struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                             const double odometry_sampling_ratio,
                             const double fixed_frame_pose_sampling_ratio,
                             const double imu_sampling_ratio,
                             const double landmark_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
          imu_sampler(imu_sampling_ratio),
          landmark_sampler(landmark_sampling_ratio) {}

    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
    ::cartographer::common::FixedRatioSampler landmark_sampler;
  };

  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::map<int, ::ros::Time> last_published_tf_stamps_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_set<int> trajectories_scheduled_for_finish_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
