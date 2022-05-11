#ifndef FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_TRAJECTORY_OPTIONS_H
#define FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_TRAJECTORY_OPTIONS_H

#include <string>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

struct TrajectoryOptions {
  ::cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectory_builder_options;
  std::string tracking_frame;
  std::string published_frame;
  std::string odom_frame;
  bool provide_odom_frame;
  bool use_odometry;
  bool use_nav_sat;
  bool use_landmarks;
  bool publish_frame_projected_to_2d;
  int num_laser_scans;
  int num_multi_echo_laser_scans;
  int num_subdivisions_per_laser_scan;
  int num_point_clouds;
  double rangefinder_sampling_ratio;
  double odometry_sampling_ratio;
  double fixed_frame_pose_sampling_ratio;
  double imu_sampling_ratio;
  double landmarks_sampling_ratio;
  double acc_noise;
  double gyro_noise;
  double acc_bias_noise;
  double gyro_bias_noise;
};

TrajectoryOptions CreateTrajectoryOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

#endif  // FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_TRAJECTORY_OPTIONS_H
