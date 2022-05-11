
#include <dirent.h>
#include "node_mgr_carto.h"

#include "cartographer/mapping/map_builder.h"

using namespace cartographer_ros;
using namespace mapping_node;

NodeMgrCarto::NodeMgrCarto(std::shared_ptr<DDSBridgeClient> dds_bridge, tf2_ros::Buffer* tf_buffer, NodeOptions& node_mapping, NodeOptions& node_localization, TrajectoryOptions& traj_mapping,TrajectoryOptions& traj_localization)
:m_nodeMappingOption(node_mapping), m_nodeLocalizationOption(node_localization), m_trajMappingOption(traj_mapping), m_trajLocalizationOption(traj_localization), m_status(MappingStatus::IDLE) {
  m_ddsBridge = dds_bridge;
  m_tfBuffer = tf_buffer;
  m_ddsBridge->registerControlCommandCallback([this](int command, std::string& path){
    MappingOption option;
    option.command = command;
    option.map_path = path;
    this->commandProcess(option);
  });
}

NodeMgrCarto::~NodeMgrCarto() {
}

void NodeMgrCarto::startMapping() {
  absl::MutexLock lock(&m_mutex);
  m_status = MappingStatus::MAPPING;

  auto map_builder = cartographer::mapping::CreateMapBuilder(m_nodeMappingOption.map_builder_options);
  m_node.reset(new Node(m_nodeMappingOption, m_trajMappingOption, std::move(map_builder), m_ddsBridge, m_tfBuffer, m_collectMetrics));

  m_trajectoryId = m_node->StartTrajectoryWithDefaultTopics(m_trajMappingOption);
  m_ddsBridge->registerOdomCallback([&](const nav_msgs::OdometryConstPtr& msg){
    m_node->HandleOdometryMessage(m_trajectoryId, "odom", msg);
  });
#if USED_MULTI_ECHO_SCAN
  m_ddsBridge->registerLaserScanCallback([&](const sensor_msgs::MultiEchoLaserScanConstPtr& msg){
    m_node->HandleMultiEchoLaserScanMessage(m_trajectoryId, "echoes", msg); });
#else
  m_ddsBridge->registerLaserScanCallback([&](const sensor_msgs::LaserScanConstPtr& msg){
    m_node->HandleLaserScanMessage(m_trajectoryId, "scan", msg); });
#endif
  m_ddsBridge->registerImuCallback([&](const sensor_msgs::ImuConstPtr& msg){
    m_node->HandleImuMessage(m_trajectoryId, "imu", msg);
  });
  m_ddsBridge->startSensor(0x1|0x2);

  m_publishMapTimer.StartTimer(2000, [&](){ // default value 1000ms
    m_node->PublishSubmapList(m_trajectoryId);
  });
  m_updatePoseByTrajectoryTimer.StartTimer(100, [&](){ // default value 5ms
    m_node->PublishLocalTrajectoryData();
  });
}

void NodeMgrCarto::reMapping(std::string& path) {
  absl::MutexLock lock(&m_mutex);
  m_status = MappingStatus::MAPPING;
  auto map_builder = cartographer::mapping::CreateMapBuilder(m_nodeMappingOption.map_builder_options);
  m_node.reset(new Node(m_nodeMappingOption, m_trajMappingOption, std::move(map_builder), m_ddsBridge, m_tfBuffer, m_collectMetrics));
  m_node->LoadState(path, true);
  m_trajectoryId = m_node->StartTrajectoryWithDefaultTopics(m_trajMappingOption);
  m_ddsBridge->registerOdomCallback([&](const nav_msgs::OdometryConstPtr& msg){
    m_node->HandleOdometryMessage(m_trajectoryId, "odom", msg);
  });
#if USED_MULTI_ECHO_SCAN
  m_ddsBridge->registerLaserScanCallback([&](const sensor_msgs::MultiEchoLaserScanConstPtr& msg){
    m_node->HandleMultiEchoLaserScanMessage(m_trajectoryId, "echoes", msg); });
#else
  m_ddsBridge->registerLaserScanCallback([&](const sensor_msgs::LaserScanConstPtr& msg){
    m_node->HandleLaserScanMessage(m_trajectoryId, "scan", msg); });
#endif
  m_ddsBridge->registerImuCallback([&](const sensor_msgs::ImuConstPtr& msg){
    m_node->HandleImuMessage(m_trajectoryId, "imu", msg);
  });
  m_ddsBridge->startSensor(0x1|0x2);

  m_publishMapTimer.StartTimer(2000, [&](){ // default value 1000ms
    m_node->PublishSubmapList(m_trajectoryId);
  });
  m_updatePoseByTrajectoryTimer.StartTimer(100, [&](){ // default value 5ms
    m_node->PublishLocalTrajectoryData();
  });
}

void NodeMgrCarto::stopMapping() {
  absl::MutexLock lock(&m_mutex);
  m_ddsBridge->startSensor(0);
  m_publishMapTimer.Expire();
  m_updatePoseByTrajectoryTimer.Expire();
  m_node.reset();
  m_status = MappingStatus::IDLE;
}
int NodeMgrCarto::saveMap(std::string& path) {
  absl::MutexLock lock(&m_mutex);
  if (m_node.get() == nullptr) {
    LOG(INFO) << path;
    LOG(INFO) << "Not in mapping status, Please mapping first";
    return -1;
  }
  cartographer_ros_msgs::WriteState::Request request;
  cartographer_ros_msgs::WriteState::Response response;
  request.filename = path;

  m_ddsBridge->startSensor(0);
  m_node->FinishAllTrajectories();
  m_publishMapTimer.Expire();
  m_updatePoseByTrajectoryTimer.Expire();
  auto ret = m_node->HandleWriteState(request, response);
  m_node.reset();
  LOG(INFO) << "ret : " << ret << " response status:" << response.status.message;
  m_status = MappingStatus::IDLE;
}

void NodeMgrCarto::startLocalization(std::string& path) {
  absl::MutexLock lock(&m_mutex);
  auto map_builder = cartographer::mapping::CreateMapBuilder(m_nodeLocalizationOption.map_builder_options);
  m_node.reset(new Node(m_nodeLocalizationOption, m_trajLocalizationOption, std::move(map_builder), m_ddsBridge, m_tfBuffer, m_collectMetrics));

  LOG(INFO) << "localization with map: " << path;
  m_node->LoadState(path, true);

  TrajectoryOptions trajectory_options = m_trajLocalizationOption;
  ::cartographer::mapping::proto::TrajectoryBuilderOptions_PureLocalizationTrimmerOptions trimmer;
    trimmer.set_max_submaps_to_keep(3);
  *trajectory_options.trajectory_builder_options.mutable_pure_localization_trimmer() = trimmer;

  m_trajectoryId = m_node->StartTrajectoryWithDefaultTopics(trajectory_options);
#if USED_MULTI_ECHO_SCAN
  m_ddsBridge->registerLaserScanCallback([&](const sensor_msgs::MultiEchoLaserScanConstPtr& msg){
    m_node->HandleMultiEchoLaserScanMessage(m_trajectoryId, "echoes", msg); });
#else
  m_ddsBridge->registerLaserScanCallback([&](const sensor_msgs::LaserScanConstPtr& msg){
    m_node->HandleLaserScanMessage(m_trajectoryId, "scan", msg); });
#endif
  m_ddsBridge->registerImuCallback([&](const sensor_msgs::ImuConstPtr& msg){
    m_node->HandleImuMessage(m_trajectoryId, "imu", msg);
  });
  m_ddsBridge->startSensor(0x1|0x2);

  m_publishMapTimer.StartTimer(10000, [&](){ // default value 1000ms
    m_node->PublishSubmapList(m_trajectoryId);
  });

  m_updatePoseByTrajectoryTimer.StartTimer(100, [&](){ // default value 5ms
    m_node->PublishLocalTrajectoryData();
  });
  m_status = MappingStatus::LOCALIZATION;
}

int NodeMgrCarto::commandProcess(MappingOption& option) {
  switch (option.command) {
    case 0: {
      if (m_status != MappingStatus::IDLE) {
        stopMapping();
      }
    } break;
    case 1: {
      if (m_status == MappingStatus::MAPPING) {
        stopMapping();
      }
      startMapping();
    } break;
    case 2: {
      reMapping(option.map_path);
    } break;
    case 3: {
      saveMap(option.map_path);
    } break;
    case 4: {
      if (m_status == MappingStatus::MAPPING) {
        stopMapping();
      }
      startLocalization(option.map_path);
    } break;
    default: break;
  }
}
