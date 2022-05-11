
#include <dirent.h>
#include <fstream>
#include "node_mgr_rtk.h"

#include "cartographer/mapping/map_builder.h"

using namespace rtk_fusion;
using namespace mapping_node;
using namespace cartographer_ros;

NodeMgrRtk::NodeMgrRtk(std::shared_ptr<DDSBridgeClient> dds_bridge, tf2_ros::Buffer* tf_buffer, NodeOptions& node_mapping, NodeOptions& node_localization, TrajectoryOptions& traj_mapping,TrajectoryOptions& traj_localization)
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

NodeMgrRtk::~NodeMgrRtk() {}

//static int imu_count = 0;
void NodeMgrRtk::startMapping() {
  absl::MutexLock lock(&m_mutex);
  m_status = MappingStatus::MAPPING;
  LOG(INFO) << "start mapping";
  double x, y, z;
  x = y = z = 0.;
  const Eigen::Vector3d imu_to_gps(x, y, z);
  m_range = {0., 0., 0., 0.};
  m_path.poses.clear();
  m_path = {};
  m_node.reset(new RtkFusionNode(m_trajMappingOption.acc_noise, m_trajMappingOption.gyro_noise, m_trajMappingOption.acc_bias_noise, m_trajMappingOption.gyro_bias_noise, imu_to_gps));

  m_ddsBridge->registerGnssCallback([&](const sensor_msgs::NavSatFixConstPtr& msg){
    m_node->HandleNavSatFixMessage(msg);});
  m_ddsBridge->registerImuCallback([&](const sensor_msgs::ImuConstPtr& msg){
    rtk_fusion::State state;
    //if (imu_count++ == 10) {
      if (m_node->HandleImuMessage(msg, &state)) {
        addPoseToPath(state);
      }
    //  imu_count = 0;
    //}
  });

  // initial pose add to path
  geometry_msgs::PoseStamped init_pose;
  init_pose.header.frame_id = "map";
  init_pose.header.stamp = ros::Time::now();
  m_path.poses.push_back(init_pose);

  m_ddsBridge->startSensor(0x2|0x8);
  m_publishMapTimer.StartTimer(1000, [&](){
    publishGridMap();
  });
  m_updatePoseByTrajectoryTimer.StartTimer(200, [&](){
    geometry_msgs::PoseStamped pose = m_path.poses.back();
    //LOG(INFO) << "pose position x: " << pose.pose.position.x << " y: " << pose.pose.position.y;
    //const Eigen::Quaterniond quaterpose(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    //LOG(INFO) << "pose orientation: " << quaterpose.matrix().eulerAngles(2,1,0);
    m_ddsBridge->postPose(pose);
  });
}

void NodeMgrRtk::reMapping(std::string& path) {
  absl::MutexLock lock(&m_mutex);
  m_status = MappingStatus::MAPPING;
  LOG(INFO) << "continue mapping with: " << path;
  double x, y, z;
  x = y = z = 0.;
  const Eigen::Vector3d imu_to_gps(x, y, z);
  m_node.reset(new RtkFusionNode(m_trajMappingOption.acc_noise, m_trajMappingOption.gyro_noise, m_trajMappingOption.acc_bias_noise, m_trajMappingOption.gyro_bias_noise, imu_to_gps));

  if (!loadMap(path)) {
    return;
  }

  m_ddsBridge->registerGnssCallback([&](const sensor_msgs::NavSatFixConstPtr& msg){
    m_node->HandleNavSatFixMessage(msg);});

  m_ddsBridge->registerImuCallback([&](const sensor_msgs::ImuConstPtr& msg){
    rtk_fusion::State state;
    if (m_node->HandleImuMessage(msg, &state)) {
      addPoseToPath(state);
    }
  });

  m_ddsBridge->startSensor(0x2|0x8);
  m_publishMapTimer.StartTimer(1000, [&](){
    publishGridMap();
  });
  m_updatePoseByTrajectoryTimer.StartTimer(100, [&](){
    geometry_msgs::PoseStamped pose = m_path.poses.back();
    //LOG(INFO) << "pose.position x: " << pose.pose.position.x << " y: " << pose.pose.position.y;
    m_ddsBridge->postPose(pose);
  });
}

void NodeMgrRtk::stopMapping() {
  absl::MutexLock lock(&m_mutex);
  LOG(INFO) << "stop mapping";
  m_ddsBridge->startSensor(0);
  m_publishMapTimer.Expire();
  m_updatePoseByTrajectoryTimer.Expire();
  m_node.reset();
  m_path.poses.clear();

  m_status = MappingStatus::IDLE;
}

int NodeMgrRtk::saveMap(std::string& path) {
  absl::MutexLock lock(&m_mutex);
  LOG(INFO) << "save map to: " << path;
  if (m_node.get() == nullptr) {
    LOG(INFO) << path;
    LOG(INFO) << "Not in mapping status, Please mapping first";
    return -1;
  }
  m_publishMapTimer.Expire();
  m_updatePoseByTrajectoryTimer.Expire();

  m_ddsBridge->startSensor(0);

  if (!saveToFile(path)) {
    return -1;
  }

  m_node.reset();
  m_path.poses.clear();
  m_status = MappingStatus::IDLE;
}

void NodeMgrRtk::startLocalization(std::string& path) {
  absl::MutexLock lock(&m_mutex);
  LOG(INFO) << "localization with: " << path;
  double x, y, z;
  x = y = z = 0.;
  const Eigen::Vector3d imu_to_gps(x, y, z);
  m_node.reset();
  m_node.reset(new RtkFusionNode(m_trajMappingOption.acc_noise, m_trajMappingOption.gyro_noise, m_trajMappingOption.acc_bias_noise, m_trajMappingOption.gyro_bias_noise, imu_to_gps));

  if (!loadMap(path)) {
    return;
  }

  publishGridMap();
  m_lastPose = {};
  m_ddsBridge->registerGnssCallback([&](const sensor_msgs::NavSatFixConstPtr& msg){
    m_node->HandleNavSatFixMessage(msg);});

  m_ddsBridge->registerImuCallback([&](const sensor_msgs::ImuConstPtr& msg){
    rtk_fusion::State state;
    if (m_node->HandleImuMessage(msg, &state)) {
      auto pose = getPoseFromState(state);
      Eigen::Vector3d delta_position(m_lastPose.pose.position.x - pose.pose.position.x, m_lastPose.pose.position.y - pose.pose.position.y, 0);
      const Eigen::Quaterniond quaterpose(m_lastPose.pose.orientation.w, m_lastPose.pose.orientation.x, m_lastPose.pose.orientation.y, m_lastPose.pose.orientation.z);
      Eigen::Vector3d delta_orientation = quaterpose.matrix().eulerAngles(2,1,0) - state.G_R_I.matrix().eulerAngles(2,1,0);

      if (delta_position.norm() > 1e-3 || delta_orientation.norm() > 1e-2) {
      // LOG(INFO) << "pose.position x: " << pose.pose.position.x << " y: " << pose.pose.position.y;
      // LOG(INFO) << "cur angle: " << state.G_R_I.matrix().eulerAngles(2,1,0);
        m_ddsBridge->postPose(pose);
        m_lastPose = pose;
      }
    }
  });

  m_ddsBridge->startSensor(0x2|0x8);
  m_status = MappingStatus::LOCALIZATION;
}

int NodeMgrRtk::commandProcess(MappingOption& option) {
  switch (option.command) {
    case 0: {
      if (m_status != MappingStatus::IDLE) {
        stopMapping();
      }
    } break;
    case 1: {
      if (m_status != MappingStatus::IDLE) {
        stopMapping();
      }
      startMapping();
    } break;
    case 2: {
      if (m_status != MappingStatus::IDLE) {
        stopMapping();
      }
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

void NodeMgrRtk::addPoseToPath(const rtk_fusion::State& state) {
  geometry_msgs::PoseStamped last_pose = m_path.poses.back();
  auto pose = getPoseFromState(state);

  Eigen::Vector3d delta_position(last_pose.pose.position.x - pose.pose.position.x, last_pose.pose.position.y - pose.pose.position.y, 0);
  const Eigen::Quaterniond quaterpose(last_pose.pose.orientation.w,last_pose.pose.orientation.x,last_pose.pose.orientation.y,last_pose.pose.orientation.z);
  Eigen::Vector3d delta_orientation = quaterpose.matrix().eulerAngles(2,1,0) - state.G_R_I.matrix().eulerAngles(2,1,0);

  if (delta_position.norm() > 1e-3 || delta_orientation.norm() > 1e-2) {
    m_range = state.range;
    // LOG(INFO) << "pose.position x: " << pose.pose.position.x << " y: " << pose.pose.position.y;
    // LOG(INFO) << "cur angle: " << state.G_R_I.matrix().eulerAngles(2,1,0);
    m_path.poses.push_back(pose);
  }
}

void NodeMgrRtk::publishGridMap() {
  if (m_path.poses.size() < 5) {
    return;
  }
  nav_msgs::OccupancyGrid grid;
  nav_msgs::Path path = m_path;
  rtk_fusion::PositionRange range = m_range;
  grid.header.stamp =  path.header.stamp;
  grid.header.frame_id = path.header.frame_id;
  grid.info.map_load_time = ros::Time::now();
  grid.info.resolution = m_trajMappingOption.trajectory_builder_options.trajectory_builder_2d_options().submaps_options().grid_options_2d().resolution();
  grid.info.width = (range.max_x - range.min_x) / grid.info.resolution + 4;
  grid.info.height = (range.max_y - range.min_y) / grid.info.resolution + 4;
  grid.info.origin.position.x = range.min_x - (2 * grid.info.resolution);
  grid.info.origin.position.y = range.min_y - (2 * grid.info.resolution);
  grid.info.origin.position.z = 0.;
  grid.info.origin.orientation.w = 1.;
  grid.info.origin.orientation.x = 0.;
  grid.info.origin.orientation.y = 0.;
  grid.info.origin.orientation.z = 0.;
  size_t length = grid.info.width * grid.info.height;
  grid.data.reserve(grid.info.width * grid.info.height);
  for (int i = 0; i < grid.info.width * grid.info.height; i++) {
    grid.data.push_back(-1);
  }
  for (auto pose : m_path.poses) {
    unsigned int delta_y = (pose.pose.position.y - range.min_y) / grid.info.resolution;
    unsigned int delta_x = (pose.pose.position.x - range.min_x) / grid.info.resolution;
    grid.data[(delta_y + 2) * grid.info.width + delta_x + 2] = 100;
  }
  m_ddsBridge->postGridMap(grid);
}

bool NodeMgrRtk::saveToFile(std::string& path) {
  auto data = ros::serialization::serializeMessage(m_path);
  std::string content;
  content.assign(reinterpret_cast<char*>(data.message_start), data.num_bytes);
  rtk_fusion::PositionRange range = m_range;
  std::ofstream fout;
  fout.open(path);

  auto lla = m_node->getInitLLA();
  fout << std::fixed << std::setprecision(15) << lla[0] << "\n" << lla[1] << "\n" << lla[2] << "\n" << range.min_x << "\n" << range.max_x << "\n" << range.min_y << "\n" << range.max_y << "\n" << content;
  if (!fout.good()) {
    return false;
  }
  fout.close();
  m_lastMap = path;
  return true;
}

bool NodeMgrRtk::loadMap(std::string& path) {
  std::string content;
  Eigen::Vector3d lla;

  const int MAX = 80;
  char buffer[MAX];
  std::ifstream fin;
  fin.open(path);
  if(!fin.is_open() || !fin.good()) {
    LOG(INFO) << "map is not exist";
    return false;
  }

  char* str_end;
  fin.getline(buffer,MAX);
  lla[0] = strtod(buffer, &str_end);
  fin.getline(buffer,MAX);
  lla[1] = strtod(buffer, &str_end);
  fin.getline(buffer,MAX);
  lla[2] = strtod(buffer, &str_end);

  fin.getline(buffer,MAX);
  m_range.min_x = strtod(buffer, &str_end);
  fin.getline(buffer,MAX);
  m_range.max_x = strtod(buffer, &str_end);
  fin.getline(buffer,MAX);
  m_range.min_y = strtod(buffer, &str_end);
  fin.getline(buffer,MAX);
  m_range.max_y = strtod(buffer, &str_end);

  LOG(INFO) << std::fixed << std::setprecision(15) << lla[0] << " - " << lla[1] << " - " << lla[2] << m_range.min_x << " - " << m_range.max_x << " - " << m_range.min_y << " - " << m_range.max_y;

  m_node->setInitLLA(lla, m_range);

  fin.seekg(0, std::ios::cur);
  int headerLen = fin.tellg();
  fin.seekg(0, std::ios::end);
  int nFileLen = fin.tellg();
  fin.seekg(0, std::ios::beg);
  char* pFileBuf = new char[nFileLen];
  fin.read(pFileBuf, nFileLen);

  content.assign(pFileBuf + headerLen, nFileLen - headerLen);
  delete pFileBuf;

  nav_msgs::Path data;

  ros::serialization::IStream strm((uint8_t*)content.data(), content.length());
  ros::serialization::deserialize(strm, data);
  m_path = data;
  return true;
}

geometry_msgs::PoseStamped NodeMgrRtk::getPoseFromState(const rtk_fusion::State& state) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = state.G_p_I[0];
  pose.pose.position.y = state.G_p_I[1];
  pose.pose.position.z = state.G_p_I[2];
  const Eigen::Quaterniond G_q_I(state.G_R_I);
  pose.pose.orientation.x = G_q_I.x();
  pose.pose.orientation.y = G_q_I.y();
  pose.pose.orientation.z = G_q_I.z();
  pose.pose.orientation.w = G_q_I.w();
  return pose;
}
