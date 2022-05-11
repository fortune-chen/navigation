#include <memory>
#include <dirent.h>
#include "mapping.h"
#include "NodeControlCommand.h"
#include "NodeControlCommandPubSubTypes.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"

using namespace flslam;

constexpr int kMaxStoredGripMap = 5;
constexpr char kStoredGripMapPath[] = "/var/tmp/flslam/map/";

#if USED_GNSS_MAPPING
constexpr char kMapTypeSuffix[] = ".trajectory";
#else
constexpr char kMapTypeSuffix[] = ".pbstream";
#endif

std::shared_ptr<NodeControlCommandPubSubType> node_control_type;
std::shared_ptr<DDSPublisher> node_control_publisher;

Mapping::Mapping() {
  //load pb stream map
  struct dirent *ptr;    
  DIR *dir;
  dir=opendir(kStoredGripMapPath);
  if (dir == NULL) {
    std::string command;
    std::string path = kStoredGripMapPath;
    command = "mkdir -p " + path;  
    system(command.c_str());
    return;
  }
  std::string path = kStoredGripMapPath;
  while((ptr=readdir(dir))!=NULL) {
  if(ptr->d_name[0] == '.')
  continue;
  std::string mapName = path + ptr->d_name;
  std::cout << "Found map: " << mapName << std::endl;
  std::size_t pos = mapName.find(kMapTypeSuffix);
  int id = atoi(mapName.substr(20, pos - 20).c_str());
  m_gridmaps.emplace(id, mapName);
  }
  closedir(dir);
  
  node_control_type = std::make_shared<NodeControlCommandPubSubType>();
  node_control_publisher = std::make_shared<DDSPublisher>(kNodeControlCommandTopicName, node_control_type);
}

Mapping::~Mapping() {
}

void Mapping::start() {
  NodeControlCommand cmd;
  cmd.type(NodeControlCommandType::Mapping);
  cmd.do_mapping(1);

  node_control_publisher->publish<NodeControlCommand>(cmd);
}

void Mapping::continueMapping(int id) {
  NodeControlCommand cmd;
  cmd.type(NodeControlCommandType::Mapping);
  cmd.do_mapping(2);
  cmd.map_path(m_gridmaps[id]);

  node_control_publisher->publish<NodeControlCommand>(cmd);
}
void Mapping::stop(){
    NodeControlCommand cmd;
    cmd.type(NodeControlCommandType::Mapping);
    cmd.do_mapping(0);
    node_control_publisher->publish<NodeControlCommand>(cmd);
}

int Mapping::saveMap() {
    NodeControlCommand cmd;
    cmd.type(NodeControlCommandType::Mapping);
    cmd.do_mapping(3);
    std::string path;
    auto id = getMapStorePath(path);
    cmd.map_path(path);
    node_control_publisher->publish<NodeControlCommand>(cmd);
    return id;
}
void Mapping::localization(int id) {
    NodeControlCommand cmd;
    cmd.type(NodeControlCommandType::Mapping);
    cmd.do_mapping(4);
    cmd.map_path(m_gridmaps[id]);

    node_control_publisher->publish<NodeControlCommand>(cmd);
}

int Mapping::getMapStorePath(std::string& path) {
  int mapOld, mapNew, mapId; 
  std::string mapPath = kStoredGripMapPath;

  mapOld = m_gridmaps.begin()->first;
  mapNew = m_gridmaps.begin()->first;

  for (auto iter : m_gridmaps) {
    if (mapOld > iter.first) {
      mapOld = iter.first;
    }
    if (mapNew < iter.first) {
      mapNew = iter.first;
    }
  }

  mapId = mapNew + 1;
  path = mapPath + std::to_string(mapId) + kMapTypeSuffix;

  if (m_gridmaps.size() >= kMaxStoredGripMap) {
    std::string command = "rm -f " + m_gridmaps[mapOld];  
    system(command.c_str());
    m_gridmaps.erase(mapOld);
  } 
  m_gridmaps.emplace(mapId, path);

  return mapId;
}

