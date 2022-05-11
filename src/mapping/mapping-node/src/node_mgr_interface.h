
#ifndef FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_MGR_INTERFACE_H
#define FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_MGR_INTERFACE_H

#include "slam_timer.h"
#include "dds_bridge_client.h"

namespace mapping_node {

enum class MappingStatus {
    IDLE = 0,
    MAPPING,
    LOCALIZATION,
};

struct MappingOption {
  int command;
  std::string map_path;
};

class NodeMgrInterface {
  public:
    virtual ~NodeMgrInterface() {};

    NodeMgrInterface(const NodeMgrInterface&) = delete;

    NodeMgrInterface& operator=(const NodeMgrInterface&) = delete;

    virtual int commandProcess(MappingOption&) = 0;

  protected:
    NodeMgrInterface() {}

    virtual void startMapping() = 0;

    virtual void reMapping(std::string&) = 0;

    virtual void stopMapping() = 0;

    virtual int saveMap(std::string&) = 0;

    virtual void startLocalization(std::string&) = 0;
};

}

#endif  // FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_MGR_INTERFACE_H
