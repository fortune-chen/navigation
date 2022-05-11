
#ifndef FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_MGR_CARTO_H
#define FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_MGR_CARTO_H

#include <map>
#include "node.h"
#include "slam_timer.h"
#include "node_mgr_interface.h"
#include "dds_bridge_client.h"

namespace mapping_node {

class NodeMgrCarto: public NodeMgrInterface {
  public:
    NodeMgrCarto(std::shared_ptr<DDSBridgeClient> dds_bridge, tf2_ros::Buffer* tf_buffer, NodeOptions& node_mapping, NodeOptions& node_localization, TrajectoryOptions& traj_mapping, TrajectoryOptions& traj_localization);

    ~NodeMgrCarto();

    int commandProcess(MappingOption&) override;

  private:
    void startMapping() override;

    void reMapping(std::string&) override;

    void stopMapping() override;

    int saveMap(std::string&) override;

    void startLocalization(std::string&) override;

    bool m_collectMetrics;

    std::shared_ptr<cartographer_ros::Node> m_node;

    MappingStatus m_status;
    std::shared_ptr<DDSBridgeClient> m_ddsBridge;

    absl::Mutex m_mutex;
    int m_trajectoryId;
    NodeOptions m_nodeMappingOption;
    NodeOptions m_nodeLocalizationOption;
    TrajectoryOptions m_trajMappingOption;
    TrajectoryOptions m_trajLocalizationOption;
    tf2_ros::Buffer* m_tfBuffer;
    Timer m_publishMapTimer;
    Timer m_updatePoseByTrajectoryTimer;
};

}

#endif  // FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_MGR_CARTO_H
