
#ifndef FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_MGR_RTK_H
#define FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_MGR_RTK_H

#include <map>
#include "tf_bridge.h"
#include "node_options.h"
#include "trajectory_options.h"
#include "slam_timer.h"
#include "rtk_fusion_node.h"
#include "dds_bridge_client.h"
#include "node_mgr_interface.h"
#include "absl/synchronization/mutex.h"

namespace mapping_node {

class NodeMgrRtk: public NodeMgrInterface  {
  public:
    NodeMgrRtk(std::shared_ptr<DDSBridgeClient> dds_bridge, tf2_ros::Buffer* tf_buffer, NodeOptions& node_mapping, NodeOptions& node_localization, TrajectoryOptions& traj_mapping, TrajectoryOptions& traj_localization);

    ~NodeMgrRtk();

    int commandProcess(MappingOption&) override;

  private:
    void startMapping() override;

    void reMapping(std::string&) override;

    void stopMapping() override;

    int saveMap(std::string&) override;

    void startLocalization(std::string&) override;

    void addPoseToPath(const rtk_fusion::State& state);

    void publishGridMap();

    bool saveToFile(std::string&);

    bool loadMap(std::string&);

    geometry_msgs::PoseStamped getPoseFromState(const rtk_fusion::State&);

    std::shared_ptr<rtk_fusion::RtkFusionNode> m_node;

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
    nav_msgs::Path m_path;
    rtk_fusion::PositionRange m_range;
    std::string m_lastMap;
    geometry_msgs::PoseStamped m_lastPose;
};

}

#endif  // FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_NODE_MGR_RTK_H
