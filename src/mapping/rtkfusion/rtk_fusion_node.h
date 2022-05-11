#ifndef FLSLAM_SRC_MAPPING_RTKFUSION_RTK_FUSION_NODE_H
#define FLSLAM_SRC_MAPPING_RTKFUSION_RTK_FUSION_NODE_H

#include <Eigen/Core>

#include "base_type.h"
#include "gps_processor.h"
#include "imu_processor.h"
#include "gnss_imu_mixer.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

namespace rtk_fusion {

class RtkFusionNode {
public:
    RtkFusionNode(double acc_noise, double gyro_noise, double acc_bias_noise, double gyro_bias_noise, const Eigen::Vector3d& imu_to_gps);
    ~RtkFusionNode() { }

    bool HandleImuMessage(const sensor_msgs::Imu::ConstPtr& msg, State* fused_state);

    bool HandleNavSatFixMessage(const sensor_msgs::NavSatFix::ConstPtr& msg);

    Eigen::Vector3d getInitLLA();

    void setInitLLA(Eigen::Vector3d & lla, PositionRange& range);

private:
    std::unique_ptr<GnssImuMixer>  m_gnssImuMixer;
    std::unique_ptr<ImuProcessor> m_imuProcessor;
    std::unique_ptr<GpsProcessor> m_gpsProcessor;

    bool m_initialized;
    bool m_located;
    bool m_withLLA;
    Eigen::Vector3d m_initLLA; // The initial reference gps point.
    State m_state;
};

}  // namespace rtk_fusion

#endif  // FLSLAM_SRC_MAPPING_RTKFUSION_RTK_FUSION_NODE_H