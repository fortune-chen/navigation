#ifndef FLSLAM_SRC_MAPPING_RTKFUSION_GNS_IMU_MIXER_H
#define FLSLAM_SRC_MAPPING_RTKFUSION_GNS_IMU_MIXER_H

#include <deque>
#include "base_type.h"

namespace rtk_fusion {

constexpr int kImuDataBufferLength = 100;
constexpr int kAccStdLimit         = 3.;

class GnssImuMixer {
public:
    GnssImuMixer(const Eigen::Vector3d& imu_to_gps);
    
    void AddImuData(const ImuDataPtr imu_data_ptr);

    bool AddGpsPositionData(const GpsPositionDataPtr gps_data_ptr, State* state);

    bool ComputeMotionPose(const GpsPositionDataPtr gps_data_ptr, State* state);

    GpsPositionDataPtr m_lastGpsData;

private:
    bool ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I);

    Eigen::Vector3d m_imuToGpsTransform;
    std::deque<ImuDataPtr> m_imuBuffer;

};

}  // namespace rtk_fusion

#endif  // FLSLAM_SRC_MAPPING_RTKFUSION_GNS_IMU_MIXER_H