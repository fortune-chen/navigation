#ifndef FLSLAM_SRC_MAPPING_RTKFUSION_GPS_PROCESSOR_H
#define FLSLAM_SRC_MAPPING_RTKFUSION_GPS_PROCESSOR_H

#include <Eigen/Dense>

#include "base_type.h"

namespace rtk_fusion {

class GpsProcessor {
public:
    GpsProcessor(const Eigen::Vector3d& imu_to_gps);

    bool UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla, const GpsPositionDataPtr gps_data_ptr, State* state);

    void AdjustPoseByGps(const GpsPositionDataPtr gps_data_ptr, State* state);

    GpsPositionDataPtr m_lastGpsData;

private:    
    void ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                    const GpsPositionDataPtr gps_data, 
                                    const State& state,
                                    Eigen::Matrix<double, 3, 15>* jacobian,
                                    Eigen::Vector3d* residual);

    const Eigen::Vector3d m_imuToGpsTransform;
};

}  // namespace rtk_fusion

#endif  // FLSLAM_SRC_MAPPING_RTKFUSION_GPS_PROCESSOR_H