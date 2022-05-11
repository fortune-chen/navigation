#ifndef FLSLAM_SRC_MAPPING_RTKFUSION_IMU_PROCESSOR_H
#define FLSLAM_SRC_MAPPING_RTKFUSION_IMU_PROCESSOR_H

#include "base_type.h"

namespace rtk_fusion {

class ImuProcessor{
public:
    ImuProcessor(const double acc_noise, const double gyro_noise,
                 const double acc_bias_noise, const double gyro_bias_noise,
                 const Eigen::Vector3d& gravity);

    void Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state);

private:
    const double m_accNoise;
    const double m_gyroNoise;
    const double m_accBiasNoise;
    const double m_gyroBiasNoise;

    const Eigen::Vector3d m_gravity;
};

}  // namespace rtk_fusion

#endif  // FLSLAM_SRC_MAPPING_RTKFUSION_IMU_PROCESSOR_H