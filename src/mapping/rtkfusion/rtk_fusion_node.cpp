
#include <glog/logging.h>

#include "utils.h"
#include "rtk_fusion_node.h"

using namespace rtk_fusion;

RtkFusionNode::RtkFusionNode(double acc_noise, double gyro_noise, double acc_bias_noise, double gyro_bias_noise, const Eigen::Vector3d& imu_to_gps) 
    : m_initialized(false), m_withLLA(false), m_located(false){
    m_state = {};
    m_initLLA = {};
    m_gnssImuMixer = std::make_unique<GnssImuMixer>(imu_to_gps);
    m_imuProcessor = std::make_unique<ImuProcessor>(acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise, Eigen::Vector3d(0., 0., -9.81007));
    m_gpsProcessor = std::make_unique<GpsProcessor>(imu_to_gps);
}

bool RtkFusionNode::HandleImuMessage(const sensor_msgs::Imu::ConstPtr& msg, State* fused_state) {
    rtk_fusion::ImuDataPtr imu_data_ptr = std::make_shared<rtk_fusion::ImuData>();
    imu_data_ptr->timestamp = msg->header.stamp.toSec();
    imu_data_ptr->acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    imu_data_ptr->gyro << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    if (!m_initialized) {
        m_gnssImuMixer->AddImuData(imu_data_ptr);
        return false;
    }
    
    // Predict.
    m_imuProcessor->Predict(m_state.imu_data_ptr, imu_data_ptr, &m_state);

    // Convert ENU state to lla.
    ConvertENUToLLA(m_initLLA, m_state.G_p_I, &(m_state.lla));
    // check the postion range
    if (m_state.G_p_I[0] > m_state.range.max_x) {
        m_state.range.max_x = m_state.G_p_I[0];
    }
    if (m_state.G_p_I[0] < m_state.range.min_x) {
        m_state.range.min_x = m_state.G_p_I[0];
    }
    if (m_state.G_p_I[1] > m_state.range.max_y) {
        m_state.range.max_y = m_state.G_p_I[1];
    }
    if (m_state.G_p_I[1] < m_state.range.min_y) {
        m_state.range.min_y = m_state.G_p_I[1];
    }
    *fused_state = m_state;
    return true;
}
// static int gnss_count = 0;
bool RtkFusionNode::HandleNavSatFixMessage(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    rtk_fusion::GpsPositionDataPtr gps_data_ptr = std::make_shared<rtk_fusion::GpsPositionData>();
    gps_data_ptr->timestamp = msg->header.stamp.toSec();
    gps_data_ptr->lla << msg->latitude, msg->longitude, msg->altitude;
    // if (gnss_count++ ==10) {
    //     gnss_count = 0;
    //     LOG(INFO) << std::fixed << std::setprecision(15) << "gps_data_lla: " << msg->latitude << " - " << msg->longitude << " - " << msg->altitude;
    // }

    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(msg->position_covariance.data());

    if (!m_initialized) {
        if (!m_gnssImuMixer->AddGpsPositionData(gps_data_ptr, &m_state)) {
            return false;
        }
        m_gnssImuMixer->m_lastGpsData = gps_data_ptr;
        // Initialize the initial gps point used to convert lla to ENU.
        if (!m_withLLA) {
            m_initLLA = gps_data_ptr->lla;
        }

        m_initialized = true;
        LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
        return true;
    }

    if (!m_located) {
        if (m_gnssImuMixer->ComputeMotionPose(gps_data_ptr, &m_state)) {
            m_located = true;
        }
        // LOG(INFO) << "fusion node init lla:" << std::fixed << std::setprecision(15) << m_initLLA[0] << " - " << m_initLLA[1] << " - " << m_initLLA[2];
        m_gnssImuMixer->m_lastGpsData = gps_data_ptr;
        m_gpsProcessor->m_lastGpsData = gps_data_ptr;
    }

    // Update.
    m_gpsProcessor->UpdateStateByGpsPosition(m_initLLA, gps_data_ptr, &m_state);
    return true;
}

Eigen::Vector3d RtkFusionNode::getInitLLA() {
    return m_initLLA;
}

void RtkFusionNode::setInitLLA(Eigen::Vector3d & lla, PositionRange& range) {
    m_initLLA = lla;
    m_state.range = range;
    m_withLLA = true;
}
