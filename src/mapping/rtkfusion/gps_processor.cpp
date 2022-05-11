#include "utils.h"
#include <glog/logging.h>
#include "gps_processor.h"

using namespace rtk_fusion;

void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state) {
    state->G_p_I     += delta_x.block<3, 1>(0, 0);
    state->G_v_I     += delta_x.block<3, 1>(3, 0);
    state->acc_bias  += delta_x.block<3, 1>(9, 0);
    state->gyro_bias += delta_x.block<3, 1>(12, 0);

    if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
        state->G_R_I *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
    }
}

GpsProcessor::GpsProcessor(const Eigen::Vector3d& imu_to_gps) : m_imuToGpsTransform(imu_to_gps) { }

bool GpsProcessor::UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla, const GpsPositionDataPtr gps_data_ptr, State* state) {
    Eigen::Matrix<double, 3, 15> H;
    Eigen::Vector3d residual;
    ComputeJacobianAndResidual(init_lla, gps_data_ptr, *state, &H, &residual);
    const Eigen::Matrix3d& V = gps_data_ptr->cov;

    // EKF.
    const Eigen::MatrixXd& P = state->cov;
    const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
    const Eigen::VectorXd delta_x = K * residual;
    // Add delta_x to state.
    AddDeltaToState(delta_x, state);
    AdjustPoseByGps(gps_data_ptr, state);

    // Covarance.
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    // mike comment
    //state->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
}

void GpsProcessor::AdjustPoseByGps(const GpsPositionDataPtr gps_data_ptr, State* state) {
    Eigen::Vector3d deltaLLA = gps_data_ptr->lla - m_lastGpsData->lla;
    deltaLLA[2] = 0.;
    if (deltaLLA.norm() > 2e-7) {
        auto angle = atan2(deltaLLA[1], deltaLLA[0]);
        Eigen::AngleAxisd rotation_vector(angle, Eigen::Vector3d(0,0,1));

        Eigen::Matrix3d I_R_G = rotation_vector.matrix();
        Eigen::Matrix3d G_R_I = I_R_G.transpose();
        Eigen::Vector3d delta_orientation = G_R_I.matrix().eulerAngles(2,1,0) - state->G_R_I.matrix().eulerAngles(2,1,0);

        if (abs(delta_orientation[0]) > 0.25 && abs(delta_orientation[0]) < 1) {
            LOG(INFO) << "orientation adjust: " << delta_orientation[0] << "z angle: " << state->G_R_I.matrix().eulerAngles(2,1,0)[0];
            state->G_R_I = (G_R_I + state->G_R_I) / 2;
        }
    }

    m_lastGpsData = gps_data_ptr;
}

void GpsProcessor::ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                              const GpsPositionDataPtr gps_data, 
                                              const State& state,
                                              Eigen::Matrix<double, 3, 15>* jacobian,
                                              Eigen::Vector3d* residual) {
    const Eigen::Vector3d& G_p_I   = state.G_p_I;
    const Eigen::Matrix3d& G_R_I   = state.G_R_I;

    // Convert wgs84 to ENU frame.
    Eigen::Vector3d G_p_Gps;
    ConvertLLAToENU(init_lla, gps_data->lla, &G_p_Gps);

    // Compute residual.
    *residual = G_p_Gps - (G_p_I + G_R_I * m_imuToGpsTransform);
    // Compute jacobian.
    jacobian->setZero();
    jacobian->block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    jacobian->block<3, 3>(0, 6)  = - G_R_I * GetSkewMatrix(m_imuToGpsTransform);
}