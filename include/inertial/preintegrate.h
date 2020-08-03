#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "../../include/util/utils.h"
#include "../../include/util/config.h"
#include "../../include/util/log.h"

using namespace std;
using namespace cv;
using namespace Eigen;

#define J_NUMS   15
#define J_NOISE  18

#define J_OP     0
#define J_OR     3
#define J_OV     6
#define J_OBA    9
#define J_OBW    12

#define V_ONA0   0
#define V_ONW0   3
#define V_ONA1   6
#define V_ONW1   9
#define V_ONBA   12
#define V_ONBW   15

class PreIntegrate {
public:
    double sum_dt_;

    vector<double>   dt_buf_;
    vector<Vector3d> accl_buf_;
    vector<Vector3d> gyro_buf_;

    Vector3d accl_bias_, gyro_bias_;

    Vector3d accl_0_, gyro_0_;
    Vector3d accl_1_, gyro_1_;    

    Vector3d    delta_p_;
    Vector3d    delta_v_;
    Quaterniond delta_q_;

    Vector3d    result_accl_bias_;
    Vector3d    result_gyro_bias_;

    // jaconbian
    Matrix<double, J_NUMS, J_NUMS>   Jacobian_;

    // covariance
    Matrix<double, J_NUMS, J_NUMS>   covariance_;

    // noise
    Matrix<double, J_NOISE, J_NOISE>  noise_;
    

public:

    PreIntegrate() {}
    PreIntegrate(const Vector3d& accl0,      const Vector3d& gyro0, 
                 const Vector3d& bias_accl,  const Vector3d& bias_gyro):
                 accl_0_(accl0),        gyro_0_(gyro0), 
                 accl_bias_(bias_accl), gyro_bias_(bias_gyro), 
                 sum_dt_(0) {
        delta_p_.setZero();
        delta_v_.setZero();
        delta_q_.setIdentity();

        Jacobian_.setIdentity();
        covariance_.setZero();

        dt_buf_.clear();
        accl_buf_.clear();
        gyro_buf_.clear();

        dt_buf_.push_back(0.0);
        accl_buf_.push_back(accl0);
        gyro_buf_.push_back(gyro0);

        Matrix3d I   = Matrix3d::Identity();

        noise_.setZero();
        noise_.block<3, 3>(V_ONA0, V_ONA0) = (ACCL_N*ACCL_N)*I;
        noise_.block<3, 3>(V_ONW0, V_ONW0) = (GYRO_N*GYRO_N)*I;
        noise_.block<3, 3>(V_ONA1, V_ONA1) = (ACCL_N*ACCL_N)*I;
        noise_.block<3, 3>(V_ONW1, V_ONW1) = (GYRO_N*GYRO_N)*I;
        noise_.block<3, 3>(V_ONBA, V_ONBA) = (ACCL_BIAS_N*ACCL_BIAS_N)*I;
        noise_.block<3, 3>(V_ONBW, V_ONBW) = (GYRO_BIAS_N*GYRO_BIAS_N)*I;
    }

    ~PreIntegrate() {
        ;
    }

    // integration
    void push_back(double dt, const Vector3d& accl, const Vector3d& gyro) {
        dt_buf_.push_back(dt);
        accl_buf_.push_back(accl);
        gyro_buf_.push_back(gyro);
        integrate(dt, accl, gyro);
    }

    void reintegrate(const Vector3d &accl_bias, const Vector3d &gyro_bias) {
        sum_dt_ = 0;

        // reset variables
        Jacobian_.setIdentity();
        covariance_.setZero();

        delta_p_.setZero();
        delta_v_.setZero();
        delta_q_.setIdentity();

        accl_bias_ = accl_bias;
        gyro_bias_ = gyro_bias;

        accl_0_ = accl_buf_[0];
        gyro_0_ = gyro_buf_[0];

        for (int i = 1; i < accl_buf_.size(); i++) {
            integrate(dt_buf_[i], accl_buf_[i], gyro_buf_[i]);
        }
    }

    void integrate(double dt, const Vector3d& accl, const Vector3d& gyro) {
        Quaterniond delta_q1_;
        Vector3d    delta_p1_;
        Vector3d    delta_v1_;
        Vector3d    accl_bias1_;
        Vector3d    gyro_bias1_;

        accl_1_ = accl;
        gyro_1_ = gyro;

        midPointIntegrate(dt, \
                          delta_q_,  delta_p_,  delta_v_,  accl_bias_,  gyro_bias_, \
                          delta_q1_, delta_p1_, delta_v1_, accl_bias1_, gyro_bias1_);

        delta_p_ = delta_p1_;
        delta_v_ = delta_v1_;
        delta_q_ = delta_q1_.normalized();

        accl_bias_ = accl_bias1_;
        gyro_bias_ = gyro_bias1_;

        accl_0_ = accl_1_;
        gyro_0_ = gyro_1_;

        sum_dt_ += dt;
    }

    void midPointIntegrate(double dt, 
                           const Quaterniond& delta_q0, const Vector3d& delta_p0, const Vector3d& delta_v0, 
                           const Vector3d&  accl_bias0, const Vector3d& gyro_bias0, 
                           Quaterniond& delta_q1, Vector3d& delta_p1, Vector3d& delta_v1, 
                           Vector3d&  accl_bias1, Vector3d& gyro_bias1) {
        const double dt2 = dt*dt;
        
        Vector3d gyro_mid    = (gyro_1_ + gyro_0_)/2 - gyro_bias0;
        Vector3d gyro_mid_dt = gyro_mid*dt;

        delta_q1 = delta_q0 * vec2quat(gyro_mid_dt);
        delta_q1.normalize();

        Vector3d accl_b0 = accl_0_ - accl_bias_;
        Vector3d accl_b1 = accl_1_ - accl_bias_;

        Vector3d accl_k0 = delta_q0*accl_b0;
        Vector3d accl_k1 = delta_q1*accl_b1;
        
        Vector3d mid_accl = (accl_k0+accl_k1)/2;

        delta_p1 = delta_p0 + delta_v0*dt + mid_accl*dt2/2;
        delta_v1 = delta_v0 + mid_accl*dt;

        accl_bias1 = accl_bias0;
        gyro_bias1 = gyro_bias0;

        {   // preintegrate
            Matrix<double, J_NUMS, J_NUMS> F;
            F.setIdentity();

            Matrix<double, J_NUMS, J_NOISE> V;
            V.setZero();

            Matrix3d I   = Matrix3d::Identity();

            Matrix3d R_0 = delta_q0.toRotationMatrix();
            Matrix3d R_1 = delta_q1.toRotationMatrix();

            Matrix3d accl_b0_x = symmetricMatrix(accl_b0);  // [acc0-bias_a]_x
            Matrix3d accl_b1_x = symmetricMatrix(accl_b1);  // [acc1-bias_a]_x

            Matrix3d gyro_mid_x = symmetricMatrix(gyro_mid);  // [[w0+w1]/2-bias_w]_x

            Matrix3d item_accl_k0 = -R_0*accl_b0_x;
            Matrix3d item_accl_k1 = -R_1*accl_b1_x*(I-gyro_mid_x*dt);

            //!< step F
            F.block<3, 3>(J_OP, J_OR)  = 0.25*(item_accl_k0 + item_accl_k1)*dt2;
            F.block<3, 3>(J_OP, J_OV)  = I*dt;
            F.block<3, 3>(J_OP, J_OBA) = -0.25*(R_0 + R_1)*dt2;
            F.block<3, 3>(J_OP, J_OBW) = 0.25*(-R_1*accl_b1_x*dt2*-dt);

            F.block<3, 3>(J_OR, J_OR)  = I - gyro_mid_x*dt;
            F.block<3, 3>(J_OR, J_OBW) = I*(-dt);

            F.block<3, 3>(J_OV, J_OR)  = 0.5*(item_accl_k0 + item_accl_k1)*dt;
            F.block<3, 3>(J_OV, J_OBA) = -0.5*(R_0+R_1)*dt;
            F.block<3, 3>(J_OV, J_OBW) = 0.5*(R_1*accl_b1_x*dt*dt);

            //!< step V
            Matrix3d R_1_A_1_x = -R_1*accl_b1_x;
            V.block<3, 3>(J_OP, V_ONA0) = 0.25*R_0*dt2;
            V.block<3, 3>(J_OP, V_ONW0) = 0.25*R_1_A_1_x*dt2*0.5*dt;
            V.block<3, 3>(J_OP, V_ONA1) = 0.25*R_1*dt2;
            V.block<3, 3>(J_OP, V_ONW1) = 0.25*R_1_A_1_x*dt2*0.5*dt;

            V.block<3, 3>(J_OR, V_ONW0) = 0.5*I*dt;
            V.block<3, 3>(J_OR, V_ONW1) = 0.5*I*dt;

            V.block<3, 3>(J_OV, V_ONA0) = 0.5*R_0*dt;
            V.block<3, 3>(J_OV, V_ONW0) = 0.5*R_1_A_1_x*dt*0.5*dt;
            V.block<3, 3>(J_OV, V_ONA1) = 0.5*R_1*dt;
            V.block<3, 3>(J_OV, V_ONW1) = 0.5*R_1_A_1_x*dt*0.5*dt;

            V.block<3, 3>(J_OBA, V_ONBA) = I*dt;
            V.block<3, 3>(J_OBW, V_ONBW) = I*dt;
            // cout << "A" << endl;
            // cout << V << endl << endl;

            // update jacobian
            Jacobian_   = F*Jacobian_;
            
            // update covariance
            covariance_ = F*covariance_*F.transpose() + V*noise_*V.transpose();
        }
    }

    Matrix<double, 15, 1> 
    evaluate(const Quaterniond& Rwi, const Vector3d& pwi, const Vector3d& vwi, const Vector3d& bai, const Vector3d& bgi, 
             const Quaterniond& Rwj, const Vector3d& pwj, const Vector3d& vwj, const Vector3d& baj, const Vector3d& bgj) {
                 
        const double dt = sum_dt_;
        Vector3d G = Gw;
        Matrix<double, 15, 1> residual;
        
        Matrix3d J_p_ba = Jacobian_.block<3, 3>(J_OP, J_OBA);
        Matrix3d J_p_bg = Jacobian_.block<3, 3>(J_OP, J_OBW);
        Matrix3d J_v_ba = Jacobian_.block<3, 3>(J_OV, J_OBA);
        Matrix3d J_v_bg = Jacobian_.block<3, 3>(J_OV, J_OBW);
        Matrix3d J_q_bg = Jacobian_.block<3, 3>(J_OR, J_OBW);

        Vector3d dba = bai - accl_bias_;
        Vector3d dbg = bgi - gyro_bias_;

        Vector3d correct_dp    = delta_p_ + J_p_ba*dba + J_p_bg*dbg;
        Vector3d correct_dv    = delta_v_ + J_v_ba*dba + J_v_bg*dbg;
        Quaterniond correct_dq = delta_q_ * vec2quat<double>(J_q_bg*dbg);

        residual.block<3, 1>(J_OP,  0) = Rwi.inverse()*(pwj - pwi - vwi*dt + 0.5*G*dt*dt) - correct_dp;
        residual.block<3, 1>(J_OR,  0) = 2*(correct_dq.inverse()*Rwi.inverse()*Rwj).vec();
        residual.block<3, 1>(J_OV,  0) = Rwi.inverse()*(vwj - vwi + G*dt) - correct_dv;
        residual.block<3, 1>(J_OBA, 0) = baj - bai;
        residual.block<3, 1>(J_OBW, 0) = bgj - bgi;
        
        return residual;
    }
};
