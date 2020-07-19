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
    float sum_dt_;

    vector<double> dt_buf_;
    vector<Vector3f> accl_buf_;
    vector<Vector3f> gyro_buf_;

    Vector3f accl_bias_, gyro_bias_;

    Vector3f accl_0_, gyro_0_;
    Vector3f accl_1_, gyro_1_;    

    Vector3f    delta_p_;
    Vector3f    delta_v_;
    Quaternionf delta_q_;

    Vector3f    result_delta_p_;
    Vector3f    result_delta_v_;
    Quaternionf result_delta_q_;

    Vector3f    result_accl_bias_;
    Vector3f    result_gyro_bias_;

    // jaconbian
    Matrix<double, J_NUMS, J_NUMS>   Jacobian_;

    // covariance
    Matrix<double, J_NUMS, J_NUMS>   covariance_;

    // noise
    Matrix<double, J_NOISE, J_NOISE>  noise_;
    

public:

    PreIntegrate() {}
    PreIntegrate(const Vector3f& accl0,      const Vector3f& gyro0, 
                 const Vector3f& bias_accl,  const Vector3f& bias_gyro):
                 accl_0_(accl0),        gyro_0_(gyro0), 
                 accl_bias_(bias_accl), gyro_bias_(bias_gyro), 
                 sum_dt_(0) {
        delta_p_.setZero();
        delta_v_.setZero();
        delta_q_.setIdentity();

        Jacobian_.setIdentity();
        covariance_.setIdentity();

        result_delta_p_.setZero();
        result_delta_v_.setZero();
        result_delta_q_.setIdentity();

        dt_buf_.clear();
        accl_buf_.clear();
        gyro_buf_.clear();

        dt_buf_.push_back(0.0f);
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
    void push_back(double dt, const Vector3f& accl, const Vector3f& gyro) {
        dt_buf_.push_back(dt);
        accl_buf_.push_back(accl);
        gyro_buf_.push_back(gyro);
        integrate(dt, accl, gyro);
    }

    void reintegrate(Vector3f accl_bias, Vector3f gyro_bias) {
        sum_dt_ = 0;

        // reset variables
        Jacobian_.setIdentity();
        covariance_.setIdentity();

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

    void integrate(double dt, const Vector3f& accl, const Vector3f& gyro) {

        midPointIntegrate(dt, accl, gyro);

        delta_p_ = result_delta_p_;
        delta_v_ = result_delta_v_;
        delta_q_ = result_delta_q_;
        delta_q_.normalize();

        accl_bias_ = result_accl_bias_;
        gyro_bias_ = result_gyro_bias_;       

        accl_0_ = accl;
        gyro_0_ = gyro;

        sum_dt_ += dt;
    }

    void midPointIntegrate(float dt, const Vector3f& accl, const Vector3f& gyro) {
        const float dt2 = dt*dt;
        
        Vector3f gyro_mid    = (gyro + gyro_0_)/2 - gyro_bias_;
        Vector3f gyro_mid_dt = gyro_mid*dt;

        Quaternionf q_k0 = delta_q_;
        Quaternionf q_k1 = q_k0 * vec2quat<float>(gyro_mid_dt);
        q_k1.normalize();

        Vector3f accl_b0 = accl_0_ - accl_bias_;
        Vector3f accl_b1 = accl    - accl_bias_;

        Vector3f accl_k0 = q_k0*(accl_b0);
        Vector3f accl_k1 = q_k1*(accl_b1);
        
        {   // preintegrate
            Matrix<float, J_NUMS, J_NUMS> F;
            F.setIdentity();

            Matrix<float, J_NUMS, J_NOISE> V;
            V.setZero();

            Matrix3f I   = Matrix3f::Identity();

            Matrix3f R_0 = q_k0.toRotationMatrix();
            Matrix3f R_1 = q_k1.toRotationMatrix();

            Matrix3f accl_b0_x = symmetricMatrix(accl_b0);  // [acc0-bias_a]_x
            Matrix3f accl_b1_x = symmetricMatrix(accl_b1);  // [acc1-bias_a]_x

            Matrix3f gyro_mid_x = symmetricMatrix(gyro_mid);  // [[w0+w1]/2-bias_w]_x

            Matrix3f item_accl_k0 = -R_0*accl_b0_x;
            Matrix3f item_accl_k1 = -R_1*accl_b1_x*(I-gyro_mid_x*dt);

            //!< step F
            F.block<3, 3>(J_OP, J_OR)  = 0.25*(item_accl_k0 + item_accl_k1)*dt2;
            F.block<3, 3>(J_OP, J_OV)  = I*dt;
            F.block<3, 3>(J_OP, J_OBA) = -0.25*(R_0 + R_1)*dt2;
            F.block<3, 3>(J_OP, J_OBW) = 0.25*(R_1*accl_b1_x*dt2*dt);

            F.block<3, 3>(J_OR, J_OR)  = I - gyro_mid_x*dt;
            F.block<3, 3>(J_OR, J_OBW) = I*(-dt);

            F.block<3, 3>(J_OV, J_OR)  = 0.5*(item_accl_k0 + item_accl_k1)*dt;
            F.block<3, 3>(J_OV, J_OBA) = -0.5*(R_0+R_1)*dt;
            F.block<3, 3>(J_OV, J_OBW) = 0.5*(R_1*accl_b1_x*dt*dt);

            //!< step V
            Matrix3f R_1_A_1_x = -R_1*accl_b1_x;
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

            MatrixXd dF = F.cast<double>();
            MatrixXd dV = V.cast<double>();

            // update jacobian
            Jacobian_   = dF*Jacobian_;
            
            // update covariance
            covariance_ = dF*covariance_*dF.transpose() + dV*noise_*dV.transpose();
        }

        Vector3f mid_accl = (accl_k0+accl_k1)/2;

        result_delta_p_ = delta_p_ + delta_v_*dt + mid_accl*dt*dt/2;
        result_delta_v_ = delta_v_ + mid_accl*dt;
        result_delta_q_ = q_k1;

        result_accl_bias_ = accl_bias_;
        result_gyro_bias_ = gyro_bias_;
    }

    Matrix<double, 15, 1> 
    evaluate(const Quaterniond& Rwi, const Vector3d& pwi, const Vector3d& vwi, const Vector3d& bai, const Vector3d& bgi, 
             const Quaterniond& Rwj, const Vector3d& pwj, const Vector3d& vwj, const Vector3d& baj, const Vector3d& bgj) {
                 
        const double dt = sum_dt_;
        Vector3d G = Gw.cast<double>();
        Matrix<double, 15, 1> residual;
        
        Matrix3d J_p_ba = Jacobian_.block<3, 3>(J_OP, J_OBA).cast<double>();
        Matrix3d J_p_bg = Jacobian_.block<3, 3>(J_OP, J_OBW).cast<double>();
        Matrix3d J_v_ba = Jacobian_.block<3, 3>(J_OV, J_OBA).cast<double>();
        Matrix3d J_v_bg = Jacobian_.block<3, 3>(J_OV, J_OBW).cast<double>();
        Matrix3d J_q_bg = Jacobian_.block<3, 3>(J_OR, J_OBW).cast<double>();

        Vector3d dba = bai - accl_bias_.cast<double>();
        Vector3d dbg = bgi - gyro_bias_.cast<double>();

        Vector3d correct_dp    = delta_p_.cast<double>() + J_p_ba*dba + J_p_bg*dbg;
        Vector3d correct_dv    = delta_v_.cast<double>() + J_v_ba*dba + J_v_bg*dbg;
        Quaterniond correct_dq = delta_q_.cast<double>() * vec2quat<double>(J_q_bg*dbg);

        residual.segment<3>(J_OP) = Rwi.inverse()*(pwj - pwi - vwi*dt + 0.5*G*dt*dt) - correct_dp;
        residual.segment<3>(J_OR) = 2*(correct_dq.inverse()*Rwi.inverse()*Rwj).vec();
        residual.segment<3>(J_OV) = Rwi.inverse()*(vwj - vwi + G*dt) - correct_dv;
        residual.segment<3>(J_OBA) = baj - bai;
        residual.segment<3>(J_OBW) = bgj - bgi;
        
        return residual;
    }
};
