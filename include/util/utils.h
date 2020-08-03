#pragma once

#include <cmath>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace cv;
using namespace Eigen;


template<typename T>
Matrix<T, 3, 3> symmetricMatrix(const Matrix<T, 3, 1>& vec) {
    Matrix<T, 3, 3> ret;

    T x = vec(0, 0);
    T y = vec(1, 0);
    T z = vec(2, 0);

    ret <<  0, -z,  y, \
            z,  0, -x, \
           -y,  x,  0;

    return ret;
}

template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
{
    //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
    //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
    //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
    //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
    return q;
}

// template <typename Derived>
// Eigen::Matrix<typename Derived::Scalar, 4, 4> quatLeftMult(const Eigen::QuaternionBase<Derived> &q)
// {
//     Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
//     Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
//     ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
//     ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + symmetricMatrix<typename Derived::Scalar>(qq.vec());
//     return ans;
// }

// template <typename Derived>
// Eigen::Matrix<typename Derived::Scalar, 4, 4> quatRightMult(const Eigen::QuaternionBase<Derived> &p)
// {
//     Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
//     Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
//     ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
//     ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - symmetricMatrix<typename Derived::Scalar>(pp.vec());
//     return ans;
// }

template<typename T>
Matrix<T, 4, 4> quatLeftMult(const Quaternion<T> &q) {
    Matrix<T, 4, 4, RowMajor> ret;

    ret << q.w(), -q.x(), -q.y(), -q.z(), \
           q.x(),  q.w(), -q.z(),  q.y(), \
           q.y(),  q.z(),  q.w(), -q.x(), \
           q.z(), -q.y(),  q.x(),  q.w();
    
    return ret;
}

template<typename T>
Matrix<T, 4, 4> quatRightMult(const Quaternion<T> &q) {
    Matrix<T, 4, 4, RowMajor> ret;

    ret << q.w(), -q.x(), -q.y(), -q.z(), \
           q.x(),  q.w(),  q.z(), -q.y(), \
           q.y(), -q.z(),  q.w(),  q.x(), \
           q.z(),  q.y(), -q.x(),  q.w();
    
    return ret;
}

template<typename T>
Quaternion<T> vec2quat(Matrix<T, 3, 1> vec) {
    Quaternion<T> ret;

    T x = vec(0, 0);
    T y = vec(1, 0);
    T z = vec(2, 0);

    ret.w() = 1;
    ret.x() = x/2;
    ret.y() = y/2;
    ret.z() = z/2;
    
    return ret;
}

template<typename T> 
Matrix<T, 3, 1> Rnb2ypr(Matrix<T, 3, 3> R) {
    Matrix<T, 3, 1> n = R.col(0);
    Matrix<T, 3, 1> o = R.col(1);
    Matrix<T, 3, 1> a = R.col(2);

    Matrix<T, 3, 1> ypr;
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}

template<typename T>
Matrix<T, 3, 3> ypr2Rnb(Matrix<T, 3, 1> ypr) {

    T y = ypr(0) / 180.0 * M_PI;
    T p = ypr(1) / 180.0 * M_PI;
    T r = ypr(2) / 180.0 * M_PI;

    Eigen::Matrix<T, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
        sin(y), cos(y), 0,
        0, 0, 1;

    Eigen::Matrix<T, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
          0., 1., 0.,
          -sin(p), 0., cos(p);

    Eigen::Matrix<T, 3, 3> Rx;
    Rx << 1., 0., 0.,
        0., cos(r), -sin(r),
        0., sin(r), cos(r);

    return Rz * Ry * Rx;
}

template<typename T>
Matrix<T, 3, 3> gravity2Rnb(Matrix<T, 3, 1> g) {
    Matrix<T, 3, 3> R0;
    Matrix<T, 3, 1> ng1 = g.normalized();
    Matrix<T, 3, 1> ng2{0, 0, 1.0};
    R0 = Quaternion<T>::FromTwoVectors(ng1, ng2).toRotationMatrix();  // w'_R_c0

    T yaw = Rnb2ypr(R0).x();
    R0 = ypr2Rnb(Matrix<T, 3, 1>{-yaw, 0, 0})*R0;  // w_R_c0 = w_R_w' * w'_R_c0
    
    return R0;
}

template<typename T> 
void cvtPoseFromBodyToCamera(const Matrix<T, 3, 3> &Rwb, const Matrix<T, 3, 1> &twb, 
                                 const Matrix<T, 3, 3> &Rbc, const Matrix<T, 3, 1> &tbc, 
                                 Matrix<T, 3, 3> &Rwc, Matrix<T, 3, 1> &twc) {
    // w_R_c = w_R_b * b_R_c
    Rwc = Rwb*Rbc;
    // w_P_c = w_P_b + w_R_b * b_P_c
    twc = twb + Rwb*tbc;
}

template<typename T> 
void cvtPoseFromCameraToBody(const Matrix<T, 3, 3> &Rwc, const Matrix<T, 3, 1> &twc, 
                                 const Matrix<T, 3, 3> &Rbc, const Matrix<T, 3, 1> &tbc, 
                                 Matrix<T, 3, 3> &Rwb, Matrix<T, 3, 1> &twb) {
    // w_R_b = w_R_c * c_R_b
    Rwb = Rwc*Rbc.transpose();
    // w_P_b = w_P_c - w_R_b * b_P_c
    twb = twc - Rwb*tbc;
}
