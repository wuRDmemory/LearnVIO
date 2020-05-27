/*
 * @Author: your name
 * @Date: 2020-05-27 21:35:10
 * @LastEditTime: 2020-05-27 22:26:32
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /LearnVIO/camera/pinhole_camera.cc
 */ 
#include "pinhole_camera.h"

PinholeCamera::PinholeCamera(int width, int height, string name, float fx, float fy, float cx, float cy, 
        float d0, float d1, float d2, float d3, float d4):
        CameraModel(width, height, name, CameraModel::PINHOLE) {
    fx_ = fx; fy_ = fy;
    cx_ = cx; cy_ = cy;
    d0_ = d0; d1_ = d1;
    d2_ = d2; d3_ = d3;
    d4_ = d4;

    use_distort = d0 >= 1e-5;

    K_ << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    invK_ = K_.inverse();
    D_ << d0, d1, d2, d3, d4;

    cvK_ = (cv::Mat_<float>(3,3) <<  fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cvinvK_ = cvK_.inv();

    cvD_ = (cv::Mat_<float>(1,5) << d0, d1, d2, d3, d4);
}

PinholeCamera::~PinholeCamera() {
    ; // do nothing here
}

Eigen::Vector3f PinholeCamera::cam2world(const float& x, const float& y) const { 
    Eigen::Vector3f xyz;
    if(!use_distort) {
        xyz[0] = (x - cx_)/fx_;
        xyz[1] = (y - cy_)/fy_;
        xyz[2] = 1.0;
    } else {
        cv::Point2f uv(x, y), px;
        const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
        cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
        cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
        xyz[0] = px.x;
        xyz[1] = px.y;
        xyz[2] = 1.0;
    }
    return xyz;
}

Vector3f PinholeCamera::cam2world (const Vector2f& uv) const {
    return cam2world(uv[0], uv[1]);
}

Vector2f PinholeCamera::world2cam(const Vector2f& uv) const {
    Vector2f px;
    if(!use_distort) {
        px[0] = fx_*uv[0] + cx_;
        px[1] = fy_*uv[1] + cy_;
    } else {
        double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
        x = uv[0];
        y = uv[1];
        r2 = x*x + y*y;
        r4 = r2*r2;
        r6 = r4*r2;
        a1 = 2*x*y;
        a2 = r2 + 2*x*x;
        a3 = r2 + 2*y*y;
        cdist = 1 + d0_*r2 + d1_*r4 + d4_*r6;
        xd = x*cdist + d2_*a1 + d3_*a2;
        yd = y*cdist + d2_*a3 + d3_*a1;
        px[0] = xd*fx_ + cx_;
        px[1] = yd*fy_ + cy_;
    }
    return px;
}

Vector2f PinholeCamera::world2cam(const Eigen::Vector3f& xyz_c) const { 
    Vector2f uv = project2d(xyz_c);
    return world2cam(uv);
}

