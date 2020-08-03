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
        float k1, float k2, float p1, float p2, float k3):
        CameraModel(width, height, name, CameraModel::PINHOLE) {
    fx_ = fx; fy_ = fy;
    cx_ = cx; cy_ = cy;
    k1_ = k1; k2_ = k2;
    p1_ = p1; p2_ = p2;
    k3_ = k3;

    use_distort = k1 >= 1e-5;

    K_ << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    invK_ = K_.inverse();
    D_ << k1_, k2_, p1_, p2, k3;

    cvK_ = (cv::Mat_<float>(3,3) <<  fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cvinvK_ = cvK_.inv();

    cvD_ = (cv::Mat_<float>(1,5) << k1, k2, p1, p2, k3);
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
        Vector2f uv(x, y), d_uv;
        
        d_uv = distortion(uv);
        // Approximate value
        uv(0) -= d_uv(0);
        uv(1) -= d_uv(1);

        for (int i = 1; i < 8; ++i)
        {
            d_uv   = distortion(uv);
            uv(0) -= d_uv(0);
            uv(1) -= d_uv(1);
        }

        xyz[0] = uv(0);
        xyz[1] = uv(1);
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
    } 
    else {
        double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
        x = uv[0];
        y = uv[1];
        r2 = x*x + y*y;
        r4 = r2*r2;
        r6 = r4*r2;
        a1 = 2*x*y;
        a2 = r2 + 2*x*x;
        a3 = r2 + 2*y*y;
        cdist = 1 + k1_*r2 + k2_*r4 + k3_*r6;
        xd = x*cdist + p1_*a1 + p2_*a2;
        yd = y*cdist + p1_*a3 + p2_*a1;
        px[0] = xd*fx_ + cx_;
        px[1] = yd*fy_ + cy_;
    }

    return px;
}


Vector2f PinholeCamera::distortion(const Eigen::Vector2f& p_u) const {
    Vector2f d_u;
    float mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0)*p_u(0);
    my2_u = p_u(1)*p_u(1);
    mxy_u = p_u(0)*p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1_*rho2_u + k2_*rho2_u*rho2_u;
    d_u << p_u(0)*rad_dist_u + 2.0*p1_*mxy_u + p2_*(rho2_u + 2.0*mx2_u),
           p_u(1)*rad_dist_u + 2.0*p2_*mxy_u + p1_*(rho2_u + 2.0*my2_u);

    return d_u;
}

Vector2f PinholeCamera::world2cam(const Eigen::Vector3f& xyz_c) const { 
    Vector2f uv = project2d(xyz_c);
    return world2cam(uv);
}

