/*
 * @Author: your name
 * @Date: 2020-05-27 21:33:48
 * @LastEditTime: 2020-05-27 22:01:52
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /LearnVIO/camera/pinhole_camera.h
 */ 
#pragma once

#include <memory>
#include "camera_base.h"

class PinholeCamera: virtual public CameraModel {
    protected:
        float fx_, fy_, cx_, cy_;
        float k1_, k2_, p1_, p2_, k3_;

        Eigen::Matrix3f K_, invK_;
        Eigen::Matrix<float, 1, 5> D_;

        cv::Mat cvK_;
        cv::Mat cvinvK_;
        cv::Mat cvD_;

        bool use_distort;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        PinholeCamera(int width, int height, string name, float fx, float fy, float cx, float cy, 
                      float k1=0, float k2=0, float p1=0, float p2=0, float k3=0);
        virtual ~PinholeCamera();

        virtual Eigen::Vector3f cam2world(const float& x, const float& y) const;

        virtual Eigen::Vector3f cam2world(const Eigen::Vector2f& px) const;
        
        virtual Eigen::Vector2f world2cam(const Eigen::Vector3f& xyz_c) const;

        virtual Eigen::Vector2f world2cam(const Eigen::Vector2f& uv) const;

        Vector2f distortion(const Eigen::Vector2f& p_u) const;

        const Eigen::Vector2f focal_length() const { return Vector2f(fx_, fy_);}

        virtual float errorMultiplier2() const { return fabs(fx_);}

        virtual float errorMultiplier() const { return fabs(4.0*fx_*fy_); }

        const Matrix3f& K()    const { return K_;    }

        const Matrix3f& invK() const { return invK_; }

        const cv::Mat& cvK()    const { return cvK_;    }

        const cv::Mat& cvinvK() const { return cvinvK_; }

        int initUnistortionMap();

        int undistortImage(const cv::Mat& raw, cv::Mat& rectified);
};
