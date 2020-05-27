#pragma once

#include <memory>
#include "camera_base.h"

class PinholeCamera: virtual public CameraBase {
    protected:
        float fx_, fy_, cx_, cy_;
        float d0_, d1_, d2_, d3_;

        Eigen::Matrix3f K_, invK_;
        Eigen::Matrix<float, 1, 4> D_;

        bool mUseDistort;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        PinholeCamera(int width, int height, float fx, float fy, float cx, float cy, 
                      float d0=0, float d1=0, float d2=0, float d3=0, float d4=0);
        virtual ~PinholeCamera();

        virtual Eigen::Vector3f cam2world(const float& x, const float& y) const;

        virtual Eigen::Vector3f cam2world(const Eigen::Vector2f& px) const;
        
        virtual Eigen::Vector2f world2cam(const Eigen::Vector3f& xyz_c) const;

        virtual Eigen::Vector2f world2cam(const Eigen::Vector2f& uv) const;

        const Eigen::Vector2f focal_length() const { return Vector2f(fx_, fy_);}

        virtual float errorMultiplier2() const { return fabs(fx_);}

        virtual float errorMultiplier() const { return fabs(4.0*fx_*fy_); }

        const Matrix3f& K()    const { return K_;    }

        const Matrix3f& invK() const { return invK_; }

        int initUnistortionMap();

        int undistortImage(const cv::Mat& raw, cv::Mat& rectified);
};;

typedef shared_ptr<PinholeCamera> PinholeCameraPtr;
