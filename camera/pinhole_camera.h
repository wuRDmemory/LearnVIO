#pragma once

#include <memory>
#include "camera_base.h"

class PinholeCamera : public CameraBase {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    float fx_, fy_, cx_, cy_;
    float k1_, k2_, p1_, p2_;

        
};

typedef shared_ptr<PinholeCamera> PinholeCameraPtr;
