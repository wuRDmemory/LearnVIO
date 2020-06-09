#include "../../include/visual/frame.h"
#include "../../include/visual/feature.h"

Frame::Frame(int id) {
    id_ = id;

    integration_ = NULL;

    Rwb_.setIdentity();
    Pwb_.setZero();
    Vw_.setZero();

    Biasa_.setZero();
    Biasw_.setZero();

    vis_ftr_.clear();
}

Frame::Frame(const Frame& frame) {
    Rwb_ = frame.Rwb_;
    Pwb_ = frame.Pwb_;
    Vw_  = frame.Vw_;

    Biasa_ = frame.Biasa_;
    Biasw_ = frame.Biasa_;

    id_ = frame.id_;

    vis_ftr_ = frame.vis_ftr_;

    memcpy(integration_, frame.integration_, sizeof(PreIntegrate));
}

bool Frame::addFeature(Feature* feature) {
    vis_ftr_.push_back(feature);
    return true;
}

bool Frame::removeFeature(Feature* feature) {
    for (auto iter = vis_ftr_.begin(); iter != vis_ftr_.end(); iter++) {
        if (*iter == feature) {
            vis_ftr_.erase(iter);
            return true;
        }
    }
    return false;
}


bool Frame::integrate(double dt, const Vector3f& accl, const Vector3f& gyro) {
    return true;
}

