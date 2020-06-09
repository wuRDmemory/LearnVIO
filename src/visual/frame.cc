#include "../../include/visual/frame.h"
#include "../../include/visual/feature.h"

Frame::Frame() {
    Rwb_.setIdentity();
    Pwb_.setZero();
    Vw_.setZero();

    Biasa_.setZero();
    Biasw_.setZero();

    vis_ftr_.clear();
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
