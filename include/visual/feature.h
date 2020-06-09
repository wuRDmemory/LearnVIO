#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

#include "../../include/visual/frame.h"

using namespace std;
using namespace cv;
using namespace Eigen;

typedef pair<int, vector<double> >            Elem_Type;
typedef map<int, pair<int, vector<double> > > Image_Type;

class Feature {
public:
    int id_;

    vector<Frame*>    vis_frames_;
    vector<Vector3f>  vis_fs_;
    vector<Vector2f>  vis_uv_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   
    Feature() = delete;
    Feature(int id, Vector3f& f, Vector2f& uv, Frame* frame) {
        id_ = id;
        vis_frames_.push_back(frame);
        vis_fs_.push_back(f);
        vis_uv_.push_back(uv);
    }

    int size() const {
        return vis_frames_.size();
    }

    int getRefFrameId() { 
        return vis_frames_[0]->id_;   
    }

    int getLastFrameId() {
        return vis_frames_.back()->id_;
    }

    bool contains(const Frame* frame) {
        int ref_id = vis_frames_[0]->id_;
        int cur_id = frame->id_;

        if (cur_id < ref_id) return false;

        return (cur_id - ref_id) < (int)vis_frames_.size();
    }

    bool addFrame(const Vector3f& f, const Vector2f& uv, Frame* frame) {
        vis_frames_.push_back(frame);
        vis_fs_.push_back(f);
        vis_uv_.push_back(uv);

        return true;
    }

    bool removeFrame(Frame* frame) {
        int j = -1;
        for (int i = 0; i < vis_frames_.size(); i++) {
            if (vis_frames_[i] == frame) {
                j = i;
                break;
            }
        }

        vis_frames_.erase(vis_frames_.begin()+j);
        return true;
    }
};

class FeatureManager {
public:
    static int feature_id;

    map<int, Feature*> all_ftr_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    FeatureManager();
    ~FeatureManager();

    bool addNewFeatures(const Image_Type& image_data, Frame* cur_frame, int window_size);
};
