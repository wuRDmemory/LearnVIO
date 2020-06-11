#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"


using namespace std;
using namespace cv;
using namespace Eigen;

typedef pair<int, vector<double> >            Elem_Type;
typedef map<int, pair<int, vector<double> > > Image_Type;

class Feature {
public:
    int id_;
    int ref_frame_id_;
    
    vector<Vector3f>  vis_fs_;
    vector<Vector2f>  vis_uv_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   
    Feature() = delete;
    Feature(int id, int ref_frame_id, Vector3f& f, Vector2f& uv) {
        id_ = id;
        ref_frame_id_ = ref_frame_id;
        vis_fs_.push_back(f);
        vis_uv_.push_back(uv);
    }

    int size() const {
        return vis_fs_.size();
    }

    int getRefFrameId() { 
        return ref_frame_id_;   
    }

    Vector3f getF(int frame_id) {
        int d = frame_id - ref_frame_id_;
        return vis_fs_[d];
    }
    
    Vector2f getUV(int frame_id) {
        int d = frame_id - ref_frame_id_;
        return vis_uv_[d];
    }

    bool contains(int frame_id) {
        if (frame_id < ref_frame_id_) 
            return false;

        return (frame_id - ref_frame_id_) < (int)vis_fs_.size();
    }

    bool addFrame(const Vector3f& f, const Vector2f& uv) {
        vis_fs_.push_back(f);
        vis_uv_.push_back(uv);

        return true;
    }

    bool removeFrame(int frame_id) {
        int delta = frame_id - ref_frame_id_;

        if (delta < vis_fs_.size()) {
            return false;
        }

        // TODO: how to remove it
        vis_fs_.erase(vis_fs_.begin()+delta);
        vis_uv_.erase(vis_uv_.begin()+delta);
        return true;
    }
};

class FeatureManager {
public:
    map<int, Feature*> all_ftr_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    FeatureManager();
    ~FeatureManager();

    int size() const { return all_ftr_.size(); }

    bool addNewFeatures(const Image_Type& image_data, int frame_id);


private:
    float computeParallax(Feature* ftr, int frame_id);
};

typedef FeatureManager* FeatureManagerPtr;
