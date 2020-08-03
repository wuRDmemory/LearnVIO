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

    double inv_d_;
    Vector3d pt3d_;

    vector<Vector3d>  vis_fs_;
    vector<Vector2d>  vis_uv_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
   
    Feature() = delete;
    Feature(int id, int ref_frame_id, Vector3d& f, Vector2d& uv) {
        id_ = id;
        ref_frame_id_ = ref_frame_id;
        vis_fs_.push_back(f);
        vis_uv_.push_back(uv);

        inv_d_ = -1;
        pt3d_ = Vector3d(0, 0, -1);
    }

    int size() const {
        return vis_fs_.size();
    }

    int getRefFrameId() const { 
        return ref_frame_id_;   
    }

    Vector3d getF(int frame_id) const {
        int d = frame_id - ref_frame_id_;
        assert(d >= 0 && d < vis_fs_.size() || "feature manager error!!!");
        return vis_fs_[d];
    }
    
    Vector2d getUV(int frame_id) const {
        int d = frame_id - ref_frame_id_;
        return vis_uv_[d];
    }

    bool contains(int frame_id) const {
        if (frame_id < ref_frame_id_) 
            return false;

        return (frame_id - ref_frame_id_) < (int)vis_fs_.size();
    }

    bool addFrame(const Vector3d& f, const Vector2d& uv) {
        vis_fs_.push_back(f);
        vis_uv_.push_back(uv);

        return true;
    }

    bool removeFrame(int frame_id) {
        int delta = frame_id - ref_frame_id_;

        if (   delta < 0
            || delta >= vis_fs_.size()) {
            return false;
        }

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

    bool clear();

    bool removeNewFrame(int newest_frame_id);

    bool removeOldestFrame();

    bool removeOldestFrame(const Matrix3d &Rcm, const Vector3d &tcm);

    bool addNewFeatures(const Image_Type& image_data, int frame_id);

    // triangle all features
    int  trianglesInitial(Matrix3d Rcw[], Vector3d tcw[]);

    // triangle new features
    int  trianglesNew(Matrix3d Rcw[], Vector3d tcw[]);

    int size() const { return all_ftr_.size(); }

    void toProto(string save_path) const;

    void fromProto(string save_path);

private:
    double computeParallax(Feature* ftr, int frame_id);
};

typedef FeatureManager* FeatureManagerPtr;
