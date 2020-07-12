#pragma once

#include <iostream>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "../include/visual/feature.h"
#include "../include/inertial/preintegrate.h"
#include "../include/initial/initial.h"
#include "../include/visual/globalsfm.h"
#include "../include/util/config.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class Estimator {
public:
    
    bool first_imu_;
    bool initial_;
    
    int frame_count_;

    FeatureManager feature_manager_;

    // this is for initial
    // do not use preintegrates to initial because the initial maybe failed
    PreIntegrate* temp_preintegrate_;

    vector<PreIntegrate*> preintegrates_;

    vector<double>      timestamp_;
    vector<Quaternionf> RS_;   // world is real world, RS[i] mean w_R_bk
    vector<Vector3f>    VS_;
    vector<Vector3f>    PS_;
    vector<Vector3f>    BAS_;
    vector<Vector3f>    BGS_;

    map<double, FrameStruct> all_frames_;
    Vector3f accl_0_;
    Vector3f gyro_0_;

    // optimize variables
    double pose_params[FEN_WINDOW_SIZE+1][POSE_SIZE]; 
    double motion_params[FEN_WINDOW_SIZE+1][MOTION_SIZE];
    double point_params[1000];

public:
    Estimator();
    ~Estimator();

    void processImu(double dt, Vector3f accl, Vector3f gyro);

    void processImage(double timestamp, Image_Type& image);

    void clearState();

private:
    bool solveOdometry();

    bool structInitial();

    bool slideWindow(bool margin_old);

    bool slideOldFrame();

    bool slideNewFrame();

    bool solveNewFrame(map<int, Feature*> &all_features, Matrix3f Rcw, Vector3f tcw);

    void vector2double();

    void double2vector();

    void solveOptimize();
};
