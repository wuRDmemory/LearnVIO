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

using namespace std;
using namespace cv;
using namespace Eigen;

class Estimator {
private:
    
    bool first_imu_;
    bool initial_;
    
    int frame_count_;

    FeatureManager feature_manager_;

    // this is for initial
    // do not use preintegrates to initial because the initial maybe failed
    PreIntegrate* temp_preintegrate_;

    vector<PreIntegrate*> preintegrates_;

    vector<Quaternionf> RS_;
    vector<Vector3f> VS_;
    vector<Vector3f> PS_;
    vector<Vector3f> BAS_;
    vector<Vector3f> BGS_;

    map<double, FrameStruct> all_frames_;

    vector<double> timestamp_;

    Vector3f g_;
    Vector3f accl_0_, gyro_0_;

public:
    Estimator();
    ~Estimator();

    void processImu(double dt, Vector3f accl, Vector3f gyro);

    void processImage(double timestamp, Image_Type& image);

    void clearState();

private:
    bool structInitial();

};
