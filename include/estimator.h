#pragma once

#include <iostream>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "../include/inertial/preintegrate.h"

using namespace std;
using namespace cv;
using namespace Eigen;

typedef vector<pair<int, double*> > Image_Type;

class Estimator {
private:
    
    bool first_imu_;
    
    int frame_count_;

    vector<PreIntegrate*> preintegrates_;

    vector<Quaternionf> RS_;
    vector<Vector3f> VS_;
    vector<Vector3f> PS_;
    vector<Vector3f> BAS_;
    vector<Vector3f> BGS_;

    Vector3f g_;

    Vector3f accl_0_, gyro_0_;

public:
    Estimator();
    ~Estimator();

    void processImu(double dt, Vector3f accl, Vector3f gyro);

    void processImage(double timestamp, Image_Type& image);

    void clearState();
};
