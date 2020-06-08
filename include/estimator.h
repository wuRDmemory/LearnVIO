#pragma once

#include <iostream>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace cv;
using namespace Eigen;

typedef vector<pair<int, double*> > Image_Type;

class Estimator {
public:
    Estimator();
    ~Estimator();

    void processImu(double dt, Vector3f accl, Vector3f gyro);
    void processImage(double timestamp, Image_Type& image);
};
