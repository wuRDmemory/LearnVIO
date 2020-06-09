#pragma once

#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <memory>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "../inertial/preintegrate.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class Feature;

// b: body coordination(IMU)
// w: world coordination
// c: camera coordination
class Frame {
public:
    int id_;

    Quaternionf Rwb_;
    Vector3f    Pwb_;
    Vector3f    Vw_;

    Vector3f    Biasa_;
    Vector3f    Biasw_;

    list<Feature*> vis_ftr_;

    PreIntegrate* integration_;

public:
    Frame() = delete;
    Frame(int id);
    ~Frame();

    Frame(const Frame& frame);

    bool addFeature(Feature* ftr);

    bool removeFeature(Feature* ftr);    

    bool integrate(double dt, const Vector3f& accl, const Vector3f& gyro);
};

typedef shared_ptr<Frame> FramePtr;
