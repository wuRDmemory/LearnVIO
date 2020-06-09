#pragma once

#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include <memory>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

using namespace std;
using namespace cv;
using namespace Eigen;

class Feature;

// b: body coordination(IMU)
// w: world coordination
// c: camera coordination
class Frame {
public:
    static int id;
    int id_;

    Quaternionf Rwb_;
    Vector3f    Pwb_;
    Vector3f    Vw_;

    Vector3f    Biasa_;
    Vector3f    Biasw_;

    list<Feature*> vis_ftr_;

public:
    Frame();
    ~Frame();

    bool removeFeature(Feature* ftr);    
};

typedef shared_ptr<Frame> FramePtr;
