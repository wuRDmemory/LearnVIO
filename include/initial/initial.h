#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <map>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include "../visual/feature.h"
#include "../inertial/preintegrate.h"

using namespace std;
using namespace cv;
using namespace Eigen;



int relativeRT(map<int, Feature*>& all_ftr, Matrix3f& Rcr, Vector3f& tcr, int window_size);

